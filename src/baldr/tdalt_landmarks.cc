#include "baldr/tdalt_landmarks.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace valhalla {
namespace baldr {
namespace {

constexpr size_t kGraphIdPackedSize = 8;

inline uint64_t graph_id_value(uint32_t lo, uint32_t hi) {
  return (static_cast<uint64_t>(hi) << 32) | lo;
}

inline void unpack_graph_id(uint64_t value, uint32_t* lo, uint32_t* hi) {
  *lo = static_cast<uint32_t>(value & 0xffffffffu);
  *hi = static_cast<uint32_t>(value >> 32);
}

bool header_valid(const TdaltLandmarksHeader* header, size_t file_size) {
  if (header == nullptr) {
    return false;
  }
  if (header->magic != kTdaltLandmarksMagic || header->version != kTdaltLandmarksVersion) {
    return false;
  }
  if (header->landmark_count == 0) {
    return false;
  }
  if (header->distances_offset < sizeof(TdaltLandmarksHeader) || header->distances_offset > file_size) {
    return false;
  }

  const size_t record_stride = kGraphIdPackedSize + 2 * header->landmark_count * sizeof(float);
  const size_t records_size = static_cast<size_t>(header->node_count) * record_stride;
  if (header->distances_offset + records_size > file_size) {
    return false;
  }
  return true;
}

} // namespace

TDALTLandmarkIndex::TDALTLandmarkIndex(const std::string& path) {
  reset();

  fd_ = ::open(path.c_str(), O_RDONLY);
  if (fd_ < 0) {
    return;
  }

  struct stat st {};
  if (::fstat(fd_, &st) != 0 || st.st_size < static_cast<off_t>(sizeof(TdaltLandmarksHeader))) {
    ::close(fd_);
    fd_ = -1;
    return;
  }

  mapping_size_ = static_cast<size_t>(st.st_size);
  mapping_ = ::mmap(nullptr, mapping_size_, PROT_READ, MAP_PRIVATE, fd_, 0);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    mapping_size_ = 0;
    ::close(fd_);
    fd_ = -1;
    return;
  }

  data_ = static_cast<const char*>(mapping_);
  data_size_ = mapping_size_;
  header_ = reinterpret_cast<const TdaltLandmarksHeader*>(data_);
  if (!header_valid(header_, data_size_)) {
    reset();
    return;
  }

  landmark_count_ = header_->landmark_count;
  record_stride_ = kGraphIdPackedSize + 2 * landmark_count_ * sizeof(float);
}

TDALTLandmarkIndex::~TDALTLandmarkIndex() {
  reset();
}

TDALTLandmarkIndex::TDALTLandmarkIndex(TDALTLandmarkIndex&& other) noexcept
    : header_(other.header_), data_(other.data_), data_size_(other.data_size_),
      landmark_count_(other.landmark_count_), record_stride_(other.record_stride_),
      mapping_(other.mapping_), mapping_size_(other.mapping_size_), fd_(other.fd_) {
  other.header_ = nullptr;
  other.data_ = nullptr;
  other.data_size_ = 0;
  other.landmark_count_ = 0;
  other.record_stride_ = 0;
  other.mapping_ = nullptr;
  other.mapping_size_ = 0;
  other.fd_ = -1;
}

TDALTLandmarkIndex& TDALTLandmarkIndex::operator=(TDALTLandmarkIndex&& other) noexcept {
  if (this != &other) {
    reset();
    header_ = other.header_;
    data_ = other.data_;
    data_size_ = other.data_size_;
    landmark_count_ = other.landmark_count_;
    record_stride_ = other.record_stride_;
    mapping_ = other.mapping_;
    mapping_size_ = other.mapping_size_;
    fd_ = other.fd_;

    other.header_ = nullptr;
    other.data_ = nullptr;
    other.data_size_ = 0;
    other.landmark_count_ = 0;
    other.record_stride_ = 0;
    other.mapping_ = nullptr;
    other.mapping_size_ = 0;
    other.fd_ = -1;
  }
  return *this;
}

void TDALTLandmarkIndex::reset() {
  if (mapping_ != nullptr && mapping_ != MAP_FAILED) {
    ::munmap(mapping_, mapping_size_);
  }
  if (fd_ >= 0) {
    ::close(fd_);
  }
  header_ = nullptr;
  data_ = nullptr;
  data_size_ = 0;
  landmark_count_ = 0;
  record_stride_ = 0;
  mapping_ = nullptr;
  mapping_size_ = 0;
  fd_ = -1;
}

bool TDALTLandmarkIndex::available() const {
  return header_ != nullptr;
}

const char* TDALTLandmarkIndex::find_record(const GraphId& node) const {
  if (!available() || header_->node_count == 0) {
    return nullptr;
  }

  uint32_t target_lo = 0;
  uint32_t target_hi = 0;
  unpack_graph_id(node.value, &target_lo, &target_hi);

  const char* records = data_ + header_->distances_offset;
  size_t lo = 0;
  size_t hi = header_->node_count;
  while (lo < hi) {
    const size_t mid = lo + (hi - lo) / 2;
    const char* record = records + mid * record_stride_;
    const auto* packed = reinterpret_cast<const uint32_t*>(record);
    const uint64_t mid_value = graph_id_value(packed[0], packed[1]);
    const uint64_t target_value = graph_id_value(target_lo, target_hi);
    if (mid_value < target_value) {
      lo = mid + 1;
    } else if (mid_value > target_value) {
      if (mid == 0) {
        return nullptr;
      }
      hi = mid;
    } else {
      return record;
    }
  }
  return nullptr;
}

float TDALTLandmarkIndex::distance_to(const GraphId& node, uint32_t landmark) const {
  // D_λ(node, L_i): λ-shortest-path distance from node toward landmark L_i.
  if (!available() || landmark >= landmark_count_) {
    return std::numeric_limits<float>::infinity();
  }
  const char* record = find_record(node);
  if (record == nullptr) {
    return std::numeric_limits<float>::infinity();
  }
  const auto* distances = reinterpret_cast<const float*>(record + kGraphIdPackedSize);
  return distances[landmark];
}

float TDALTLandmarkIndex::distance_from(const GraphId& node, uint32_t landmark) const {
  // D_λ(L_i, node): λ-shortest-path distance from landmark L_i to node.
  if (!available() || landmark >= landmark_count_) {
    return std::numeric_limits<float>::infinity();
  }
  const char* record = find_record(node);
  if (record == nullptr) {
    return std::numeric_limits<float>::infinity();
  }
  const auto* distances =
      reinterpret_cast<const float*>(record + kGraphIdPackedSize + landmark_count_ * sizeof(float));
  return distances[landmark];
}

float TDALTLandmarkIndex::potential_to_target(const GraphId& u, const GraphId& t) const {
  // π_f(u): admissible estimate of remaining λ-cost from u to target t (forward A* key).
  float best = 0.f;
  for (uint32_t i = 0; i < landmark_count_; ++i) {
    best = std::max(best, distance_to(u, i) - distance_to(t, i));
    best = std::max(best, distance_from(t, i) - distance_from(u, i));
  }
  return best;
}

float TDALTLandmarkIndex::potential_from_source(const GraphId& u, const GraphId& s) const {
  // π_b(u): admissible estimate of remaining λ-cost from source s to u (backward A* key).
  float best = 0.f;
  for (uint32_t i = 0; i < landmark_count_; ++i) {
    best = std::max(best, distance_to(s, i) - distance_to(u, i));
    best = std::max(best, distance_from(u, i) - distance_from(s, i));
  }
  return best;
}

} // namespace baldr
} // namespace valhalla
