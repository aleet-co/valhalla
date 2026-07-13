#ifndef VALHALLA_BALDR_TDALT_LANDMARKS_H_
#define VALHALLA_BALDR_TDALT_LANDMARKS_H_

#include <valhalla/baldr/graphid.h>

#include <cstdint>
#include <string>

namespace valhalla {
namespace baldr {

// File magic: ASCII "TDAL" (first four characters of "TDALT"), little-endian uint32.
constexpr uint32_t kTdaltLandmarksMagic = 0x4C414454u;
constexpr uint32_t kTdaltLandmarksVersion = 1;

#pragma pack(push, 1)
struct TdaltLandmarksHeader {
  uint32_t magic;
  uint32_t version;
  uint32_t landmark_count;
  uint32_t node_count;
  uint64_t distances_offset;
};
#pragma pack(pop)

static_assert(sizeof(TdaltLandmarksHeader) == 24, "unexpected TdaltLandmarksHeader size");

// Per-node record layout (variable size, landmark_count from header):
//   uint32_t graphid_lo, uint32_t graphid_hi,
//   float dist_to_landmark[L], float dist_from_landmark[L]

/**
 * Read-only mmap index of precomputed ALT landmark distances on G_lambda.
 */
class TDALTLandmarkIndex {
public:
  explicit TDALTLandmarkIndex(const std::string& path);
  ~TDALTLandmarkIndex();

  TDALTLandmarkIndex(const TDALTLandmarkIndex&) = delete;
  TDALTLandmarkIndex& operator=(const TDALTLandmarkIndex&) = delete;
  TDALTLandmarkIndex(TDALTLandmarkIndex&&) noexcept;
  TDALTLandmarkIndex& operator=(TDALTLandmarkIndex&&) noexcept;

  bool available() const;

  float distance_to(const GraphId& node, uint32_t landmark) const;
  float distance_from(const GraphId& node, uint32_t landmark) const;
  float potential_to_target(const GraphId& node, const GraphId& target) const;
  float potential_from_source(const GraphId& node, const GraphId& source) const;

private:
  const TdaltLandmarksHeader* header_{nullptr};
  const char* data_{nullptr};
  size_t data_size_{0};
  uint32_t landmark_count_{0};
  size_t record_stride_{0};
  void* mapping_{nullptr};
  size_t mapping_size_{0};
  int fd_{-1};

  const char* find_record(const GraphId& node) const;
  void reset();
};

} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_TDALT_LANDMARKS_H_
