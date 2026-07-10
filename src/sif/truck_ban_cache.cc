#include "sif/truck_ban_cache.h"

#include <chrono>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

#include <pthread.h>

namespace valhalla {
namespace sif {
namespace truck_ban {
namespace {

constexpr const char* kDefaultCachePath = "/custom_files/cache/truck_ban_cache.bin";
constexpr const char* kDefaultDecisionCachePath = "/custom_files/cache/truck_ban_decision_cache.mmap";
constexpr const char* kDefaultTraverseCachePath = "/custom_files/cache/truck_ban_traverse_cache.mmap";
constexpr int64_t kEntryLifetimeSeconds = 7LL * 24LL * 3600LL;
constexpr uint32_t kSnapshotVersion = 1;
constexpr std::chrono::seconds kFlushInterval{60};
constexpr uint64_t kTraverseChecksLogInterval = 100000;

constexpr char kMagic[4] = {'T', 'B', 'C', 'H'};

struct FileHeader {
  char magic[4];
  uint32_t version;
  uint32_t entry_count;
  uint32_t reserved;
};

struct FileEntry {
  uint64_t key;
  uint8_t allowed;
  uint8_t padding[7];
  int64_t expires_at;
};

static_assert(sizeof(FileHeader) == 16, "unexpected FileHeader size");
static_assert(sizeof(FileEntry) == 24, "unexpected FileEntry size");

std::atomic<uint64_t> g_traverse_checks{0};
std::atomic<uint64_t> g_traverse_hits{0};
std::atomic<uint64_t> g_traverse_misses{0};
std::atomic<uint64_t> g_traverse_stores{0};
std::atomic<uint64_t> g_decision_hits{0};
std::atomic<uint64_t> g_decision_misses{0};
std::atomic<uint64_t> g_decision_stores{0};
std::atomic<uint64_t> g_local_hour_hits{0};
std::atomic<uint64_t> g_local_hour_misses{0};
std::atomic<uint64_t> g_last_logged_traverse_checks{0};

struct RouteEndpoints {
  double origin_lat{0.0};
  double origin_lon{0.0};
  double dest_lat{0.0};
  double dest_lon{0.0};
  bool valid{false};
};

std::mutex g_route_log_mutex;
RouteEndpoints g_active_route{};
RouteEndpoints g_last_route{};

bool IsMotorwayOrTrunk(baldr::RoadClass road_class) {
  return road_class == baldr::RoadClass::kMotorway || road_class == baldr::RoadClass::kTrunk;
}

int64_t NowEpoch() {
  return static_cast<int64_t>(std::time(nullptr));
}

const char* CachePathFromEnv() {
  const char* configured = std::getenv("TRUCK_BAN_CACHE_PATH");
  if (configured != nullptr && configured[0] != '\0') {
    return configured;
  }
  return kDefaultCachePath;
}

const char* TraverseCachePathFromEnv() {
  const char* configured = std::getenv("TRUCK_BAN_TRAVERSE_CACHE_PATH");
  if (configured != nullptr && configured[0] != '\0') {
    return configured;
  }
  return kDefaultTraverseCachePath;
}

const char* DecisionCachePathFromEnv() {
  const char* configured = std::getenv("TRUCK_BAN_DECISION_CACHE_PATH");
  if (configured != nullptr && configured[0] != '\0') {
    return configured;
  }
  return kDefaultDecisionCachePath;
}

bool PersistenceDisabled() {
  const char* flag = std::getenv("TRUCK_BAN_CACHE_ENABLED");
  return flag != nullptr &&
         (flag[0] == '0' || flag[0] == 'f' || flag[0] == 'F' || flag[0] == 'n' || flag[0] == 'N');
}

bool StatsLoggingEnabled() {
  const char* flag = std::getenv("TRUCK_BAN_CACHE_LOG_STATS");
  return flag != nullptr &&
         (flag[0] == '1' || flag[0] == 't' || flag[0] == 'T' || flag[0] == 'y' || flag[0] == 'Y');
}

bool PeriodicStatsLoggingEnabled() {
  if (!StatsLoggingEnabled()) {
    return false;
  }
  const char* flag = std::getenv("TRUCK_BAN_CACHE_LOG_PERIODIC");
  if (flag == nullptr) {
    return true;
  }
  return flag[0] == '1' || flag[0] == 't' || flag[0] == 'T' || flag[0] == 'y' || flag[0] == 'Y';
}

void SetActiveRouteEndpoints(double origin_lat,
                             double origin_lon,
                             double dest_lat,
                             double dest_lon) {
  std::lock_guard lock(g_route_log_mutex);
  g_active_route = RouteEndpoints{
      .origin_lat = origin_lat,
      .origin_lon = origin_lon,
      .dest_lat = dest_lat,
      .dest_lon = dest_lon,
      .valid = true,
  };
}

void ClearActiveRouteEndpoints() {
  std::lock_guard lock(g_route_log_mutex);
  g_active_route.valid = false;
}

void AppendRouteContext(std::ostream& out) {
  std::lock_guard lock(g_route_log_mutex);
  const RouteEndpoints* endpoints = nullptr;
  const char* role = nullptr;
  if (g_active_route.valid) {
    endpoints = &g_active_route;
    role = "active";
  } else if (g_last_route.valid) {
    endpoints = &g_last_route;
    role = "last";
  }
  if (endpoints == nullptr) {
    return;
  }
  const auto format = [](double value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4) << value;
    return stream.str();
  };
  out << " route_" << role << "_origin=" << format(endpoints->origin_lat) << ','
      << format(endpoints->origin_lon) << " route_" << role << "_dest="
      << format(endpoints->dest_lat) << ',' << format(endpoints->dest_lon);
}

bool HeaderValid(const FileHeader& header) {
  return std::memcmp(header.magic, kMagic, sizeof(kMagic)) == 0 && header.version == kSnapshotVersion;
}

const char* CacheModeName(CacheMode mode) {
  switch (mode) {
  case CacheMode::kOff:
    return "off";
  case CacheMode::kDecisionOnly:
    return "decision";
  case CacheMode::kFull:
    return "full";
  }
  return "unknown";
}

void MaybeLogTraverseInterval() {
  if (!PeriodicStatsLoggingEnabled()) {
    return;
  }
  const uint64_t checks = g_traverse_checks.load();
  if (checks == 0 || checks % kTraverseChecksLogInterval != 0) {
    return;
  }
  LogCacheStats("interval");
}

struct alignas(64) SharedRwLock {
  pthread_rwlock_t lock;
};

size_t AlignUp(size_t value, size_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

void InitSharedRwLock(SharedRwLock* shared_lock) {
  pthread_rwlockattr_t attr;
  pthread_rwlockattr_init(&attr);
  pthread_rwlockattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
  pthread_rwlock_init(&shared_lock->lock, &attr);
  pthread_rwlockattr_destroy(&attr);
}

constexpr char kDecisionMagic[4] = {'T', 'B', 'D', 'C'};
constexpr uint32_t kDecisionVersion = 1;
constexpr uint32_t kDecisionShardCount = 64;
constexpr uint32_t kDecisionSlotsPerShard = 4096;
constexpr uint32_t kDecisionMaxProbes = 16;

struct DecisionSlot {
  uint64_t key{0};
  int64_t expires_at{0};
  uint8_t occupied{0};
  uint8_t allowed{0};
  uint8_t reserved[6]{};
};

struct DecisionFileHeader {
  char magic[4];
  uint32_t version;
  uint32_t shard_count;
  uint32_t slots_per_shard;
  uint32_t reserved;
};

size_t DecisionShardStride() {
  return AlignUp(sizeof(SharedRwLock), 64) +
         static_cast<size_t>(kDecisionSlotsPerShard) * sizeof(DecisionSlot);
}

size_t DecisionFileSize() {
  return AlignUp(sizeof(DecisionFileHeader), 64) +
         static_cast<size_t>(kDecisionShardCount) * DecisionShardStride();
}

bool DecisionHeaderValid(const DecisionFileHeader& header) {
  return std::memcmp(header.magic, kDecisionMagic, sizeof(kDecisionMagic)) == 0 &&
         header.version == kDecisionVersion && header.shard_count == kDecisionShardCount &&
         header.slots_per_shard == kDecisionSlotsPerShard;
}

class SharedDecisionCacheMap {
public:
  static std::unique_ptr<SharedDecisionCacheMap> Open(const char* path) {
    auto store = std::unique_ptr<SharedDecisionCacheMap>(new SharedDecisionCacheMap());
    if (!store->initialize(path)) {
      return nullptr;
    }
    return store;
  }

  ~SharedDecisionCacheMap() {
    if (mapping_ != nullptr && mapping_ != MAP_FAILED) {
      munmap(mapping_, mapping_size_);
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  int file_descriptor() const {
    return fd_;
  }

  bool newly_initialized() const {
    return newly_initialized_;
  }

  std::optional<bool> lookup(uint64_t key, int64_t now) const {
    const uint32_t shard_id = shard_for_key(key);
    SharedRwLock* shard_lock = shard_lock_at(shard_id);
    DecisionSlot* slots = shard_slots(shard_id);

    pthread_rwlock_rdlock(&shard_lock->lock);
    const DecisionSlot* found = find_slot(slots, key);
    const std::optional<bool> result =
        found != nullptr && found->expires_at > now ? std::optional<bool>(found->allowed != 0)
                                                  : std::nullopt;
    pthread_rwlock_unlock(&shard_lock->lock);
    return result;
  }

  void store(uint64_t key, bool allowed, int64_t expires_at) {
    const uint32_t shard_id = shard_for_key(key);
    SharedRwLock* shard_lock = shard_lock_at(shard_id);
    DecisionSlot* slots = shard_slots(shard_id);

    pthread_rwlock_wrlock(&shard_lock->lock);
    DecisionSlot* slot = find_slot(slots, key);
    if (slot != nullptr) {
      slot->allowed = allowed ? 1 : 0;
      slot->expires_at = expires_at;
      pthread_rwlock_unlock(&shard_lock->lock);
      return;
    }

    slot = find_empty_slot(slots, key);
    if (slot != nullptr) {
      slot->key = key;
      slot->allowed = allowed ? 1 : 0;
      slot->expires_at = expires_at;
      slot->occupied = 1;
    }
    pthread_rwlock_unlock(&shard_lock->lock);
  }

  void clear() {
    for (uint32_t shard_id = 0; shard_id < kDecisionShardCount; ++shard_id) {
      SharedRwLock* shard_lock = shard_lock_at(shard_id);
      DecisionSlot* slots = shard_slots(shard_id);
      pthread_rwlock_wrlock(&shard_lock->lock);
      for (uint32_t slot_index = 0; slot_index < kDecisionSlotsPerShard; ++slot_index) {
        slots[slot_index].occupied = 0;
      }
      pthread_rwlock_unlock(&shard_lock->lock);
    }
  }

  bool empty() const {
    for (uint32_t shard_id = 0; shard_id < kDecisionShardCount; ++shard_id) {
      SharedRwLock* shard_lock = shard_lock_at(shard_id);
      const DecisionSlot* slots = shard_slots(shard_id);
      pthread_rwlock_rdlock(&shard_lock->lock);
      for (uint32_t slot_index = 0; slot_index < kDecisionSlotsPerShard; ++slot_index) {
        if (slots[slot_index].occupied != 0) {
          pthread_rwlock_unlock(&shard_lock->lock);
          return false;
        }
      }
      pthread_rwlock_unlock(&shard_lock->lock);
    }
    return true;
  }

  template <typename Visitor>
  void for_each_entry(int64_t now, Visitor&& visitor) const {
    for (uint32_t shard_id = 0; shard_id < kDecisionShardCount; ++shard_id) {
      SharedRwLock* shard_lock = shard_lock_at(shard_id);
      const DecisionSlot* slots = shard_slots(shard_id);
      pthread_rwlock_rdlock(&shard_lock->lock);
      for (uint32_t slot_index = 0; slot_index < kDecisionSlotsPerShard; ++slot_index) {
        const DecisionSlot& slot = slots[slot_index];
        if (slot.occupied != 0 && slot.expires_at > now) {
          visitor(slot.key, slot.allowed != 0, slot.expires_at);
        }
      }
      pthread_rwlock_unlock(&shard_lock->lock);
    }
  }

private:
  SharedDecisionCacheMap() = default;

  int fd_{-1};
  void* mapping_{nullptr};
  size_t mapping_size_{0};
  DecisionFileHeader* header_{nullptr};
  bool newly_initialized_{false};

  bool initialize(const char* path) {
    try {
      const std::filesystem::path mapped_path(path);
      if (mapped_path.has_parent_path()) {
        std::filesystem::create_directories(mapped_path.parent_path());
      }
    } catch (...) {
      return false;
    }

    mapping_size_ = DecisionFileSize();
    fd_ = open(path, O_RDWR | O_CREAT, 0666);
    if (fd_ < 0) {
      return false;
    }

    struct flock init_lock {};
    init_lock.l_type = F_WRLCK;
    init_lock.l_whence = SEEK_SET;
    if (fcntl(fd_, F_SETLKW, &init_lock) != 0) {
      close(fd_);
      fd_ = -1;
      return false;
    }

    struct stat stat_buf {};
    if (fstat(fd_, &stat_buf) != 0) {
      init_lock.l_type = F_UNLCK;
      fcntl(fd_, F_SETLK, &init_lock);
      close(fd_);
      fd_ = -1;
      return false;
    }

    if (stat_buf.st_size != static_cast<off_t>(mapping_size_)) {
      if (ftruncate(fd_, static_cast<off_t>(mapping_size_)) != 0) {
        init_lock.l_type = F_UNLCK;
        fcntl(fd_, F_SETLK, &init_lock);
        close(fd_);
        fd_ = -1;
        return false;
      }
    }

    mapping_ = mmap(nullptr, mapping_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED) {
      mapping_ = nullptr;
      init_lock.l_type = F_UNLCK;
      fcntl(fd_, F_SETLK, &init_lock);
      close(fd_);
      fd_ = -1;
      return false;
    }

    header_ = reinterpret_cast<DecisionFileHeader*>(mapping_);
    if (!DecisionHeaderValid(*header_)) {
      newly_initialized_ = true;
      std::memset(mapping_, 0, mapping_size_);
      std::memcpy(header_->magic, kDecisionMagic, sizeof(kDecisionMagic));
      header_->version = kDecisionVersion;
      header_->shard_count = kDecisionShardCount;
      header_->slots_per_shard = kDecisionSlotsPerShard;
      for (uint32_t shard_id = 0; shard_id < kDecisionShardCount; ++shard_id) {
        InitSharedRwLock(shard_lock_at(shard_id));
      }
    }

    init_lock.l_type = F_UNLCK;
    fcntl(fd_, F_SETLK, &init_lock);
    return true;
  }

  uint32_t shard_for_key(uint64_t key) const {
    return static_cast<uint32_t>(std::hash<uint64_t>{}(key) % kDecisionShardCount);
  }

  size_t shard_offset(uint32_t shard_id) const {
    return AlignUp(sizeof(DecisionFileHeader), 64) + static_cast<size_t>(shard_id) * DecisionShardStride();
  }

  SharedRwLock* shard_lock_at(uint32_t shard_id) const {
    return reinterpret_cast<SharedRwLock*>(static_cast<char*>(mapping_) + shard_offset(shard_id));
  }

  DecisionSlot* shard_slots(uint32_t shard_id) const {
    return reinterpret_cast<DecisionSlot*>(reinterpret_cast<char*>(shard_lock_at(shard_id)) +
                                           AlignUp(sizeof(SharedRwLock), 64));
  }

  static DecisionSlot* find_slot(DecisionSlot* slots, uint64_t key) {
    const uint32_t start = static_cast<uint32_t>(std::hash<uint64_t>{}(key) % kDecisionSlotsPerShard);
    for (uint32_t probe = 0; probe < kDecisionMaxProbes; ++probe) {
      DecisionSlot& slot = slots[(start + probe) % kDecisionSlotsPerShard];
      if (slot.occupied == 0) {
        return nullptr;
      }
      if (slot.key == key) {
        return &slot;
      }
    }
    return nullptr;
  }

  static DecisionSlot* find_empty_slot(DecisionSlot* slots, uint64_t key) {
    const uint32_t start = static_cast<uint32_t>(std::hash<uint64_t>{}(key) % kDecisionSlotsPerShard);
    for (uint32_t probe = 0; probe < kDecisionMaxProbes; ++probe) {
      DecisionSlot& slot = slots[(start + probe) % kDecisionSlotsPerShard];
      if (slot.occupied == 0) {
        return &slot;
      }
    }
    return nullptr;
  }
};

struct DecisionMemoryEntry {
  bool allowed;
  int64_t expires_at;
};

struct TraverseKey {
  uint64_t from{0};
  uint64_t to{0};
  uint32_t depart_hour{0};
  uint32_t arrive_hour{0};
  uint8_t motorway{0};

  bool operator==(const TraverseKey& other) const {
    return from == other.from && to == other.to && depart_hour == other.depart_hour &&
           arrive_hour == other.arrive_hour && motorway == other.motorway;
  }
};

struct TraverseKeyHash {
  size_t operator()(const TraverseKey& key) const {
    size_t hash = std::hash<uint64_t>{}(key.from);
    hash ^= std::hash<uint64_t>{}(key.to) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<uint32_t>{}(key.depart_hour) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<uint32_t>{}(key.arrive_hour) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<uint8_t>{}(key.motorway) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    return hash;
  }
};

TraverseKey MakeTraverseKey(const baldr::GraphId& from_node,
                            const baldr::GraphId& to_node,
                            uint64_t depart_time,
                            uint64_t arrive_time,
                            baldr::RoadClass road_class) {
  return TraverseKey{
      .from = from_node.value,
      .to = to_node.value,
      .depart_hour = static_cast<uint32_t>(depart_time / 3600ULL),
      .arrive_hour = static_cast<uint32_t>(arrive_time / 3600ULL),
      .motorway = static_cast<uint8_t>(IsMotorwayOrTrunk(road_class) ? 1 : 0),
  };
}

constexpr char kTraverseMagic[4] = {'T', 'B', 'T', 'R'};
constexpr uint32_t kTraverseVersion = 1;
constexpr uint32_t kTraverseShardCount = 256;
constexpr uint32_t kTraverseSlotsPerShard = 16384;
constexpr uint32_t kTraverseMaxProbes = 32;

struct TraverseSlot {
  uint64_t from{0};
  uint64_t to{0};
  uint32_t depart_hour{0};
  uint32_t arrive_hour{0};
  uint8_t motorway{0};
  uint8_t occupied{0};
  uint8_t allowed{0};
  uint8_t reserved[5]{};
};

struct TraverseFileHeader {
  char magic[4];
  uint32_t version;
  uint32_t shard_count;
  uint32_t slots_per_shard;
  uint32_t reserved;
};

size_t TraverseShardStride() {
  return AlignUp(sizeof(SharedRwLock), 64) +
         static_cast<size_t>(kTraverseSlotsPerShard) * sizeof(TraverseSlot);
}

size_t TraverseFileSize() {
  return AlignUp(sizeof(TraverseFileHeader), 64) +
         static_cast<size_t>(kTraverseShardCount) * TraverseShardStride();
}

bool TraverseHeaderValid(const TraverseFileHeader& header) {
  return std::memcmp(header.magic, kTraverseMagic, sizeof(kTraverseMagic)) == 0 &&
         header.version == kTraverseVersion && header.shard_count == kTraverseShardCount &&
         header.slots_per_shard == kTraverseSlotsPerShard;
}

class SharedTraverseCacheMap {
public:
  static std::unique_ptr<SharedTraverseCacheMap> Open(const char* path) {
    auto store = std::unique_ptr<SharedTraverseCacheMap>(new SharedTraverseCacheMap());
    if (!store->initialize(path)) {
      return nullptr;
    }
    return store;
  }

  ~SharedTraverseCacheMap() {
    if (mapping_ != nullptr && mapping_ != MAP_FAILED) {
      munmap(mapping_, mapping_size_);
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  std::optional<bool> lookup(const TraverseKey& key) const {
    const uint32_t shard_id = shard_for_key(key);
    SharedRwLock* shard_lock = shard_lock_at(shard_id);
    TraverseSlot* slots = shard_slots(shard_id);

    pthread_rwlock_rdlock(&shard_lock->lock);
    const TraverseSlot* found = find_slot(slots, key);
    const std::optional<bool> result =
        found != nullptr ? std::optional<bool>(found->allowed != 0) : std::nullopt;
    pthread_rwlock_unlock(&shard_lock->lock);
    return result;
  }

  void store(const TraverseKey& key, bool allowed) {
    const uint32_t shard_id = shard_for_key(key);
    SharedRwLock* shard_lock = shard_lock_at(shard_id);
    TraverseSlot* slots = shard_slots(shard_id);

    pthread_rwlock_wrlock(&shard_lock->lock);
    TraverseSlot* slot = find_slot(slots, key);
    if (slot != nullptr) {
      slot->allowed = allowed ? 1 : 0;
      pthread_rwlock_unlock(&shard_lock->lock);
      return;
    }

    slot = find_empty_slot(slots, key);
    if (slot != nullptr) {
      slot->from = key.from;
      slot->to = key.to;
      slot->depart_hour = key.depart_hour;
      slot->arrive_hour = key.arrive_hour;
      slot->motorway = key.motorway;
      slot->allowed = allowed ? 1 : 0;
      slot->occupied = 1;
    }
    pthread_rwlock_unlock(&shard_lock->lock);
  }

  void clear() {
    for (uint32_t shard_id = 0; shard_id < kTraverseShardCount; ++shard_id) {
      SharedRwLock* shard_lock = shard_lock_at(shard_id);
      TraverseSlot* slots = shard_slots(shard_id);
      pthread_rwlock_wrlock(&shard_lock->lock);
      for (uint32_t slot_index = 0; slot_index < kTraverseSlotsPerShard; ++slot_index) {
        slots[slot_index].occupied = 0;
      }
      pthread_rwlock_unlock(&shard_lock->lock);
    }
  }

private:
  SharedTraverseCacheMap() = default;

  int fd_{-1};
  void* mapping_{nullptr};
  size_t mapping_size_{0};
  TraverseFileHeader* header_{nullptr};

  bool initialize(const char* path) {
    try {
      const std::filesystem::path mapped_path(path);
      if (mapped_path.has_parent_path()) {
        std::filesystem::create_directories(mapped_path.parent_path());
      }
    } catch (...) {
      return false;
    }

    mapping_size_ = TraverseFileSize();
    fd_ = open(path, O_RDWR | O_CREAT, 0666);
    if (fd_ < 0) {
      return false;
    }

    struct flock init_lock {};
    init_lock.l_type = F_WRLCK;
    init_lock.l_whence = SEEK_SET;
    if (fcntl(fd_, F_SETLKW, &init_lock) != 0) {
      close(fd_);
      fd_ = -1;
      return false;
    }

    struct stat stat_buf {};
    if (fstat(fd_, &stat_buf) != 0) {
      init_lock.l_type = F_UNLCK;
      fcntl(fd_, F_SETLK, &init_lock);
      close(fd_);
      fd_ = -1;
      return false;
    }

    if (stat_buf.st_size != static_cast<off_t>(mapping_size_)) {
      if (ftruncate(fd_, static_cast<off_t>(mapping_size_)) != 0) {
        init_lock.l_type = F_UNLCK;
        fcntl(fd_, F_SETLK, &init_lock);
        close(fd_);
        fd_ = -1;
        return false;
      }
    }

    mapping_ = mmap(nullptr, mapping_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapping_ == MAP_FAILED) {
      mapping_ = nullptr;
      init_lock.l_type = F_UNLCK;
      fcntl(fd_, F_SETLK, &init_lock);
      close(fd_);
      fd_ = -1;
      return false;
    }

    header_ = reinterpret_cast<TraverseFileHeader*>(mapping_);
    if (!TraverseHeaderValid(*header_)) {
      std::memset(mapping_, 0, mapping_size_);
      std::memcpy(header_->magic, kTraverseMagic, sizeof(kTraverseMagic));
      header_->version = kTraverseVersion;
      header_->shard_count = kTraverseShardCount;
      header_->slots_per_shard = kTraverseSlotsPerShard;
      for (uint32_t shard_id = 0; shard_id < kTraverseShardCount; ++shard_id) {
        InitSharedRwLock(shard_lock_at(shard_id));
      }
    }

    init_lock.l_type = F_UNLCK;
    fcntl(fd_, F_SETLK, &init_lock);
    return true;
  }

  uint32_t shard_for_key(const TraverseKey& key) const {
    return static_cast<uint32_t>(TraverseKeyHash{}(key) % kTraverseShardCount);
  }

  size_t shard_offset(uint32_t shard_id) const {
    return AlignUp(sizeof(TraverseFileHeader), 64) + static_cast<size_t>(shard_id) * TraverseShardStride();
  }

  SharedRwLock* shard_lock_at(uint32_t shard_id) const {
    return reinterpret_cast<SharedRwLock*>(static_cast<char*>(mapping_) + shard_offset(shard_id));
  }

  TraverseSlot* shard_slots(uint32_t shard_id) const {
    return reinterpret_cast<TraverseSlot*>(reinterpret_cast<char*>(shard_lock_at(shard_id)) +
                                           AlignUp(sizeof(SharedRwLock), 64));
  }

  static bool slot_matches(const TraverseSlot& slot, const TraverseKey& key) {
    return slot.occupied != 0 && slot.from == key.from && slot.to == key.to &&
           slot.depart_hour == key.depart_hour && slot.arrive_hour == key.arrive_hour &&
           slot.motorway == key.motorway;
  }

  static TraverseSlot* find_slot(TraverseSlot* slots, const TraverseKey& key) {
    const uint32_t start = static_cast<uint32_t>(TraverseKeyHash{}(key) % kTraverseSlotsPerShard);
    for (uint32_t probe = 0; probe < kTraverseMaxProbes; ++probe) {
      TraverseSlot& slot = slots[(start + probe) % kTraverseSlotsPerShard];
      if (slot.occupied == 0) {
        return nullptr;
      }
      if (slot_matches(slot, key)) {
        return &slot;
      }
    }
    return nullptr;
  }

  static TraverseSlot* find_empty_slot(TraverseSlot* slots, const TraverseKey& key) {
    const uint32_t start = static_cast<uint32_t>(TraverseKeyHash{}(key) % kTraverseSlotsPerShard);
    for (uint32_t probe = 0; probe < kTraverseMaxProbes; ++probe) {
      TraverseSlot& slot = slots[(start + probe) % kTraverseSlotsPerShard];
      if (slot.occupied == 0) {
        return &slot;
      }
    }
    return nullptr;
  }
};

} // namespace

struct BanDecisionCache::Impl {
  std::unique_ptr<SharedDecisionCacheMap> shared;
  std::unordered_map<uint64_t, DecisionMemoryEntry> local_fallback;
  mutable std::shared_mutex fallback_mutex;
  bool use_shared{false};

  std::string snapshot_path;
  bool persistence_enabled_{false};
  std::atomic<bool> dirty_{false};
  std::atomic<bool> shutdown_{false};
  std::thread flush_thread_;
  std::mutex flush_mutex_;
  std::condition_variable flush_cv_;

  Impl() : snapshot_path(CachePathFromEnv()) {
    shared = SharedDecisionCacheMap::Open(DecisionCachePathFromEnv());
    if (shared != nullptr) {
      use_shared = true;
    } else {
      std::cerr << "[truck_ban_cache] warning: failed to open shared decision cache at "
                << DecisionCachePathFromEnv() << "; using per-worker in-memory fallback"
                << std::endl;
    }

    if (!PersistenceDisabled()) {
      init_persistence();
    }
  }

  ~Impl() {
    if (StatsLoggingEnabled()) {
      LogCacheStats("shutdown");
    }
    stop_flush_thread();
    if (persistence_enabled_) {
      write_snapshot();
    }
  }

  void init_persistence() {
    try {
      const std::filesystem::path path(snapshot_path);
      if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
      }
    } catch (...) {
      persistence_enabled_ = false;
      return;
    }

    persistence_enabled_ = true;
    import_snapshot_if_needed();

    shutdown_.store(false);
    flush_thread_ = std::thread([this] { flush_thread_main(); });
  }

  void import_snapshot_if_needed() {
    if (!use_shared) {
      load_snapshot_into_fallback();
      return;
    }

    if (!shared->newly_initialized() && !shared->empty()) {
      return;
    }

    struct flock import_lock {};
    import_lock.l_type = F_WRLCK;
    import_lock.l_whence = SEEK_SET;
    if (fcntl(shared->file_descriptor(), F_SETLKW, &import_lock) != 0) {
      return;
    }

    if (!shared->empty()) {
      import_lock.l_type = F_UNLCK;
      fcntl(shared->file_descriptor(), F_SETLK, &import_lock);
      return;
    }

    load_snapshot_into_shared();

    import_lock.l_type = F_UNLCK;
    fcntl(shared->file_descriptor(), F_SETLK, &import_lock);
  }

  void load_snapshot_into_shared() {
    std::ifstream in(snapshot_path, std::ios::binary);
    if (!in) {
      return;
    }

    FileHeader header{};
    if (!in.read(reinterpret_cast<char*>(&header), sizeof(header)) || !HeaderValid(header)) {
      return;
    }

    const int64_t now = NowEpoch();
    for (uint32_t i = 0; i < header.entry_count; ++i) {
      FileEntry entry{};
      if (!in.read(reinterpret_cast<char*>(&entry), sizeof(entry))) {
        break;
      }
      if (entry.expires_at <= now) {
        continue;
      }
      shared->store(entry.key, entry.allowed != 0, entry.expires_at);
    }
  }

  void load_snapshot_into_fallback() {
    std::ifstream in(snapshot_path, std::ios::binary);
    if (!in) {
      return;
    }

    FileHeader header{};
    if (!in.read(reinterpret_cast<char*>(&header), sizeof(header)) || !HeaderValid(header)) {
      return;
    }

    const int64_t now = NowEpoch();
    std::unique_lock lock(fallback_mutex);
    for (uint32_t i = 0; i < header.entry_count; ++i) {
      FileEntry entry{};
      if (!in.read(reinterpret_cast<char*>(&entry), sizeof(entry))) {
        break;
      }
      if (entry.expires_at <= now) {
        continue;
      }
      local_fallback[entry.key] = DecisionMemoryEntry{
          .allowed = entry.allowed != 0,
          .expires_at = entry.expires_at,
      };
    }
  }

  void write_snapshot() {
    if (!persistence_enabled_) {
      return;
    }

    const int64_t now = NowEpoch();
    std::vector<FileEntry> entries;

    if (use_shared) {
      shared->for_each_entry(now, [&](uint64_t key, bool allowed, int64_t expires_at) {
        FileEntry entry{};
        entry.key = key;
        entry.allowed = allowed ? 1 : 0;
        std::memset(entry.padding, 0, sizeof(entry.padding));
        entry.expires_at = expires_at;
        entries.push_back(entry);
      });
    } else {
      std::shared_lock lock(fallback_mutex);
      entries.reserve(local_fallback.size());
      for (const auto& [key, value] : local_fallback) {
        if (value.expires_at <= now) {
          continue;
        }
        FileEntry entry{};
        entry.key = key;
        entry.allowed = value.allowed ? 1 : 0;
        std::memset(entry.padding, 0, sizeof(entry.padding));
        entry.expires_at = value.expires_at;
        entries.push_back(entry);
      }
    }

    const std::filesystem::path path(snapshot_path);
    const std::filesystem::path tmp_path = path.string() + ".tmp";

    std::ofstream out(tmp_path, std::ios::binary | std::ios::trunc);
    if (!out) {
      return;
    }

    FileHeader header{};
    std::memcpy(header.magic, kMagic, sizeof(kMagic));
    header.version = kSnapshotVersion;
    header.entry_count = static_cast<uint32_t>(entries.size());
    header.reserved = 0;

    out.write(reinterpret_cast<const char*>(&header), sizeof(header));
    if (!entries.empty()) {
      out.write(reinterpret_cast<const char*>(entries.data()),
                static_cast<std::streamsize>(entries.size() * sizeof(FileEntry)));
    }

    out.close();
    if (!out) {
      std::error_code ec;
      std::filesystem::remove(tmp_path, ec);
      return;
    }

    std::error_code ec;
    std::filesystem::rename(tmp_path, path, ec);
    if (ec) {
      std::filesystem::remove(tmp_path, ec);
    }
  }

  void request_flush() {
    if (!persistence_enabled_) {
      return;
    }
    dirty_.store(true);
    flush_cv_.notify_one();
  }

  void stop_flush_thread() {
    if (!flush_thread_.joinable()) {
      return;
    }
    shutdown_.store(true);
    flush_cv_.notify_one();
    flush_thread_.join();
  }

  void flush_thread_main() {
    while (!shutdown_.load()) {
      std::unique_lock lock(flush_mutex_);
      flush_cv_.wait_for(lock, kFlushInterval, [this] {
        return shutdown_.load() || dirty_.load();
      });
      if (shutdown_.load()) {
        break;
      }
      if (!dirty_.exchange(false)) {
        LogCacheStats("flush");
        continue;
      }
      lock.unlock();
      write_snapshot();
      LogCacheStats("flush");
    }
  }

  void clear() {
    if (use_shared) {
      shared->clear();
      return;
    }
    std::unique_lock lock(fallback_mutex);
    local_fallback.clear();
  }

  std::optional<bool> lookup_key(uint64_t key) {
    if (!DecisionCacheEnabled()) {
      g_decision_misses.fetch_add(1, std::memory_order_relaxed);
      return std::nullopt;
    }

    const int64_t now = NowEpoch();
    if (use_shared) {
      if (const auto cached = shared->lookup(key, now)) {
        g_decision_hits.fetch_add(1, std::memory_order_relaxed);
        return cached;
      }
      g_decision_misses.fetch_add(1, std::memory_order_relaxed);
      return std::nullopt;
    }

    std::shared_lock lock(fallback_mutex);
    const auto found = local_fallback.find(key);
    if (found != local_fallback.end() && found->second.expires_at > now) {
      g_decision_hits.fetch_add(1, std::memory_order_relaxed);
      return found->second.allowed;
    }

    g_decision_misses.fetch_add(1, std::memory_order_relaxed);
    return std::nullopt;
  }

  void store_key(uint64_t key, bool allowed) {
    if (!DecisionCacheEnabled()) {
      return;
    }

    g_decision_stores.fetch_add(1, std::memory_order_relaxed);
    const int64_t expires_at = NowEpoch() + kEntryLifetimeSeconds;

    if (use_shared) {
      shared->store(key, allowed, expires_at);
    } else {
      std::unique_lock lock(fallback_mutex);
      local_fallback[key] = DecisionMemoryEntry{.allowed = allowed, .expires_at = expires_at};
    }

    request_flush();
  }
};

struct TraverseBanCache::Impl {
  std::unique_ptr<SharedTraverseCacheMap> shared;
  std::unordered_map<TraverseKey, bool, TraverseKeyHash> local_fallback;
  mutable std::shared_mutex fallback_mutex;
  bool use_shared{false};

  Impl() {
    shared = SharedTraverseCacheMap::Open(TraverseCachePathFromEnv());
    if (shared != nullptr) {
      use_shared = true;
      return;
    }
    std::cerr << "[truck_ban_cache] warning: failed to open shared traverse cache at "
              << TraverseCachePathFromEnv() << "; using per-worker in-memory fallback"
              << std::endl;
  }

  std::optional<bool> lookup(const TraverseKey& key) {
    if (use_shared) {
      return shared->lookup(key);
    }
    std::shared_lock lock(fallback_mutex);
    const auto found = local_fallback.find(key);
    if (found == local_fallback.end()) {
      return std::nullopt;
    }
    return found->second;
  }

  void store(const TraverseKey& key, bool allowed) {
    if (use_shared) {
      shared->store(key, allowed);
      return;
    }
    std::unique_lock lock(fallback_mutex);
    local_fallback[key] = allowed;
  }

  void clear() {
    if (use_shared) {
      shared->clear();
      return;
    }
    std::unique_lock lock(fallback_mutex);
    local_fallback.clear();
  }
};

CacheMode GetCacheMode() {
  const char* mode = std::getenv("TRUCK_BAN_CACHE_MODE");
  if (mode == nullptr || mode[0] == '\0') {
    return CacheMode::kFull;
  }
  if (std::strcmp(mode, "off") == 0 || std::strcmp(mode, "none") == 0 ||
      std::strcmp(mode, "0") == 0) {
    return CacheMode::kOff;
  }
  if (std::strcmp(mode, "decision") == 0 || std::strcmp(mode, "decision_only") == 0) {
    return CacheMode::kDecisionOnly;
  }
  return CacheMode::kFull;
}

bool DecisionCacheEnabled() {
  return GetCacheMode() != CacheMode::kOff;
}

bool TraverseCacheEnabled() {
  return GetCacheMode() == CacheMode::kFull;
}

void ResetCacheStats() {
  g_traverse_checks.store(0);
  g_traverse_hits.store(0);
  g_traverse_misses.store(0);
  g_traverse_stores.store(0);
  g_decision_hits.store(0);
  g_decision_misses.store(0);
  g_decision_stores.store(0);
  g_local_hour_hits.store(0);
  g_local_hour_misses.store(0);
}

CacheStatsSnapshot GetCacheStats() {
  return CacheStatsSnapshot{
      .traverse_checks = g_traverse_checks.load(),
      .traverse_hits = g_traverse_hits.load(),
      .traverse_misses = g_traverse_misses.load(),
      .traverse_stores = g_traverse_stores.load(),
      .decision_hits = g_decision_hits.load(),
      .decision_misses = g_decision_misses.load(),
      .decision_stores = g_decision_stores.load(),
      .local_hour_hits = g_local_hour_hits.load(),
      .local_hour_misses = g_local_hour_misses.load(),
  };
}

void LogCacheStats(const char* label) {
  if (!StatsLoggingEnabled()) {
    return;
  }

  const CacheStatsSnapshot stats = GetCacheStats();
  if (label != nullptr && std::strcmp(label, "flush") == 0) {
    if (!PeriodicStatsLoggingEnabled()) {
      return;
    }
    if (stats.traverse_checks == g_last_logged_traverse_checks.load()) {
      return;
    }
  }

  std::cerr << "[truck_ban_cache]"
            << (label != nullptr ? " " : "") << (label != nullptr ? label : "") << " mode="
            << CacheModeName(GetCacheMode());
  AppendRouteContext(std::cerr);
  std::cerr << " traverse_checks=" << stats.traverse_checks
            << " traverse_hits=" << stats.traverse_hits << " traverse_misses=" << stats.traverse_misses
            << " traverse_stores=" << stats.traverse_stores << " decision_hits=" << stats.decision_hits
            << " decision_misses=" << stats.decision_misses << " decision_stores=" << stats.decision_stores
            << " local_hour_hits=" << stats.local_hour_hits
            << " local_hour_misses=" << stats.local_hour_misses << std::endl;
  g_last_logged_traverse_checks.store(stats.traverse_checks);
}

RouteCacheLogScope::RouteCacheLogScope(double origin_lat,
                                       double origin_lon,
                                       double dest_lat,
                                       double dest_lon) {
  if (!StatsLoggingEnabled()) {
    return;
  }
  SetActiveRouteEndpoints(origin_lat, origin_lon, dest_lat, dest_lon);
  active_ = true;
}

RouteCacheLogScope::~RouteCacheLogScope() {
  if (!active_) {
    return;
  }
  LogCacheStats("route");
  {
    std::lock_guard lock(g_route_log_mutex);
    g_last_route = g_active_route;
    g_active_route.valid = false;
  }
}

void ClearAllCaches() {
  BanDecisionCache::instance().clear_memory();
  TraverseBanCache::instance().clear_memory();
}

void RecordTraverseCheck() {
  g_traverse_checks.fetch_add(1, std::memory_order_relaxed);
  MaybeLogTraverseInterval();
}

void RecordLocalHourHit() {
  g_local_hour_hits.fetch_add(1, std::memory_order_relaxed);
}

void RecordLocalHourMiss() {
  g_local_hour_misses.fetch_add(1, std::memory_order_relaxed);
}

BanDecisionCache& BanDecisionCache::instance() {
  static BanDecisionCache cache;
  return cache;
}

BanDecisionCache::BanDecisionCache() : impl_(std::make_unique<Impl>()) {}

BanDecisionCache::~BanDecisionCache() = default;

uint64_t BanDecisionCache::pack_local_hour_key(const LocalBanTime& local) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(local.ymd)) << 5) |
         static_cast<uint64_t>(static_cast<uint8_t>(local.hour));
}

uint64_t BanDecisionCache::make_key(const char* country_iso,
                                    uint32_t resolved_tz,
                                    uint64_t local_hour_key,
                                    baldr::RoadClass road_class) {
  const uint64_t country = static_cast<uint64_t>(static_cast<uint8_t>(country_iso[0])) |
                           (static_cast<uint64_t>(static_cast<uint8_t>(country_iso[1])) << 8);
  const uint64_t motorway = IsMotorwayOrTrunk(road_class) ? 1ULL : 0ULL;
  return (country << 48) | (static_cast<uint64_t>(resolved_tz) << 16) | (local_hour_key << 1) |
         motorway;
}

void BanDecisionCache::clear_memory() {
  impl_->clear();
}

std::optional<bool> BanDecisionCache::lookup_hour_key(const char* country_iso,
                                                      uint32_t resolved_tz,
                                                      uint64_t local_hour_key,
                                                      baldr::RoadClass road_class) {
  if (local_hour_key == 0) {
    return std::nullopt;
  }
  return impl_->lookup_key(make_key(country_iso, resolved_tz, local_hour_key, road_class));
}

void BanDecisionCache::store_hour_key(const char* country_iso,
                                      uint32_t resolved_tz,
                                      uint64_t local_hour_key,
                                      baldr::RoadClass road_class,
                                      bool allowed) {
  if (local_hour_key == 0) {
    return;
  }
  impl_->store_key(make_key(country_iso, resolved_tz, local_hour_key, road_class), allowed);
}

std::optional<bool> BanDecisionCache::lookup(const char* country_iso,
                                             uint32_t resolved_tz,
                                             const LocalBanTime& local,
                                             baldr::RoadClass road_class) {
  if (local.year == 0) {
    return std::nullopt;
  }
  return lookup_hour_key(country_iso, resolved_tz, pack_local_hour_key(local), road_class);
}

void BanDecisionCache::store(const char* country_iso,
                             uint32_t resolved_tz,
                             const LocalBanTime& local,
                             baldr::RoadClass road_class,
                             bool allowed) {
  if (local.year == 0) {
    return;
  }
  store_hour_key(country_iso, resolved_tz, pack_local_hour_key(local), road_class, allowed);
}

TraverseBanCache& TraverseBanCache::instance() {
  static TraverseBanCache cache;
  return cache;
}

TraverseBanCache::TraverseBanCache() : impl_(std::make_unique<Impl>()) {}

TraverseBanCache::~TraverseBanCache() = default;

void TraverseBanCache::clear_memory() {
  impl_->clear();
}

std::optional<bool> TraverseBanCache::lookup(const baldr::GraphId& from_node,
                                             const baldr::GraphId& to_node,
                                             uint64_t depart_time,
                                             uint64_t arrive_time,
                                             baldr::RoadClass road_class) {
  if (!from_node.is_valid() || !to_node.is_valid() || depart_time == 0 || arrive_time == 0) {
    return std::nullopt;
  }

  if (!TraverseCacheEnabled()) {
    g_traverse_misses.fetch_add(1, std::memory_order_relaxed);
    return std::nullopt;
  }

  const TraverseKey key = MakeTraverseKey(from_node, to_node, depart_time, arrive_time, road_class);
  if (const auto cached = impl_->lookup(key)) {
    g_traverse_hits.fetch_add(1, std::memory_order_relaxed);
    return cached;
  }

  g_traverse_misses.fetch_add(1, std::memory_order_relaxed);
  return std::nullopt;
}

void TraverseBanCache::store(const baldr::GraphId& from_node,
                             const baldr::GraphId& to_node,
                             uint64_t depart_time,
                             uint64_t arrive_time,
                             baldr::RoadClass road_class,
                             bool allowed) {
  if (!TraverseCacheEnabled()) {
    return;
  }

  if (!from_node.is_valid() || !to_node.is_valid() || depart_time == 0 || arrive_time == 0) {
    return;
  }

  g_traverse_stores.fetch_add(1, std::memory_order_relaxed);
  const TraverseKey key = MakeTraverseKey(from_node, to_node, depart_time, arrive_time, road_class);
  impl_->store(key, allowed);
}

} // namespace truck_ban
} // namespace sif
} // namespace valhalla
