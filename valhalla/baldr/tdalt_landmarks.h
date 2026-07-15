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
 * mmap index of ALT landmark distances on G_λ (precomputed by valhalla_build_tdalt_landmarks).
 *
 * Landmark notation:
 *   L ⊆ V          Set of landmark nodes (landmark_count_, e.g. 16).
 *   D_λ(x,y)       Shortest-path distance on G_λ from x to y (sum of λ along the path).
 *   dist_to[v][L]  = D_λ(v, L)   stored per node v (forward λ-distances toward landmark)
 *   dist_from[v][L]= D_λ(L, v)   reverse λ-distances from landmark
 *
 * ALT potentials (feasible lower bounds on remaining λ-distance):
 *   π_f(u) = max_{L∈L} ( D_λ(u,L) − D_λ(t,L),  D_λ(L,t) − D_λ(L,u) )
 *   π_b(u) = max_{L∈L} ( D_λ(u,L) − D_λ(s,L),  D_λ(L,s) − D_λ(L,u) )
 *
 * Used in A* keys: forward sortcost = g(u) + π_f(u); backward sortcost = g_λ(u) + π*_b(u).
 * Not related to Mjolnir POI landmarks.sqlite.
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
