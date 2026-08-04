file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/thor/*.h)

set(sources
  alternates.cc
  bidirectional_astar.cc
  cch/cch_graph.cc
  cch/cch_matrix.cc
  cch/contracted_search.cc
  cch/customizer.cc
  cch/nested_dissection.cc
  cch/order.cc
  cch/profile.cc
  region_grid/h3_partition.cc
  region_grid/medoid.cc
  region_grid/network_voronoi.cc
  region_grid/region_grid.cc
  costmatrix.cc
  dijkstras.cc
  matrix_action.cc
  multimodal_astar.cc
  multimodal_transit.cc
  route_action.cc
  timedistancebssmatrix.cc
  timedistancematrix.cc
  triplegbuilder.cc
  unidirectional_astar.cc
  worker.cc
  centroid.cc
  expansion_action.cc
  isochrone_action.cc
  isochrone.cc
  map_matcher.cc
  optimized_route_action.cc
  optimizer.cc
  route_matcher.cc
  status_action.cc
  trace_attributes_action.cc
  trace_route_action.cc
  triplegbuilder_utils.h)

set(system_includes
  ${date_include_dir}
  ${METIS_INCLUDE_DIR}
  $<$<BOOL:${WIN32}>:${dirent_include_dir}>)

valhalla_module(NAME thor
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
    PRIVATE
      ${CMAKE_BINARY_DIR}
      ${CMAKE_BINARY_DIR}/third_party/h3/src/h3lib/include
      ${VALHALLA_SOURCE_DIR}/third_party/h3/src/h3lib/include
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
    PRIVATE
      ${rapidjson_include_dir}
      ${unordered_dense_include_dir}

  DEPENDS
    valhalla::proto
    valhalla::sif
    valhalla::meili
    valhalla::odin
    ${valhalla_protobuf_targets}
    Boost::boost
    METIS::metis
    h3
    ${libprime_server_targets})
