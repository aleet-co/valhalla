#include "proto/api.pb.h"
#include "worker.h"

#include <gtest/gtest.h>

using namespace valhalla;

TEST(TdaltRequest, invariant_route_rejected) {
  Api request;
  try {
    ParseApi(R"({"locations":[{"lon":0,"lat":0},{"lon":1,"lat":1}],
               "costing":"auto",
               "date_time":{"type":3,"value":"2026-07-13T08:00"}})",
             Options::route, request);
    FAIL() << "Expected invariant route to be rejected";
  } catch (const valhalla_exception_t& e) {
    EXPECT_EQ(e.code, 169);
  } catch (...) {
    FAIL() << "Expected valhalla_exception_t";
  }
}

TEST(TdaltRequest, invariant_isochrone_not_route_error) {
  Api request;
  try {
    ParseApi(R"({"locations":[{"lon":0,"lat":0}],
               "costing":"auto",
               "contours":[{"time":15}],
               "polygons":false,
               "date_time":{"type":3,"value":"2026-07-13T08:00"}})",
             Options::isochrone, request);
    FAIL() << "Expected isochrone invariant to be rejected";
  } catch (const valhalla_exception_t& e) {
    EXPECT_NE(e.code, 169);
    EXPECT_EQ(e.code, 142);
  } catch (...) {
    FAIL() << "Expected valhalla_exception_t";
  }
}

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
