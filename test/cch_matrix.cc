#include "thor/cch/cch_matrix.h"
#include "test.h"

#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>

using namespace valhalla;

namespace {

TEST(CCHMatrixConfig, DefaultsToContractedPareto) {
  boost::property_tree::ptree pt;
  pt.put("cch.enabled", true);
  pt.put("cch.artifact", "/tmp/missing.bin");
  // omit query_mode
  thor::CCHMatrix m(pt);
  EXPECT_EQ(m.query_mode(), thor::cch::QueryMode::ContractedPareto);
}

TEST(CCHMatrixConfig, ParsesCorridorExplicitly) {
  boost::property_tree::ptree pt;
  pt.put("cch.enabled", true);
  pt.put("cch.artifact", "/tmp/missing.bin");
  pt.put("cch.query_mode", "corridor");
  thor::CCHMatrix m(pt);
  EXPECT_EQ(m.query_mode(), thor::cch::QueryMode::Corridor);
}

TEST(CCHMatrixConfig, ParsesContracted) {
  boost::property_tree::ptree pt;
  pt.put("cch.enabled", true);
  pt.put("cch.artifact", "/tmp/missing.bin");
  pt.put("cch.query_mode", "contracted");
  thor::CCHMatrix m(pt);
  EXPECT_EQ(m.query_mode(), thor::cch::QueryMode::Contracted);
}

TEST(CCHMatrixConfig, ParsesContractedPareto) {
  boost::property_tree::ptree pt;
  pt.put("cch.enabled", true);
  pt.put("cch.artifact", "/tmp/missing.bin");
  pt.put("cch.query_mode", "contracted_pareto");
  thor::CCHMatrix m(pt);
  EXPECT_EQ(m.query_mode(), thor::cch::QueryMode::ContractedPareto);
}

TEST(CCHMatrixConfig, UnknownFallsBackToCorridor) {
  boost::property_tree::ptree pt;
  pt.put("cch.enabled", true);
  pt.put("cch.artifact", "/tmp/missing.bin");
  pt.put("cch.query_mode", "not-a-mode");
  thor::CCHMatrix m(pt);
  EXPECT_EQ(m.query_mode(), thor::cch::QueryMode::Corridor);
}

} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
