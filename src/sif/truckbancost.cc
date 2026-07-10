#include "sif/truckbancost.h"

#include "sif/truckcost.h"

namespace valhalla {
namespace sif {

void ParseTruckBanCostOptions(const rapidjson::Document& doc,
                              const std::string& costing_options_key,
                              Costing* pbf_costing,
                              google::protobuf::RepeatedPtrField<CodedDescription>& warnings) {
  ParseTruckCostOptions(doc, costing_options_key, pbf_costing, warnings);
  pbf_costing->set_type(Costing::truck_ban);
}

cost_ptr_t CreateTruckBanCost(const Costing& costing) {
  Costing truck_ban_costing = costing;
  truck_ban_costing.set_type(Costing::truck_ban);
  return CreateTruckCost(truck_ban_costing);
}

} // namespace sif
} // namespace valhalla
