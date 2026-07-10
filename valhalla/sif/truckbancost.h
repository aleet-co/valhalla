#ifndef VALHALLA_SIF_TRUCKBANCOST_H_
#define VALHALLA_SIF_TRUCKBANCOST_H_

#include <valhalla/proto/options.pb.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla {
namespace sif {

void ParseTruckBanCostOptions(const rapidjson::Document& doc,
                              const std::string& costing_options_key,
                              Costing* pbf_costing,
                              google::protobuf::RepeatedPtrField<CodedDescription>& warnings);

cost_ptr_t CreateTruckBanCost(const Costing& costing);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_TRUCKBANCOST_H_
