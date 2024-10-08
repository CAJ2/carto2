#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

cartographer_proto::mapping::scan_matching::
    RealTimeCorrelativeScanMatcherOptions
    CreateRealTimeCorrelativeScanMatcherOptions(
        common::LuaParameterDictionary* const parameter_dictionary) {
  cartographer_proto::mapping::scan_matching::
      RealTimeCorrelativeScanMatcherOptions options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_translation_delta_cost_weight(
      parameter_dictionary->GetDouble("translation_delta_cost_weight"));
  options.set_rotation_delta_cost_weight(
      parameter_dictionary->GetDouble("rotation_delta_cost_weight"));
  CHECK_GE(options.translation_delta_cost_weight(), 0.);
  CHECK_GE(options.rotation_delta_cost_weight(), 0.);
  return options;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
