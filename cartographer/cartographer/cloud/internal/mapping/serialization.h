/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_CLOUD_INTERNAL_MAPPING_SERIALIZATION_H
#define CARTOGRAPHER_CLOUD_INTERNAL_MAPPING_SERIALIZATION_H

#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer_proto/cloud/map_builder_service.pb.h"

namespace cartographer {
namespace cloud {

cartographer_proto::cloud::TrajectoryState ToProto(
    const mapping::PoseGraphInterface::TrajectoryState& trajectory_state);
mapping::PoseGraphInterface::TrajectoryState FromProto(
    const cartographer_proto::cloud::TrajectoryState& proto);

cartographer_proto::cloud::TrajectoryRemapping ToProto(
    const std::map<int, int>& trajectory_remapping);
std::map<int, int> FromProto(
    const cartographer_proto::cloud::TrajectoryRemapping& proto);

}  // namespace cloud
}  // namespace cartographer

#endif  // CARTOGRAPHER_CLOUD_INTERNAL_MAPPING_SERIALIZATION_H
