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

#include "cartographer/cloud/internal/mapping/serialization.h"

#include "cartographer/common/port.h"

namespace cartographer {
namespace cloud {
namespace {

using TrajectoryState =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

}  // namespace

cartographer_proto::cloud::TrajectoryState ToProto(
    const TrajectoryState& trajectory_state) {
  switch (trajectory_state) {
    case TrajectoryState::ACTIVE:
      return cartographer_proto::cloud::TrajectoryState::ACTIVE;
    case TrajectoryState::FINISHED:
      return cartographer_proto::cloud::TrajectoryState::FINISHED;
    case TrajectoryState::FROZEN:
      return cartographer_proto::cloud::TrajectoryState::FROZEN;
    case TrajectoryState::DELETED:
      return cartographer_proto::cloud::TrajectoryState::DELETED;
    default:
      LOG(FATAL) << "Unknown TrajectoryState";
  }
}

TrajectoryState FromProto(
    const cartographer_proto::cloud::TrajectoryState& proto) {
  switch (proto) {
    case cartographer_proto::cloud::TrajectoryState::ACTIVE:
      return TrajectoryState::ACTIVE;
    case cartographer_proto::cloud::TrajectoryState::FINISHED:
      return TrajectoryState::FINISHED;
    case cartographer_proto::cloud::TrajectoryState::FROZEN:
      return TrajectoryState::FROZEN;
    case cartographer_proto::cloud::TrajectoryState::DELETED:
      return TrajectoryState::DELETED;
    default:
      LOG(FATAL) << "Unknown cartographer_proto::cloud::TrajectoryState";
  }
}

cartographer_proto::cloud::TrajectoryRemapping ToProto(
    const std::map<int, int>& trajectory_remapping) {
  cartographer_proto::cloud::TrajectoryRemapping proto;
  *proto.mutable_serialized_trajectories_to_trajectories() =
      google::protobuf::Map<int32, int32>(trajectory_remapping.begin(),
                                          trajectory_remapping.end());
  return proto;
}

std::map<int, int> FromProto(
    const cartographer_proto::cloud::TrajectoryRemapping& proto) {
  return std::map<int, int>(
      proto.serialized_trajectories_to_trajectories().begin(),
      proto.serialized_trajectories_to_trajectories().end());
}

}  // namespace cloud
}  // namespace cartographer
