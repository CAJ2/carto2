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

#include "cartographer/cloud/internal/handlers/get_trajectory_states_handler.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/cloud/internal/mapping/serialization.h"
#include "cartographer_proto/cloud/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void GetTrajectoryStatesHandler::OnRequest(
    const google::protobuf::Empty& request) {
  auto trajectories_state = GetContext<MapBuilderContextInterface>()
                                ->map_builder()
                                .pose_graph()
                                ->GetTrajectoryStates();
  auto response = absl::make_unique<
      cartographer_proto::cloud::GetTrajectoryStatesResponse>();
  for (const auto& entry : trajectories_state) {
    (*response->mutable_trajectories_state())[entry.first] =
        ToProto(entry.second);
  }
  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer