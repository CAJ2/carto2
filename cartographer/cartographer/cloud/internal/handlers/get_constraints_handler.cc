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

#include "cartographer/cloud/internal/handlers/get_constraints_handler.h"

#include "absl/memory/memory.h"
#include "async_grpc/rpc_handler.h"
#include "cartographer/cloud/internal/map_builder_context_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer_proto/cloud/map_builder_service.pb.h"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void GetConstraintsHandler::OnRequest(const google::protobuf::Empty& request) {
  auto constraints = GetContext<MapBuilderContextInterface>()
                         ->map_builder()
                         .pose_graph()
                         ->constraints();
  auto response =
      absl::make_unique<cartographer_proto::cloud::GetConstraintsResponse>();
  response->mutable_constraints()->Reserve(constraints.size());
  for (const auto& constraint : constraints) {
    *response->add_constraints() = mapping::ToProto(constraint);
  }
  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
