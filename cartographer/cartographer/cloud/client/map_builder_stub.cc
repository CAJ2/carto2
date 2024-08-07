/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/cloud/client/map_builder_stub.h"

#include "cartographer/cloud/internal/client/pose_graph_stub.h"
#include "cartographer/cloud/internal/client/trajectory_builder_stub.h"
#include "cartographer/cloud/internal/handlers/add_trajectory_handler.h"
#include "cartographer/cloud/internal/handlers/finish_trajectory_handler.h"
#include "cartographer/cloud/internal/handlers/get_submap_handler.h"
#include "cartographer/cloud/internal/handlers/load_state_from_file_handler.h"
#include "cartographer/cloud/internal/handlers/load_state_handler.h"
#include "cartographer/cloud/internal/handlers/write_state_handler.h"
#include "cartographer/cloud/internal/handlers/write_state_to_file_handler.h"
#include "cartographer/cloud/internal/mapping/serialization.h"
#include "cartographer/cloud/internal/sensor/serialization.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer_proto/cloud/map_builder_service.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace cloud {
namespace {

using absl::make_unique;
constexpr int kChannelTimeoutSeconds = 10;
constexpr int kRetryBaseDelayMilliseconds = 500;
constexpr float kRetryDelayFactor = 2.0;
constexpr int kMaxRetries = 5;

}  // namespace

MapBuilderStub::MapBuilderStub(const std::string& server_address,
                               const std::string& client_id)
    : client_channel_(::grpc::CreateChannel(
          server_address, ::grpc::InsecureChannelCredentials())),
      pose_graph_stub_(make_unique<PoseGraphStub>(client_channel_, client_id)),
      client_id_(client_id) {
  LOG(INFO) << "Connecting to SLAM process at " << server_address
            << " with client_id " << client_id;
  std::chrono::system_clock::time_point deadline(
      std::chrono::system_clock::now() +
      std::chrono::seconds(kChannelTimeoutSeconds));
  if (!client_channel_->WaitForConnected(deadline)) {
    LOG(FATAL) << "Failed to connect to " << server_address;
  }
}

int MapBuilderStub::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const cartographer_proto::cloud::mapping::TrajectoryBuilderOptions&
        trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  cartographer_proto::cloud::AddTrajectoryRequest request;
  request.set_client_id(client_id_);
  *request.mutable_trajectory_builder_options() = trajectory_options;
  for (const auto& sensor_id : expected_sensor_ids) {
    *request.add_expected_sensor_ids() = cloud::ToProto(sensor_id);
  }
  async_grpc::Client<handlers::AddTrajectorySignature> client(
      client_channel_, common::FromSeconds(kChannelTimeoutSeconds),
      async_grpc::CreateLimitedBackoffStrategy(
          common::FromMilliseconds(kRetryBaseDelayMilliseconds),
          kRetryDelayFactor, kMaxRetries));
  CHECK(client.Write(request));

  // Construct trajectory builder stub.
  trajectory_builder_stubs_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(client.response().trajectory_id()),
      std::forward_as_tuple(make_unique<TrajectoryBuilderStub>(
          client_channel_, client.response().trajectory_id(), client_id_,
          local_slam_result_callback)));
  return client.response().trajectory_id();
}

int MapBuilderStub::AddTrajectoryForDeserialization(
    const cartographer_proto::cloud::mapping::
        TrajectoryBuilderOptionsWithSensorIds& options_with_sensor_ids_proto) {
  LOG(FATAL) << "Not implemented";
}

mapping::TrajectoryBuilderInterface* MapBuilderStub::GetTrajectoryBuilder(
    int trajectory_id) const {
  auto it = trajectory_builder_stubs_.find(trajectory_id);
  if (it == trajectory_builder_stubs_.end()) {
    return nullptr;
  }
  return it->second.get();
}

void MapBuilderStub::FinishTrajectory(int trajectory_id) {
  cartographer_proto::cloud::FinishTrajectoryRequest request;
  request.set_client_id(client_id_);
  request.set_trajectory_id(trajectory_id);
  async_grpc::Client<handlers::FinishTrajectorySignature> client(
      client_channel_);
  ::grpc::Status status;
  client.Write(request, &status);
  if (!status.ok()) {
    LOG(ERROR) << "Failed to finish trajectory " << trajectory_id
               << " for client_id " << client_id_ << ": "
               << status.error_message();
    return;
  }
  trajectory_builder_stubs_.erase(trajectory_id);
}

std::string MapBuilderStub::SubmapToProto(
    const mapping::SubmapId& submap_id,
    cartographer_proto::cloud::mapping::SubmapQuery::Response*
        submap_query_response) {
  cartographer_proto::cloud::GetSubmapRequest request;
  submap_id.ToProto(request.mutable_submap_id());
  async_grpc::Client<handlers::GetSubmapSignature> client(client_channel_);
  CHECK(client.Write(request));
  submap_query_response->CopyFrom(client.response().submap_query_response());
  return client.response().error_msg();
}

void MapBuilderStub::SerializeState(bool include_unfinished_submaps,
                                    io::ProtoStreamWriterInterface* writer) {
  if (include_unfinished_submaps) {
    LOG(WARNING) << "Serializing unfinished submaps is currently unsupported. "
                    "Proceeding to write the state without them.";
  }
  google::protobuf::Empty request;
  async_grpc::Client<handlers::WriteStateSignature> client(client_channel_);
  CHECK(client.Write(request));
  cartographer_proto::cloud::WriteStateResponse response;
  while (client.StreamRead(&response)) {
    switch (response.state_chunk_case()) {
      case cartographer_proto::cloud::WriteStateResponse::kHeader:
        writer->WriteProto(response.header());
        break;
      case cartographer_proto::cloud::WriteStateResponse::kSerializedData:
        writer->WriteProto(response.serialized_data());
        break;
      default:
        LOG(FATAL) << "Unhandled message type";
    }
  }
}

bool MapBuilderStub::SerializeStateToFile(bool include_unfinished_submaps,
                                          const std::string& filename) {
  if (include_unfinished_submaps) {
    LOG(WARNING) << "Serializing unfinished submaps is currently unsupported. "
                    "Proceeding to write the state without them.";
  }
  cartographer_proto::cloud::WriteStateToFileRequest request;
  request.set_filename(filename);
  ::grpc::Status status;
  async_grpc::Client<handlers::WriteStateToFileSignature> client(
      client_channel_);
  if (!client.Write(request, &status)) {
    LOG(ERROR) << "WriteStateToFileRequest failed - "
               << "code: " << status.error_code()
               << " reason: " << status.error_message();
  }
  return client.response().success();
}

std::map<int, int> MapBuilderStub::LoadState(
    io::ProtoStreamReaderInterface* reader, const bool load_frozen_state) {
  async_grpc::Client<handlers::LoadStateSignature> client(client_channel_);
  {
    cartographer_proto::cloud::LoadStateRequest request;
    request.set_client_id(client_id_);
    CHECK(client.Write(request));
  }

  io::ProtoStreamDeserializer deserializer(reader);
  // Request with the SerializationHeader proto is sent first.
  {
    cartographer_proto::cloud::LoadStateRequest request;
    *request.mutable_serialization_header() = deserializer.header();
    request.set_load_frozen_state(load_frozen_state);
    CHECK(client.Write(request));
  }
  // Request with a PoseGraph proto is sent second.
  {
    cartographer_proto::cloud::LoadStateRequest request;
    *request.mutable_serialized_data()->mutable_pose_graph() =
        deserializer.pose_graph();
    request.set_load_frozen_state(load_frozen_state);
    CHECK(client.Write(request));
  }
  // Request with an AllTrajectoryBuilderOptions should be third.
  {
    cartographer_proto::cloud::LoadStateRequest request;
    *request.mutable_serialized_data()
         ->mutable_all_trajectory_builder_options() =
        deserializer.all_trajectory_builder_options();
    request.set_load_frozen_state(load_frozen_state);
    CHECK(client.Write(request));
  }
  // Multiple requests with SerializedData are sent after.
  cartographer_proto::cloud::LoadStateRequest request;
  while (
      deserializer.ReadNextSerializedData(request.mutable_serialized_data())) {
    request.set_load_frozen_state(load_frozen_state);
    CHECK(client.Write(request));
  }

  CHECK(reader->eof());
  CHECK(client.StreamWritesDone());
  CHECK(client.StreamFinish().ok());
  return FromProto(client.response().trajectory_remapping());
}

std::map<int, int> MapBuilderStub::LoadStateFromFile(
    const std::string& filename, const bool load_frozen_state) {
  cartographer_proto::cloud::LoadStateFromFileRequest request;
  request.set_file_path(filename);
  request.set_client_id(client_id_);
  request.set_load_frozen_state(load_frozen_state);
  async_grpc::Client<handlers::LoadStateFromFileSignature> client(
      client_channel_);
  CHECK(client.Write(request));
  return FromProto(client.response().trajectory_remapping());
}

int MapBuilderStub::num_trajectory_builders() const {
  return trajectory_builder_stubs_.size();
}

mapping::PoseGraphInterface* MapBuilderStub::pose_graph() {
  return pose_graph_stub_.get();
}

const std::vector<
    cartographer_proto::cloud::mapping::TrajectoryBuilderOptionsWithSensorIds>&
MapBuilderStub::GetAllTrajectoryBuilderOptions() const {
  LOG(FATAL) << "Not implemented";
}

}  // namespace cloud
}  // namespace cartographer
