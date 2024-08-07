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

#include "cartographer/cloud/internal/sensor/serialization.h"

namespace cartographer {
namespace cloud {

void CreateSensorMetadata(const std::string& sensor_id, const int trajectory_id,
                          const std::string& client_id,
                          cartographer_proto::cloud::SensorMetadata* proto) {
  proto->set_sensor_id(sensor_id);
  proto->set_trajectory_id(trajectory_id);
  proto->set_client_id(client_id);
}

void CreateAddFixedFramePoseDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const std::string& client_id,
    const cartographer_proto::cloud::sensor::FixedFramePoseData&
        fixed_frame_pose_data,
    cartographer_proto::cloud::AddFixedFramePoseDataRequest* proto) {
  CreateSensorMetadata(sensor_id, trajectory_id, client_id,
                       proto->mutable_sensor_metadata());
  *proto->mutable_fixed_frame_pose_data() = fixed_frame_pose_data;
}

void CreateAddImuDataRequest(
    const std::string& sensor_id, const int trajectory_id,
    const std::string& client_id,
    const cartographer_proto::cloud::sensor::ImuData& imu_data,
    cartographer_proto::cloud::AddImuDataRequest* proto) {
  CreateSensorMetadata(sensor_id, trajectory_id, client_id,
                       proto->mutable_sensor_metadata());
  *proto->mutable_imu_data() = imu_data;
}

void CreateAddOdometryDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const std::string& client_id,
    const cartographer_proto::cloud::sensor::OdometryData& odometry_data,
    cartographer_proto::cloud::AddOdometryDataRequest* proto) {
  CreateSensorMetadata(sensor_id, trajectory_id, client_id,
                       proto->mutable_sensor_metadata());
  *proto->mutable_odometry_data() = odometry_data;
}

void CreateAddRangeFinderDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const std::string& client_id,
    const cartographer_proto::cloud::sensor::TimedPointCloudData&
        timed_point_cloud_data,
    cartographer_proto::cloud::AddRangefinderDataRequest* proto) {
  CreateSensorMetadata(sensor_id, trajectory_id, client_id,
                       proto->mutable_sensor_metadata());
  *proto->mutable_timed_point_cloud_data() = timed_point_cloud_data;
}

void CreateAddLandmarkDataRequest(
    const std::string& sensor_id, int trajectory_id,
    const std::string& client_id,
    const cartographer_proto::cloud::sensor::LandmarkData& landmark_data,
    cartographer_proto::cloud::AddLandmarkDataRequest* proto) {
  CreateSensorMetadata(sensor_id, trajectory_id, client_id,
                       proto->mutable_sensor_metadata());
  *proto->mutable_landmark_data() = landmark_data;
}

void CreateSensorDataForLocalSlamResult(
    const std::string& sensor_id, int trajectory_id,
    const std::string& client_id, common::Time time, int starting_submap_index,
    const mapping::TrajectoryBuilderInterface::InsertionResult&
        insertion_result,
    cartographer_proto::cloud::SensorData* proto) {
  CreateSensorMetadata(sensor_id, trajectory_id, client_id,
                       proto->mutable_sensor_metadata());
  proto->mutable_local_slam_result_data()->set_timestamp(
      common::ToUniversal(time));
  *proto->mutable_local_slam_result_data()->mutable_node_data() =
      mapping::ToProto(*insertion_result.constant_data);
  for (const auto& insertion_submap : insertion_result.insertion_submaps) {
    // We only send the probability grid up if the submap is finished.
    auto* submap = proto->mutable_local_slam_result_data()->add_submaps();
    *submap = insertion_submap->ToProto(insertion_submap->insertion_finished());
    submap->mutable_submap_id()->set_trajectory_id(trajectory_id);
    submap->mutable_submap_id()->set_submap_index(starting_submap_index);
    ++starting_submap_index;
  }
}

cartographer_proto::cloud::SensorId ToProto(
    const mapping::TrajectoryBuilderInterface::SensorId& sensor_id) {
  using SensorType = mapping::TrajectoryBuilderInterface::SensorId::SensorType;
  cartographer_proto::cloud::SensorType type;
  switch (sensor_id.type) {
    case SensorType::RANGE:
      type = cartographer_proto::cloud::SensorType::RANGE;
      break;
    case SensorType::IMU:
      type = cartographer_proto::cloud::SensorType::IMU;
      break;
    case SensorType::ODOMETRY:
      type = cartographer_proto::cloud::SensorType::ODOMETRY;
      break;
    case SensorType::FIXED_FRAME_POSE:
      type = cartographer_proto::cloud::SensorType::FIXED_FRAME_POSE;
      break;
    case SensorType::LANDMARK:
      type = cartographer_proto::cloud::SensorType::LANDMARK;
      break;
    case SensorType::LOCAL_SLAM_RESULT:
      type = cartographer_proto::cloud::SensorType::LOCAL_SLAM_RESULT;
      break;
    default:
      LOG(FATAL) << "unknown SensorType";
  }
  cartographer_proto::cloud::SensorId proto;
  proto.set_type(type);
  proto.set_id(sensor_id.id);
  return proto;
}

mapping::TrajectoryBuilderInterface::SensorId FromProto(
    const cartographer_proto::cloud::SensorId& proto) {
  using SensorId = mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  SensorType type;
  switch (proto.type()) {
    case cartographer_proto::cloud::SensorType::RANGE:
      type = SensorType::RANGE;
      break;
    case cartographer_proto::cloud::SensorType::IMU:
      type = SensorType::IMU;
      break;
    case cartographer_proto::cloud::SensorType::ODOMETRY:
      type = SensorType::ODOMETRY;
      break;
    case cartographer_proto::cloud::SensorType::FIXED_FRAME_POSE:
      type = SensorType::FIXED_FRAME_POSE;
      break;
    case cartographer_proto::cloud::SensorType::LANDMARK:
      type = SensorType::LANDMARK;
      break;
    case cartographer_proto::cloud::SensorType::LOCAL_SLAM_RESULT:
      type = SensorType::LOCAL_SLAM_RESULT;
      break;
    default:
      LOG(FATAL) << "unknown cartographer_proto::cloud::SensorType";
  }
  return SensorId{type, proto.id()};
}

}  // namespace cloud
}  // namespace cartographer
