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

#include "cartographer/cloud/internal/handlers/add_fixed_frame_pose_data_handler.h"

#include "cartographer/cloud/internal/testing/handler_test.h"
#include "cartographer/cloud/internal/testing/test_helpers.h"
#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace cloud {
namespace handlers {
namespace {

using ::testing::_;
using ::testing::Eq;
using ::testing::Pointee;
using ::testing::Truly;

const std::string kMessage = R"(
  sensor_metadata {
    trajectory_id: 1
    sensor_id: "sensor_id"
  }
  fixed_frame_pose_data {
    timestamp: 2
    pose {
      translation {
        x: 3 y: 4 z: 5
      }
      rotation {
        w: 6 x: 7 y: 8 z: 9
      }
    }
  })";

using AddFixedFramePoseDataHandlerTest =
    testing::HandlerTest<AddFixedFramePoseDataSignature,
                         AddFixedFramePoseDataHandler>;

TEST_F(AddFixedFramePoseDataHandlerTest, NoLocalSlamUploader) {
  cartographer_proto::cloud::AddFixedFramePoseDataRequest request;
  EXPECT_TRUE(
      google::protobuf::TextFormat::ParseFromString(kMessage, &request));
  SetNoLocalTrajectoryUploader();
  EXPECT_CALL(
      *mock_map_builder_context_,
      CheckClientIdForTrajectory(Eq(request.sensor_metadata().client_id()),
                                 Eq(request.sensor_metadata().trajectory_id())))
      .WillOnce(::testing::Return(true));
  EXPECT_CALL(*mock_map_builder_context_,
              DoEnqueueSensorData(
                  Eq(request.sensor_metadata().trajectory_id()),
                  Pointee(Truly(testing::BuildDataPredicateEquals(request)))));
  test_server_->SendWrite(request);
  test_server_->SendWritesDone();
  test_server_->SendFinish();
}

TEST_F(AddFixedFramePoseDataHandlerTest, WithMockLocalSlamUploader) {
  cartographer_proto::cloud::AddFixedFramePoseDataRequest request;
  EXPECT_TRUE(
      google::protobuf::TextFormat::ParseFromString(kMessage, &request));
  SetMockLocalTrajectoryUploader();
  EXPECT_CALL(
      *mock_map_builder_context_,
      CheckClientIdForTrajectory(Eq(request.sensor_metadata().client_id()),
                                 Eq(request.sensor_metadata().trajectory_id())))
      .WillOnce(::testing::Return(true));
  EXPECT_CALL(*mock_map_builder_context_,
              DoEnqueueSensorData(
                  Eq(request.sensor_metadata().trajectory_id()),
                  Pointee(Truly(testing::BuildDataPredicateEquals(request)))));
  cartographer_proto::cloud::SensorData sensor_data;
  *sensor_data.mutable_sensor_metadata() = request.sensor_metadata();
  *sensor_data.mutable_fixed_frame_pose_data() =
      request.fixed_frame_pose_data();
  EXPECT_CALL(*mock_local_trajectory_uploader_,
              DoEnqueueSensorData(Pointee(
                  Truly(testing::BuildProtoPredicateEquals(&sensor_data)))));
  test_server_->SendWrite(request);
  test_server_->SendWritesDone();
  test_server_->SendFinish();
}

}  // namespace
}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
