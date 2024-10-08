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

#ifndef CARTOGRAPHER_SENSOR_FIXED_FRAME_POSE_DATA_H_
#define CARTOGRAPHER_SENSOR_FIXED_FRAME_POSE_DATA_H_

#include <memory>

#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_proto/sensor/sensor.pb.h"

namespace cartographer {
namespace sensor {

// The fixed frame pose data (like GPS, pose, etc.) will be used in the
// optimization.
struct FixedFramePoseData {
  common::Time time;
  absl::optional<transform::Rigid3d> pose;
};

// Converts 'pose_data' to a cartographer_proto::sensor::FixedFramePoseData.
cartographer_proto::sensor::FixedFramePoseData ToProto(
    const FixedFramePoseData& pose_data);

// Converts 'proto' to an FixedFramePoseData.
FixedFramePoseData FromProto(
    const cartographer_proto::sensor::FixedFramePoseData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_FIXED_FRAME_POSE_DATA_H_
