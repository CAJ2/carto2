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

#ifndef CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H_

#include "cartographer/common/time.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_proto/transform/timestamped_transform.pb.h"

namespace cartographer {
namespace transform {

struct TimestampedTransform {
  common::Time time;
  transform::Rigid3d transform;
};

TimestampedTransform FromProto(
    const cartographer_proto::transform::TimestampedTransform& proto);
cartographer_proto::transform::TimestampedTransform ToProto(
    const TimestampedTransform& transform);

TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const common::Time time);

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
