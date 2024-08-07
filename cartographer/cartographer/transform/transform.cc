/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/transform/transform.h"

namespace cartographer {
namespace transform {

Rigid2d ToRigid2(const cartographer_proto::transform::Rigid2d& transform) {
  return Rigid2d({transform.translation().x(), transform.translation().y()},
                 transform.rotation());
}

Eigen::Vector2d ToEigen(const cartographer_proto::transform::Vector2d& vector) {
  return Eigen::Vector2d(vector.x(), vector.y());
}

Eigen::Vector3f ToEigen(const cartographer_proto::transform::Vector3f& vector) {
  return Eigen::Vector3f(vector.x(), vector.y(), vector.z());
}

Eigen::Vector4f ToEigen(const cartographer_proto::transform::Vector4f& vector) {
  return Eigen::Vector4f(vector.x(), vector.y(), vector.z(), vector.t());
}

Eigen::Vector3d ToEigen(const cartographer_proto::transform::Vector3d& vector) {
  return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

Eigen::Quaterniond ToEigen(
    const cartographer_proto::transform::Quaterniond& quaternion) {
  return Eigen::Quaterniond(quaternion.w(), quaternion.x(), quaternion.y(),
                            quaternion.z());
}

cartographer_proto::transform::Rigid2d ToProto(
    const transform::Rigid2d& transform) {
  cartographer_proto::transform::Rigid2d proto;
  proto.mutable_translation()->set_x(transform.translation().x());
  proto.mutable_translation()->set_y(transform.translation().y());
  proto.set_rotation(transform.rotation().angle());
  return proto;
}

cartographer_proto::transform::Rigid2f ToProto(
    const transform::Rigid2f& transform) {
  cartographer_proto::transform::Rigid2f proto;
  proto.mutable_translation()->set_x(transform.translation().x());
  proto.mutable_translation()->set_y(transform.translation().y());
  proto.set_rotation(transform.rotation().angle());
  return proto;
}

cartographer_proto::transform::Rigid3d ToProto(
    const transform::Rigid3d& rigid) {
  cartographer_proto::transform::Rigid3d proto;
  *proto.mutable_translation() = ToProto(rigid.translation());
  *proto.mutable_rotation() = ToProto(rigid.rotation());
  return proto;
}

transform::Rigid3d ToRigid3(
    const cartographer_proto::transform::Rigid3d& rigid) {
  return transform::Rigid3d(ToEigen(rigid.translation()),
                            ToEigen(rigid.rotation()));
}

cartographer_proto::transform::Rigid3f ToProto(
    const transform::Rigid3f& rigid) {
  cartographer_proto::transform::Rigid3f proto;
  *proto.mutable_translation() = ToProto(rigid.translation());
  *proto.mutable_rotation() = ToProto(rigid.rotation());
  return proto;
}

cartographer_proto::transform::Vector2d ToProto(const Eigen::Vector2d& vector) {
  cartographer_proto::transform::Vector2d proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  return proto;
}

cartographer_proto::transform::Vector3f ToProto(const Eigen::Vector3f& vector) {
  cartographer_proto::transform::Vector3f proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  return proto;
}

cartographer_proto::transform::Vector4f ToProto(const Eigen::Vector4f& vector) {
  cartographer_proto::transform::Vector4f proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  proto.set_t(vector.w());
  return proto;
}

cartographer_proto::transform::Vector3d ToProto(const Eigen::Vector3d& vector) {
  cartographer_proto::transform::Vector3d proto;
  proto.set_x(vector.x());
  proto.set_y(vector.y());
  proto.set_z(vector.z());
  return proto;
}

cartographer_proto::transform::Quaternionf ToProto(
    const Eigen::Quaternionf& quaternion) {
  cartographer_proto::transform::Quaternionf proto;
  proto.set_w(quaternion.w());
  proto.set_x(quaternion.x());
  proto.set_y(quaternion.y());
  proto.set_z(quaternion.z());
  return proto;
}

cartographer_proto::transform::Quaterniond ToProto(
    const Eigen::Quaterniond& quaternion) {
  cartographer_proto::transform::Quaterniond proto;
  proto.set_w(quaternion.w());
  proto.set_x(quaternion.x());
  proto.set_y(quaternion.y());
  proto.set_z(quaternion.z());
  return proto;
}

}  // namespace transform
}  // namespace cartographer
