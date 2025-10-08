// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include "Eigen/Geometry"
#include "depth_image_proc/visibility.h"
#include "image_geometry/pinhole_camera_model.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <depth_image_proc/depth_traits.hpp>
#include <cv_bridge/cv_bridge.hpp>

std::string matInfo(const cv::Mat& m) {
    std::string typeStr;
    int type = m.type() & CV_MAT_TYPE_MASK;
    switch (type) {
        case CV_8UC1:  typeStr = "CV_8UC1"; break;
        case CV_8UC3:  typeStr = "CV_8UC3"; break;
        case CV_8UC4:  typeStr = "CV_8UC4"; break;
        case CV_32FC1: typeStr = "CV_32FC1"; break;
        case CV_32FC3: typeStr = "CV_32FC3"; break;
        case CV_64FC1: typeStr = "CV_64FC1"; break;
        case CV_64FC3: typeStr = "CV_64FC3"; break;
        default:       typeStr = "unknown"; break;
    }

    char buf[128];
    snprintf(buf, sizeof(buf), "Mat: %dx%d %s", m.rows, m.cols, typeStr.c_str());
    return std::string(buf);
}

namespace depth_image_proc
{

class RegisterNode : public rclcpp::Node
{
public:
  DEPTH_IMAGE_PROC_PUBLIC RegisterNode(const rclcpp::NodeOptions & options);

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  // Subscriptions
  image_transport::SubscriberFilter sub_depth_image_;
  message_filters::Subscriber<CameraInfo> sub_depth_info_, sub_rgb_info_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    Image, CameraInfo,
    CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Synchronizer> sync_;

  // Publications
  std::mutex connect_mutex_;
  image_transport::CameraPublisher pub_registered_;

  image_geometry::PinholeCameraModel depth_model_, rgb_model_;

  cv::Mat depth_transform_;
  cv::Mat depth_transform_2_;

  // Parameters
  bool fill_upsampling_holes_;
  bool use_rgb_timestamp_;  // use source time stamp from RGB camera
  bool radial_source_;
  bool rectified_source_;
  bool rectified_dest_;

  //
  CameraInfo last_depth_info_;
  CameraInfo last_rgb_info_;
  Eigen::Affine3f last_depth_to_rgb_;

  void imageCb(
    const Image::ConstSharedPtr & depth_image_msg,
    const CameraInfo::ConstSharedPtr & depth_info_msg,
    const CameraInfo::ConstSharedPtr & rgb_info_msg);

  template <typename T>
  void convert(
      const Image::ConstSharedPtr &depth_msg,
      const Image::SharedPtr &registered_msg,
      const Eigen::Affine3f &depth_to_rgb);

  void depthToPixels(cv::Mat &depth, cv::Mat &d_transform, cv::Mat &pixel_coords, cv::Mat &zs);

  void createTransform(
      const CameraInfo::ConstSharedPtr &depth_info_msg,
      const CameraInfo::ConstSharedPtr &rgb_info_msg,
      const Eigen::Affine3f &depth_to_rgb);
  void create_source_transform(
      const std::shared_ptr<const sensor_msgs::msg::CameraInfo> &depth_info_msg, 
      const Eigen::Affine3f &depth_to_rgb, 
      cv::Mat &d_transform,
      float offset = 0.0);
};

RegisterNode::RegisterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("RegisterNode", options)
{
  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("depth_image_transport", "raw");

  rclcpp::Clock::SharedPtr clock = this->get_clock();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Read parameters
  int queue_size = this->declare_parameter<int>("queue_size", 5);
  fill_upsampling_holes_ = this->declare_parameter<bool>("fill_upsampling_holes", false);
  use_rgb_timestamp_ = this->declare_parameter<bool>("use_rgb_timestamp", false);
  radial_source_ =  this->declare_parameter<bool>("radial_source", false);
  rectified_source_ =  this->declare_parameter<bool>("rectified_source", true);
  rectified_dest_ =  this->declare_parameter<bool>("rectified_dest", true);


  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  sync_ = std::make_shared<Synchronizer>(
    SyncPolicy(queue_size),
    sub_depth_image_,
    sub_depth_info_,
    sub_rgb_info_);
  sync_->registerCallback(
    std::bind(
      &RegisterNode::imageCb, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  // Create publisher with connect callback
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.matched_callback =
    [this](rclcpp::MatchedInfo &)
    {
      std::lock_guard<std::mutex> lock(connect_mutex_);
      if (pub_registered_.getNumSubscribers() == 0) {
        sub_depth_image_.unsubscribe();
        sub_depth_info_.unsubscribe();
        sub_rgb_info_.unsubscribe();
      } else if (!sub_depth_image_.getSubscriber()) {
        // Allow overriding QoS settings (history, depth, reliability)
        rclcpp::SubscriptionOptions sub_options;
        sub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

        // For compressed topics to remap appropriately, we need to pass a
        // fully expanded and remapped topic name to image_transport
        auto node_base = this->get_node_base_interface();
        std::string topic = node_base->resolve_topic_or_service_name("depth/image_rect", false);
        image_transport::TransportHints hints(this, "raw", "depth_image_transport");
        sub_depth_image_.subscribe(this, topic, hints.getTransport(), rmw_qos_profile_default,
          sub_options);
        auto qos = rmw_qos_profile_default;
        qos.depth = 10;
        sub_depth_info_.subscribe(this, "depth/camera_info", qos, sub_options);
        sub_rgb_info_.subscribe(this, "rgb/camera_info", qos, sub_options);
      }
    };
  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  std::string topic =
    node_base->resolve_topic_or_service_name("depth_registered/image_rect", false);

  // Allow overriding QoS settings (history, depth, reliability)
  pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pub_registered_ =
    image_transport::create_camera_publisher(
    this, topic,
    rmw_qos_profile_default, pub_options);
}

void RegisterNode::createTransform(
  const CameraInfo::ConstSharedPtr & depth_info_msg,
  const CameraInfo::ConstSharedPtr & rgb_info_msg,
  const Eigen::Affine3f & depth_to_rgb)
{
  last_depth_info_.header.stamp = depth_info_msg->header.stamp;
  last_rgb_info_.header.stamp = rgb_info_msg->header.stamp;
  if (*depth_info_msg == last_depth_info_) {
    if (*rgb_info_msg == last_rgb_info_) {
      if (last_depth_to_rgb_.isApprox(depth_to_rgb)) return;
    }
  }
  RCLCPP_INFO(get_logger(), "Creating new transform");

  last_depth_info_ = *depth_info_msg;
  last_rgb_info_ = *rgb_info_msg;
  last_depth_to_rgb_ = depth_to_rgb;
  depth_model_.fromCameraInfo(depth_info_msg);
  rgb_model_.fromCameraInfo(rgb_info_msg);
  if (fill_upsampling_holes_) {
    create_source_transform(depth_info_msg, depth_to_rgb, depth_transform_, -0.5);
    create_source_transform(depth_info_msg, depth_to_rgb, depth_transform_2_, 0.5);
  } else {
    create_source_transform(depth_info_msg, depth_to_rgb, depth_transform_);
  }
}

void RegisterNode::create_source_transform(
  const std::shared_ptr<const sensor_msgs::msg::CameraInfo> &depth_info_msg, 
  const Eigen::Affine3f &depth_to_rgb, 
  cv::Mat &d_transform,
  float offset
)
{
  auto w = depth_info_msg->width;
  auto h = depth_info_msg->height;
  auto points = cv::Mat(w * h, 2, CV_32FC1);
  int index = 0;
  for (unsigned int j = 0; j < h; j++)
  {
    for (unsigned int i = 0; i < w; i++)
    {
      points.at<float>(index, 0) = i + offset;
      points.at<float>(index, 1) = j + offset;
      index++;
    }
  }
  cv::Mat projection_2d;
  cv::Mat projection_3d;
  if (rectified_source_)
  {
    cv::Matx33d k = depth_model_.projectionMatrix().get_minor<3, 3>(0, 0);
    cv::undistortPoints(points, projection_2d, k, cv::Mat());
  }
  else
  {
    cv::undistortPoints(points, projection_2d, depth_model_.intrinsicMatrix(), depth_model_.distortionCoeffs());
  }
  cv::convertPointsToHomogeneous(projection_2d, projection_3d);
  if (radial_source_)
  { // convert rays to unit vectors if a radial source
    // Loop through each row (vector) of the matrix
    for (int i = 0; i < projection_3d.rows; ++i)
    {
      // Extract the current vector as a Mat row
      cv::Mat vec = projection_3d.row(i);

      // Calculate the L2 norm (magnitude) of the vector
      float norm = cv::norm(vec, cv::NORM_L2);

      // Handle the case of a zero vector to avoid division by zero
      if (norm > 0)
      {
        // Normalize the vector by dividing each element by its magnitude
        projection_3d.row(i) /= norm;
      }
    }
  }
  projection_3d = projection_3d.reshape(1, projection_3d.rows);
  cv::Mat rotation;
  eigen2cv(depth_to_rgb.rotation(), rotation);
  d_transform = projection_3d * rotation;
}

void RegisterNode::imageCb(
  const Image::ConstSharedPtr & depth_image_msg,
  const CameraInfo::ConstSharedPtr & depth_info_msg,
  const CameraInfo::ConstSharedPtr & rgb_info_msg)
{
  // Update camera models - these take binning & ROI into account
  depth_model_.fromCameraInfo(depth_info_msg);
  rgb_model_.fromCameraInfo(rgb_info_msg);

  // Query tf2 for transform from (X,Y,Z) in depth camera frame to RGB camera frame
  Eigen::Affine3f depth_to_rgb;
  try {
    tf2::TimePoint tf2_time(std::chrono::nanoseconds(depth_info_msg->header.stamp.nanosec) +
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::seconds(
          depth_info_msg->header.stamp.sec)));
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      rgb_info_msg->header.frame_id, depth_info_msg->header.frame_id,
      tf2_time);

    depth_to_rgb = tf2::transformToEigen(transform).cast<float>();
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "TF2 exception:\n%s", ex.what());
    return;
    /// @todo Can take on order of a minute to register a disconnect callback when we
    /// don't call publish() in this cb. What's going on roscpp?
  }
  createTransform(depth_info_msg, rgb_info_msg, depth_to_rgb);
  auto registered_msg = std::make_shared<Image>();
  registered_msg->header.stamp =
    use_rgb_timestamp_ ? rgb_info_msg->header.stamp : depth_image_msg->header.stamp;
  registered_msg->header.frame_id = rgb_info_msg->header.frame_id;
  registered_msg->encoding = depth_image_msg->encoding;

  cv::Size resolution = rgb_model_.reducedResolution();
  registered_msg->height = resolution.height;
  registered_msg->width = resolution.width;
  // step and data set in convert(), depend on depth data type

  if (depth_image_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convert<uint16_t>(depth_image_msg, registered_msg, depth_to_rgb);
  } else if (depth_image_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convert<float>(depth_image_msg, registered_msg, depth_to_rgb);
  } else {
    RCLCPP_ERROR(
      get_logger(), "Depth image has unsupported encoding [%s]",
      depth_image_msg->encoding.c_str());
    return;
  }

  // Registered camera info is the same as the RGB info, but uses the depth timestamp
  auto registered_info_msg = std::make_shared<CameraInfo>(*rgb_info_msg);
  registered_info_msg->header.stamp = registered_msg->header.stamp;

  pub_registered_.publish(registered_msg, registered_info_msg);
}

template<typename T>
void RegisterNode::convert(
  const Image::ConstSharedPtr & depth_msg,
  const Image::SharedPtr & registered_msg,
  const Eigen::Affine3f & depth_to_rgb)
{
  // Allocate memory for registered depth image
  registered_msg->step = registered_msg->width * sizeof(T);
  registered_msg->data.resize(registered_msg->height * registered_msg->step);
  T * registered_data = reinterpret_cast<T *>(&registered_msg->data[0]);
  // data is already zero-filled in the uint16 case,
  //   but for floats we want to initialize everything to NaN.
  DepthTraits<T>::initializeBuffer(registered_msg->data);
  cv_bridge::CvImagePtr cv_ptr;

  try {
        cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding); // or "passthrough"
  } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
        return;
  }
  cv::Mat depth = cv_ptr->image.reshape(1, depth_transform_.rows);

  if (fill_upsampling_holes_ == false) {
      cv::Mat pixel_coords;
      cv::Mat zs;
      depthToPixels(depth, depth_transform_, pixel_coords, zs);

      for(int i=0; i< pixel_coords.rows; i++) {
        T new_depth = DepthTraits<T>::fromMeters(zs.at<float>(i));
        int u_rgb = pixel_coords.at<float>(i,0);
        int v_rgb = pixel_coords.at<float>(i,1);
        if (u_rgb < 0 || u_rgb >= static_cast<int>(registered_msg->width) ||
          v_rgb < 0 || v_rgb >= static_cast<int>(registered_msg->height))
        {
          continue;
        }

        T & reg_depth = registered_data[v_rgb * registered_msg->width + u_rgb];
        // Validity and Z-buffer checks
        if (!DepthTraits<T>::valid(reg_depth) || reg_depth > new_depth) {
          reg_depth = new_depth;
        }

      }
      return;
  } else {
      cv::Mat pixel_coords_1, pixel_coords_2;
      cv::Mat zs_1, zs_2;
      depthToPixels(depth, depth_transform_, pixel_coords_1, zs_1);
      depthToPixels(depth, depth_transform_2_, pixel_coords_2, zs_2);

      for(int i=0; i< pixel_coords_1.rows; i++) {
        T new_depth = DepthTraits<T>::fromMeters(0.5 * (zs_1.at<float>(i) + zs_2.at<float>(i)));
        int u_rgb_1 = pixel_coords_1.at<float>(i,0);
        int v_rgb_1 = pixel_coords_1.at<float>(i,1);
        int u_rgb_2 = pixel_coords_2.at<float>(i,0);
        int v_rgb_2 = pixel_coords_2.at<float>(i,1);
        if (u_rgb_1 < 0 || u_rgb_2 >= static_cast<int>(registered_msg->width) ||
          v_rgb_1 < 0 || v_rgb_2 >= static_cast<int>(registered_msg->height))
        {
          continue;
        }
        for (int nv = v_rgb_1; nv <= v_rgb_2; ++nv) {
          for (int nu = u_rgb_1; nu <= u_rgb_2; ++nu) {
            T & reg_depth = registered_data[nv * registered_msg->width + nu];
            // Validity and Z-buffer checks
            if (!DepthTraits<T>::valid(reg_depth) || reg_depth > new_depth) {
              reg_depth = new_depth;
            }
          }
        }
      }
      return;
  }
  RCLCPP_INFO(get_logger(), "done");
  // Extract all the parameters we need
  double inv_depth_fx = 1.0 / depth_model_.fx();
  double inv_depth_fy = 1.0 / depth_model_.fy();
  double depth_cx = depth_model_.cx(), depth_cy = depth_model_.cy();
  double depth_Tx = depth_model_.Tx(), depth_Ty = depth_model_.Ty();
  double rgb_fx = rgb_model_.fx(), rgb_fy = rgb_model_.fy();
  double rgb_cx = rgb_model_.cx(), rgb_cy = rgb_model_.cy();
  double rgb_Tx = rgb_model_.Tx(), rgb_Ty = rgb_model_.Ty();

  // Transform the depth values into the RGB frame
  /// @todo When RGB is higher res, interpolate by rasterizing depth triangles onto the
  //   registered image
  const T * depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  int raw_index = 0;
  for (unsigned v = 0; v < depth_msg->height; ++v, depth_row += row_step) {
    for (unsigned u = 0; u < depth_msg->width; ++u, ++raw_index) {
      T raw_depth = depth_row[u];
      if (!DepthTraits<T>::valid(raw_depth)) {
        continue;
      }

      double depth = DepthTraits<T>::toMeters(raw_depth);

      if (fill_upsampling_holes_) {
        // Reproject (u,v,Z) to (X,Y,Z,1) in depth camera frame
        Eigen::Vector4f xyz_depth_1, xyz_depth_2;
        xyz_depth_1 << ((u - 0.5f - depth_cx) * depth - depth_Tx) * inv_depth_fx,
          ((v - 0.5f - depth_cy) * depth - depth_Ty) * inv_depth_fy,
          depth,
          1;
        xyz_depth_2 << ((u + 0.5f - depth_cx) * depth - depth_Tx) * inv_depth_fx,
          ((v + 0.5f - depth_cy) * depth - depth_Ty) * inv_depth_fy,
          depth,
          1;

        // Transform to RGB camera frame
        Eigen::Vector4f xyz_rgb_1 = depth_to_rgb * xyz_depth_1;
        Eigen::Vector4f xyz_rgb_2 = depth_to_rgb * xyz_depth_2;

        // Project to (u,v) in RGB image
        double inv_Z = 1.0 / xyz_rgb_1.z();
        int u_rgb_1 = (rgb_fx * xyz_rgb_1.x() + rgb_Tx) * inv_Z + rgb_cx + 0.5;
        int v_rgb_1 = (rgb_fy * xyz_rgb_1.y() + rgb_Ty) * inv_Z + rgb_cy + 0.5;
        inv_Z = 1.0 / xyz_rgb_2.z();
        int u_rgb_2 = (rgb_fx * xyz_rgb_2.x() + rgb_Tx) * inv_Z + rgb_cx + 0.5;
        int v_rgb_2 = (rgb_fy * xyz_rgb_2.y() + rgb_Ty) * inv_Z + rgb_cy + 0.5;

        if (u_rgb_1 < 0 || u_rgb_2 >= static_cast<int>(registered_msg->width) ||
          v_rgb_1 < 0 || v_rgb_2 >= static_cast<int>(registered_msg->height))
        {
          continue;
        }

        for (int nv = v_rgb_1; nv <= v_rgb_2; ++nv) {
          for (int nu = u_rgb_1; nu <= u_rgb_2; ++nu) {
            T & reg_depth = registered_data[nv * registered_msg->width + nu];
            T new_depth = DepthTraits<T>::fromMeters(0.5 * (xyz_rgb_1.z() + xyz_rgb_2.z()));
            // Validity and Z-buffer checks
            if (!DepthTraits<T>::valid(reg_depth) || reg_depth > new_depth) {
              reg_depth = new_depth;
            }
          }
        }
      }
    }
  }
}


void RegisterNode::depthToPixels(cv::Mat &depth, cv::Mat &d_transform, cv::Mat &pixel_coords, cv::Mat &zs)
{
  cv::Mat xyz_rgb(d_transform.size(), d_transform.type());
  auto t = last_depth_to_rgb_.translation();
  cv::Vec3f translation(t.x(), t.y(), t.z());
  for (int i = 0; i < 3; i++)
  {
    xyz_rgb.col(i) = d_transform.col(i).mul(depth) + translation[i];
  }
  cv::Vec3f empty(0, 0, 0);
  if (rectified_dest_)
  {
    cv::Matx33d k = rgb_model_.projectionMatrix().get_minor<3, 3>(0, 0);
    cv::projectPoints(xyz_rgb, empty, empty, k, cv::Mat(), pixel_coords);
  }
  else
  {
    cv::projectPoints(xyz_rgb, empty, empty, rgb_model_.intrinsicMatrix(), rgb_model_.distortionCoeffs(), pixel_coords);
  }
  zs = xyz_rgb.col(2);
}

}  // namespace depth_image_proc

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(depth_image_proc::RegisterNode)
