/*********************************************************************
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.

 *  Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */

#include "head_time_calibration_tools/head_time_calibration_lib.h"
#include "head_time_calibration_tools/head_tilting.h"

#include <image_transport/camera_common.h>
#include <pcl/ros/conversions.h>

#include <fstream>

#include <message_filters/subscriber.h>

namespace head_time_calibration_tools
{

using namespace floor_filtered_pointcloud;

HeadTimeCalibration::HeadTimeCalibration(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh),
    head_tilting_(new HeadTilting(-1.0, 0.0, 1)),
    floor_plane_estimation_(new floor_filtered_pointcloud::FloorPlaneEstimation()),
    tf_buffer_(new tf2::Buffer()),
    tf_listener_(new tf2::TransformListener(*tf_buffer_))
{
   depth_it_.reset(new image_transport::ImageTransport(nh_));
   depth_sub_.reset(new image_transport::SubscriberFilter());

   camera_info_sub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>());

   // read depth map topic from param server
   std::string depthmap_topic;
   pnh_.param<std::string>("depth_image", depthmap_topic, "/head_camera/depth_registered/image");

      // read rgb transport from param server
   std::string depth_topic_transport;
   pnh_.param<std::string>("depth_topic_transport", depth_topic_transport, "compressedDepth");

   // read odom frame from param server
   pnh_.param<std::string>("base_footprint_frame", odom_frame_, "base_footprint");

   if (odom_frame_[0]=='/')
     odom_frame_ = odom_frame_.substr(1);

   // subscribe to image topics
   depth_sub_->subscribe(*depth_it_,
                         depthmap_topic,
                         1,
                         image_transport::TransportHints(depth_topic_transport));

   depth_sub_->registerCallback(boost::bind(&HeadTimeCalibration::depthImageCallback, this, _1));

   // subscribe to CameraInfo  topic
   camera_info_sub_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>());

   std::string info_topic = image_transport::getCameraInfoTopic(depthmap_topic);
   camera_info_sub_->subscribe(nh_, info_topic, 1);
   camera_info_sub_->registerCallback(boost::bind(&HeadTimeCalibration::cameraInfoCallback, this, _1));

   pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("floor_points", 0);

   out_file.open("data.log");

}

HeadTimeCalibration::~HeadTimeCalibration()
{
  out_file.close();
}

void HeadTimeCalibration::cameraInfoCallback( const sensor_msgs::CameraInfoConstPtr& msg )
{
  boost::mutex::scoped_lock l(camera_info_mutex);
  camera_info_msg = msg;
}

void HeadTimeCalibration::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg )
{
  geometry_msgs::TransformStampedPtr depth_to_odom_transform(new geometry_msgs::TransformStamped);
  geometry_msgs::TransformStampedPtr camera_to_odom_transform(new geometry_msgs::TransformStamped);
  try
  {

    std::string source_frame = depth_msg->header.frame_id;
    if (source_frame[0] == '/')
      source_frame = source_frame.substr(1);

    try
    {
      // lookup transformation for tf_pair
      *depth_to_odom_transform = tf_buffer_->lookupTransform(odom_frame_, source_frame, depth_msg->header.stamp,
                                                             ros::Duration(3.0));

      double ransac_floor_distance_ = 0.15;
      double filtered_floor_distance_ = 0.10;

      geometry_msgs::PointPtr floor_normal(new geometry_msgs::Point);

      floor_plane_estimation_->floorPlaneEstimation(depth_msg,
                                                    camera_info_msg,
                                                    depth_to_odom_transform,
                                                    ransac_floor_distance_,
                                                    floor_normal
                                                    );


      // set Z axis to Z axis in odom frame
      Eigen::Quaternion<float> eig_quat_in;
      eig_quat_in.x() = depth_to_odom_transform->transform.rotation.x;
      eig_quat_in.y() = depth_to_odom_transform->transform.rotation.y;
      eig_quat_in.z() = depth_to_odom_transform->transform.rotation.z;
      eig_quat_in.w() = depth_to_odom_transform->transform.rotation.w;
      Eigen::Matrix3f eig_in = eig_quat_in.normalized().toRotationMatrix();

      Eigen::Vector3f camera_in = eig_in.col(2);

      float floor_angle = atan2(floor_normal->y, floor_normal->z)*-1;
      float camera_angle = atan2(camera_in(0), camera_in(2));


      ROS_INFO("Time:%.4f, camera tilt: %.4f, floor angle: %.4f\n", depth_msg->header.stamp.toSec(), camera_angle, floor_angle);


      out_file<<depth_msg->header.stamp.toSec()<<"; "<<camera_angle<<"; "<<floor_angle;


    }
    catch (const tf2::TransformException &ex)
    {
      ROS_ERROR("Transform failure: %s", ex.what());
      return;
    }




  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

}
