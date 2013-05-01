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

#ifndef HEAD_TIME_CALIBRATION_LIB_H_
#define HEAD_TIME_CALIBRATION_LIB_H_

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include "boost/thread.hpp"

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>

#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/point_types.h>

#include "pointcloud_floor_filter/floor_plane_estimation.h"

#include <fstream>

namespace head_time_calibration_tools
{

class HeadTilting;


class HeadTimeCalibration
{
public:

  HeadTimeCalibration(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~HeadTimeCalibration();

private:
  void cameraInfoCallback( const sensor_msgs::CameraInfoConstPtr& msg );
  void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg );

  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  boost::shared_ptr<HeadTilting> head_tilting_;
  boost::shared_ptr<floor_filtered_pointcloud::FloorPlaneEstimation> floor_plane_estimation_;

  // ROS stuff
  boost::shared_ptr<image_transport::ImageTransport> depth_it_;
  boost::shared_ptr<image_transport::SubscriberFilter> depth_sub_;

  boost::shared_ptr< message_filters::Subscriber<sensor_msgs::CameraInfo> > camera_info_sub_;

  boost::mutex camera_info_mutex;
  sensor_msgs::CameraInfo::ConstPtr camera_info_msg;

  std::string odom_frame_;

  boost::shared_ptr<tf2::Buffer> tf_buffer_;
  boost::shared_ptr<tf2::TransformListener> tf_listener_;

  ros::Publisher pointcloud_pub_;

  std::ofstream out_file;
};

}

#endif
