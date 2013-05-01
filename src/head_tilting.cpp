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

#include "head_time_calibration_tools/head_tilting.h"

#include <std_msgs/Float64.h>

#include <ros/ros.h>

#define CONTROLLER_RATE 20.0

namespace head_time_calibration_tools
{

HeadTilting::HeadTilting(double min_range, double max_range, double frequency) :
    min_range_(min_range),
    max_range_(max_range),
    frequency_(frequency),
    is_started_(true)
{
  // advertise ROS topics
  pub_head_pan_joint_command_ = nh_.advertise < std_msgs::Float64 > ("/head_pan_joint/command", 2);
  pub_head_tilt_joint_command_ = nh_.advertise < std_msgs::Float64 > ("/head_tilt_joint/command", 2);

  // start head control callback
  timer_ = nh_.createTimer(ros::Duration(1.0 / CONTROLLER_RATE),  boost::bind(&HeadTilting::controllerUpdateCB, this, _1));

  start();

}

HeadTilting::~HeadTilting()
{
  std_msgs::Float64 head_pan_link_msg;
  head_pan_link_msg.data = 0.0;
  pub_head_pan_joint_command_.publish(head_pan_link_msg);


  std_msgs::Float64 head_tilt_link_msg;
  head_tilt_link_msg.data = 0.0;
  pub_head_tilt_joint_command_.publish(head_tilt_link_msg);
}

void HeadTilting::start()
{
  timer_.start();
}

void HeadTilting::stop()
{
  timer_.stop();
}

// local controller callback
void HeadTilting::controllerUpdateCB(const ros::TimerEvent&)
{
  std_msgs::Float64 head_pan_link_msg;
  head_pan_link_msg.data = 0.0;
  pub_head_pan_joint_command_.publish(head_pan_link_msg);


  double time = ros::Time::now().toSec();

  double head_tilt_link_pos_ = min_range_ +
                               (((sin(time * 2.0 * M_PI * frequency_) / M_PI) + 1.0) / 2.0) * (max_range_ - min_range_);

  std_msgs::Float64 head_tilt_link_msg;
  head_tilt_link_msg.data = head_tilt_link_pos_;
  pub_head_tilt_joint_command_.publish(head_tilt_link_msg);
}

}
