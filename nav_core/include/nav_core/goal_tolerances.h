/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Clearpath Robotics, Inc.
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
*   * Neither the name of Clearpath Robotics, Inc. nor the names of its
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
*
* Author: Jason Mercer
*********************************************************************/

#ifndef NAV_CORE_GOAL_TOLERANCES_H
#define NAV_CORE_GOAL_TOLERANCES_H

#include <tf/tf.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <math.h>

namespace nav_core
{
/**
 * This class provides a way to define and test tolerances for planning and tracking.
 */
class GoalTolerances
{
public:
  typedef boost::shared_ptr<GoalTolerances> Ptr;

  /**
   * @brief Constructor where values are read from a node handle
   * @param nh Node Handle
   */
  GoalTolerances(ros::NodeHandle& nh)
  {
    loadDefaultValues();

    nh.param<float>("goal_tolerance_xy", goal_tolerance_xy_, goal_tolerance_xy_);
    nh.param<float>("goal_tolerance_yaw", goal_tolerance_yaw_, goal_tolerance_yaw_);
  }

  /**
   * @brief Constructor where values are set by arguments
   * @param goal_tolerance_xy The maximum distance in the XY plane between two poses where they
   *                          can still be considered the same
   * @param goal_tolerance_yaw The maximum angle between two poses where they
   *                           can still be considered the same
   */
  GoalTolerances(float goal_tolerance_xy, float goal_tolerance_yaw)
    : goal_tolerance_xy_(goal_tolerance_xy), goal_tolerance_yaw_(goal_tolerance_yaw)
  {
  }

  /**
   * @brief Constructor with default values
   */
  GoalTolerances()
  {
    loadDefaultValues();
  }

  /**
   * @brief Use goal tolerances to determine if poses are equal
   * @param a Pose A
   * @param b Pose B
   * @return true if distance and yaw offsets are less than goal tolerances
   */
  bool equalUnderTolerances(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) const
  {
    const float delta_yaw = tf::getYaw(a.pose.orientation) - tf::getYaw(b.pose.orientation);
    const float delta_x = a.pose.position.x - b.pose.position.x;
    const float delta_y = a.pose.position.y - b.pose.position.y;

    return lessThanTolerances(delta_x, delta_y, delta_yaw);
  }

  /**
   * @brief Determine if given offsets are less than goal tolerances
   * @param dx offset in the X direction
   * @param dy offset in the Y direction
   * @param dyaw offset in radians
   * @return true if distance and yaw offsets are less than goal tolerances
   */
  bool lessThanTolerances(float dx, float dy, float dyaw) const
  {
    return (dx * dx + dy * dy <= goal_tolerance_xy_ * goal_tolerance_xy_) &
           (fmod(fabs(dyaw), 2.0 * M_PI) <= goal_tolerance_yaw_);
  }
  /**
   * @brief Distance in R2 between goal and a pose above which it can be declared
   *        that the goal has not been achieved
   */
  float goal_tolerance_xy_;

  /**
   * @brief Distance in radians between goal and a pose above which it can be declared
   *        that the goal has not been achieved
   */
  float goal_tolerance_yaw_;

protected:
  /**
   * @brief Set the default values
   */
  virtual void loadDefaultValues()
  {
    goal_tolerance_xy_ = 0.05;
    goal_tolerance_yaw_ = 0.05;
  }
};
}  // namespace nav_core

#endif  // CPR_PATHTRACKER_GOAL_TOLERANCES_H
