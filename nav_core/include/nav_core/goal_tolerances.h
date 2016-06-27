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
    use_relative_tolerances_ = false;
    tolerance_left_ = goal_tolerance_xy_;
    tolerance_right_ = goal_tolerance_xy_;
    tolerance_front_ = goal_tolerance_xy_;
    tolerance_back_ = goal_tolerance_xy_;
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
   * @param target Pose to be compared to
   * @param test Pose to be tested against target pose
   * @return true if distance and yaw offsets are less than goal tolerances
   */
  bool equalUnderTolerances(const geometry_msgs::PoseStamped& target,
                            const geometry_msgs::PoseStamped& test) const
  {
    if (use_relative_tolerances_)
    {
      return equalUnderRelativeTolerances(target, test);
    }
    else
    {
      return equalUnderGlobalTolerances(target, test);
    }
  }

  /**
   * @brief Use goal tolerances to determine if poses are equal under global constraints
   * @param target Pose to be compared to
   * @param test Pose to be tested against target pose
   * @return true if distance and yaw offsets are less than goal tolerances
   */
  bool equalUnderGlobalTolerances(const geometry_msgs::PoseStamped& target,
                                  const geometry_msgs::PoseStamped& test) const
  {
    const float delta_yaw = tf::getYaw(target.pose.orientation) - tf::getYaw(test.pose.orientation);
    const float delta_x = target.pose.position.x - test.pose.position.x;
    const float delta_y = target.pose.position.y - test.pose.position.y;

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
    dyaw = fmod(fmod(dyaw, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
    if (dyaw > M_PI)
    {
      dyaw -= 2.0 * M_PI;
    }
    return (dx * dx + dy * dy <= goal_tolerance_xy_ * goal_tolerance_xy_) &
           (fabs(dyaw) <= goal_tolerance_yaw_);
  }

  /**
   * @brief Use goal tolerances to determine if poses are equal under relative constraints
   * @param target Pose to be compared to
   * @param test Pose to be tested against target pose
   * @return true if relative distance and yaw offsets are less than goal tolerances
   */
  bool equalUnderRelativeTolerances(const geometry_msgs::PoseStamped& target,
                                    const geometry_msgs::PoseStamped& test) const
  {
    // need to use target as the origin for test pose
    const float test_relative_x = test.pose.position.x - target.pose.position.x;
    const float test_relative_y = test.pose.position.y - target.pose.position.y;

    // and rotate tests by -target.yaw
    float sin_neg_yaw, cos_neg_yaw;

#if __APPLE__
    __sincosf(-tf::getYaw(target.pose.orientation), &sin_neg_yaw, &cos_neg_yaw);
#else
    sincosf(-tf::getYaw(target.pose.orientation), &sin_neg_yaw, &cos_neg_yaw);
#endif

    const float rotated_test_relative_x = cos_neg_yaw * test_relative_x - sin_neg_yaw * test_relative_y;
    const float rotated_test_relative_y = sin_neg_yaw * test_relative_x + cos_neg_yaw * test_relative_y;

    // now have enough info to make relative decisions. +x is forward, -x is backward
    // -y is right and +y is left

    if (rotated_test_relative_x > tolerance_front_)
    {
      return false;
    }

    if (rotated_test_relative_x < -tolerance_back_)
    {
      return false;
    }

    if (rotated_test_relative_y > tolerance_left_)
    {
      return false;
    }

    if (rotated_test_relative_y < -tolerance_right_)
    {
      return false;
    }

    // relative goal tolerances met, check yaw tolerance
    const float delta_yaw = tf::getYaw(target.pose.orientation) - tf::getYaw(test.pose.orientation);

    return lessThanTolerances(0, 0, delta_yaw);
  }

  /**
   * @brief Enable and set relative goal tolerance checks
   * @param tolerance_left_right left/right slop allowed about target
   * @param tolerance_front_back forward/back slop allowed about target
   */
  void setRelativeTolerances(float tolerance_left_right, float tolerance_front_back)
  {
    setRelativeTolerances(tolerance_left_right, tolerance_left_right,
                          tolerance_front_back, tolerance_front_back);
  }

  /**
   * @brief Enable and set relative goal tolerance checks
   * @param tolerance_left left slop allowed about target
   * @param tolerance_right right slop allowed about target
   * @param tolerance_front front slop allowed about target
   * @param tolerance_back back slop allowed about target
   */
  void setRelativeTolerances(float tolerance_left, float tolerance_right,
                             float tolerance_front, float tolerance_back)
  {
    tolerance_left_ = tolerance_left;
    tolerance_right_ = tolerance_right;
    tolerance_front_ = tolerance_front;
    tolerance_back_ = tolerance_back;
    use_relative_tolerances_ = true;
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


  /**
   * @brief Decides if relative tolerances should be used or gobal tolerances (xy)
   */
  bool use_relative_tolerances_;

  /**
   * @brief Distance in meters to the left of the goal pose above which it can be declared
   *        that the goal has not been achieved. Only considered if use_relative_tolerances_
   *        is true
   */
  float tolerance_left_;

  /**
   * @brief Distance in meters to the right of the goal pose above which it can be declared
   *        that the goal has not been achieved. Only considered if use_relative_tolerances_
   *        is true
   */
  float tolerance_right_;

  /**
   * @brief Distance in meters to the front of the goal pose above which it can be declared
   *        that the goal has not been achieved. Only considered if use_relative_tolerances_
   *        is true
   */
  float tolerance_front_;

  /**
   * @brief Distance in meters to the back of the goal pose above which it can be declared
   *        that the goal has not been achieved. Only considered if use_relative_tolerances_
   *        is true
   */
  float tolerance_back_;

protected:
  /**
   * @brief Set the default values
   */
  virtual void loadDefaultValues()
  {
    goal_tolerance_xy_ = 0.05;
    goal_tolerance_yaw_ = 0.05;

    use_relative_tolerances_ = false;
    tolerance_left_ = goal_tolerance_xy_;
    tolerance_right_ = goal_tolerance_xy_;
    tolerance_front_ = goal_tolerance_xy_;
    tolerance_back_ = goal_tolerance_xy_;
  }
};
}  // namespace nav_core

#endif  // CPR_PATHTRACKER_GOAL_TOLERANCES_H
