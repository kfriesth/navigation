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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef NAV_CORE_NAV_CORE_STATE_H_
#define NAV_CORE_NAV_CORE_STATE_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <boost/shared_ptr.hpp>

namespace nav_core
{
class BaseLocalPlanner;
class BaseGlobalPlanner;

/**
 * @class State
 * @brief Holds the objects related to planning and tracking for easier sharing
 *        with components
 */
class State
{
public:
  typedef boost::shared_ptr<State> Ptr;

  State()
    : global_costmap_(NULL), local_costmap_(NULL)
  {
  }

  /**
   * @brief Pointer to the current global costmap
   */
  costmap_2d::Costmap2DROS* global_costmap_;

  /**
   * @brief Pointer to the current local costmap
   */
  costmap_2d::Costmap2DROS* local_costmap_;

  /**
   * @brief Shared Pointer to the current global planner
   */
  boost::shared_ptr<BaseGlobalPlanner> global_planner_;

  /**
   * @brief Shared Pointer to the current local planner
   */
  boost::shared_ptr<BaseLocalPlanner> local_planner_;
};
}
#endif  // NAV_CORE_STATE_H_
