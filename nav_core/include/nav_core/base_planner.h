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
#ifndef NAV_CORE_BASE_PLANNER
#define NAV_CORE_BASE_PLANNER

#include <nav_core/goal_tolerances.h>
#include <nav_core/goal_tolerances_aware.h>
#include <nav_core/footprint_set.h>

namespace nav_core
{
/**
 * @class BasePlanner
 * @brief Provides an interface for planners used in navigation.
 */
class BasePlanner : public GoalTolerancesAware
{
public:
  /**
   * @brief Virtual destructor as this will have derived classes
   */
  virtual ~BasePlanner()
  {
  }

  /**
   * @brief Provides pointer so planner knows where to look for required footprints.
   * @param[in] footprint_set_ptr Pointer to the pointer that tracks the current FootprintSet object.
   */
  void setFootprintSet(FootprintSet::Ptr* footprint_set_ptr)
  {
    footprint_set_ptr_ = footprint_set_ptr;
  }

protected:
  /**
   * @brief Convenience get function so we don't have to double dereference the FootprintSet pointer.
   * @return Pointer to the FootprintSet object.
   */
  FootprintSet::Ptr footprintMgr()
  {
    return *footprint_set_ptr_;
  }

protected:
  FootprintSet::Ptr* footprint_set_ptr_;
};

}  // namespace nav_core

#endif  // NAV_CORE_BASE_PLANNER
