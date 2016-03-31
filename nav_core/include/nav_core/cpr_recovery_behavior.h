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
* Author: Jason Mercer <jmercer@clearpathrobotics.com>
*********************************************************************/

#ifndef NAV_CORE_CPR_RECOVERY_BEHAVIOR_H_
#define NAV_CORE_CPR_RECOVERY_BEHAVIOR_H_

#include <nav_core/recovery_behavior.h>
#include <string>

namespace nav_core
{
/**
 * @brief CPRBaseRecovery is a base class for all CPR recovery behaviors
 *
 * This is an extension of the nav_core::RecoveryBehavior that allows richer interaction with the
 * recovery mechanism with the ability to setup the recovery with functionality different from
 * initialization.
 */
class CPRRecoveryBehavior : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief Destructor
   */
  virtual ~CPRRecoveryBehavior() {};

  /**
   * @brief To be run once the recovery is loaded
   */
  virtual void setUp() {}

  /**
   * @brief Retrieves this behavior's name
   * @return the name of this behavior
   */
  std::string getName() const
  {
    return name_;
  }

  /**
   * @brief Allow the behaviour to predict how successful it will be
   * @return success score
   */
  virtual float successPrediction()
  {
    return 0;
  }

  /**
   * @brief Attempt to run the behavor
   * @return true if behavior ran without problems
   */
  virtual bool attemptBehavior() = 0;

protected:
  /**
   * @brief Constructor
   */
  CPRRecoveryBehavior() {};

  /**
   * @brief The name of the recovery behavior in question. Available to inherited classes.
   */
  std::string name_;
};

}  // namespace nav_core

#endif  // NAV_CORE_CPR_RECOVERY_BEHAVIOR_H_
