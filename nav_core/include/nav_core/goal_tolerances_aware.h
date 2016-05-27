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
#ifndef NAV_CORE_GOAL_TOLERANCES_AWARE
#define NAV_CORE_GOAL_TOLERANCES_AWARE

#include <nav_core/goal_tolerances.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <list>

namespace nav_core
{
/**
 * @class GoalTolerancesAware
 * @brief An interface that provides a goal tolerances object, a setter and a getter
 */
class GoalTolerancesAware
{
public:
  typedef boost::function<void (nav_core::GoalTolerances::Ptr goal_tolerances)> GoalTolerancesSetCallback_t;
  typedef boost::tuple<boost::weak_ptr<GoalTolerancesAware>,  GoalTolerancesSetCallback_t> ptr_func_tuple_t;

  /**
   * @brief Virtual destructor as this will have derived classes
   */
  virtual ~GoalTolerancesAware()
  {
  }

  /**
   * @brief Set the object that will be used to make decisions about goal tolerances
   * @param goal_tolerances The smart pointer to the object
   */
  virtual void setGoalTolerances(nav_core::GoalTolerances::Ptr goal_tolerances)
  {
    boost::lock_guard<boost::mutex> lock(goaltolerances_propagate_mutex_);
    goal_tolerances_ = goal_tolerances;

    // iterate through raw functions
    {
      std::list<GoalTolerancesSetCallback_t>::iterator it;

      for (it = goalTolerancesSetCallbacks_.begin(); it != goalTolerancesSetCallbacks_.end(); ++it)
      {
        (*it)(goal_tolerances_);
      }
    }

    // iterate through tuples, remove weak refs as needed
    {
      std::list<ptr_func_tuple_t>::iterator it;

      for (it = ptr_goalTolerancesSetCallbacks_.begin(); it != ptr_goalTolerancesSetCallbacks_.end(); /*no inc*/)
      {
        boost::weak_ptr<GoalTolerancesAware> q;
        GoalTolerancesSetCallback_t f;

        boost::tie(q, f) = *it;

        // get a lock on the weak reference
        boost::shared_ptr<GoalTolerancesAware> p = q.lock();

        if (p)
        {
          f(goal_tolerances_);
          ++it;
        }
        else
        {
          // managed resource has expired, remove weak reference and function from list
          it = ptr_goalTolerancesSetCallbacks_.erase(it);
        }
      }
    }
  }

  /**
   * @brief Get the object that is used to make decisions about goal tolerances
   * @return The smart pointer to the object
   */
  virtual nav_core::GoalTolerances::Ptr goalTolerances() const
  {
    return goal_tolerances_;
  }

  /**
   * @brief Add a function that will be called when setGoalTolerances is called. There is
   *        no way to remove a single function as boost functions are not comparable.
   * @param func Callback function
   */
  void addGoalTolerancesSetCallback(GoalTolerancesSetCallback_t func)
  {
    boost::lock_guard<boost::mutex> lock(goaltolerances_propagate_mutex_);
    goalTolerancesSetCallbacks_.push_back(func);
  }

  /**
   * @brief Remove all functions from the callback list
   */
  void clearGoalTolerancesSetCallbacks()
  {
    boost::lock_guard<boost::mutex> lock(goaltolerances_propagate_mutex_);
    goalTolerancesSetCallbacks_.clear();
    ptr_goalTolerancesSetCallbacks_.clear();
  }

  /**
   * @brief Convenience method to use the callback mechanism to propagate new goal tolerances
   *        to other GoalTolerancesAware objects. This version allows shared pointers where the managed
   *        resource may get destroyed at any time.
   * @param object Object that will get new goal tolerances set
   */
  template <typename T>
  void propagateGoalTolerancesTo(boost::shared_ptr<T> object)
  {
    boost::lock_guard<boost::mutex> lock(goaltolerances_propagate_mutex_);

    // cast to base type
    boost::shared_ptr<GoalTolerancesAware> p = boost::dynamic_pointer_cast<GoalTolerancesAware>(object);

    if (p)
    {
      // get a weak reference to the pointer
      boost::weak_ptr<GoalTolerancesAware> q(p);

      // create the bound function
      GoalTolerancesSetCallback_t f = boost::bind(&T::setGoalTolerances, object.get(), _1);

      // add to the list
      ptr_goalTolerancesSetCallbacks_.push_back(ptr_func_tuple_t(q, f));

      // make sure the current goal is propagated
      f(goalTolerances());
    }
  }

  /**
   * @brief Convenience method to use the callback mechanism to propagate new goal tolerances
   *        to other GoalTolerancesAware objects. This version allows raw pointers where the
   *        object will live until manually deleted.
   * @param object Object that will get new goal tolerances set
   */
  template <typename T>
  void propagateGoalTolerancesTo(T* object)
  {
    boost::lock_guard<boost::mutex> lock(goaltolerances_propagate_mutex_);

    // create the bound function
    GoalTolerancesSetCallback_t f = boost::bind(&T::setGoalTolerances, object, _1);

    // add to the list
    goalTolerancesSetCallbacks_.push_back(f);


    // make sure the current goal is propagated
    f(goalTolerances());
  }

protected:
  /**
   * @brief The goal tolerances appropriate for this object
   */
  nav_core::GoalTolerances::Ptr goal_tolerances_;

private:
  /**
   * @brief List of ptr-functions tuples to call when the goal tolerance has been set.
   *        elements will be automatically removed when the weak ptr points to nothing.
   */
  std::list<ptr_func_tuple_t> ptr_goalTolerancesSetCallbacks_;

  /**
   * @brief List of functions to call when the goal tolerance has been set.
   */
  std::list<GoalTolerancesSetCallback_t> goalTolerancesSetCallbacks_;

  /**
   * @brief Mutex to protect against concurrent access to the stl list
   */
  boost::mutex goaltolerances_propagate_mutex_;
};

}  // namespace nav_core

#endif  // NAV_CORE_GOAL_TOLERANCES_AWARE
