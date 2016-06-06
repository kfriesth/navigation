/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <nav_core/nav_goal_manager.h>
#include <nav_core/nav_core_state.h>
#include <nav_core/goal_tolerances_aware.h>

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

  enum MoveBaseState {
    PLANNING,
    CONTROLLING,
    CLEARING
  };

  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class MoveBase : public nav_core::GoalTolerancesAware
  {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf::TransformListener& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

      /**
       * @brief  Performs a control cycle
       * @param goal A reference to the goal to pursue
       * @param global_plan A reference to the global plan being used
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);

    private:
      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);


      /**
       * @brief  A service call that can be made to force a replan of the current goal
       * @param  req The service request
       * @param  resp The service reponse
       * @return True if planning succeeded, false otherwise
       */
      bool replanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Publishes a velocity command of zero to the base
       */
      void publishZeroVelocity();

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();

      void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      /**
       * @brief  Computes the X-Y-theta distance between two poses
       * @param p1 The first pose
       * @param p1 The second pose
       * @return The 3D (X-Y-theta) distance between the poses
       */
      double distanceXYTheta(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      /**
       * @brief Used to retrieve an instance of the specified global planner plugin.
       * Creates an instance of the global planner plugin and saves it to cache if it does not exist.
       * Otherwise, this function will return the global planner plugin instanced saved in cache.
       * @param plugin_name Name of the planner plugin.
       * @return Pointer to the global planner plugin instance.
       */
      boost::shared_ptr<nav_core::BaseGlobalPlanner> getGlobalPlannerPlugin(std::string plugin_name);

      /**
       * @brief Used to retrieve the current global planner plugin.
       * @return Pointer to the global planner plugin instance.
       */
      nav_core::BaseGlobalPlanner::Ptr getCurrentGlobalPlannerPlugin();

      /**
       * @brief Used to retrieve the current local planner plugin.
       * @return Pointer to the local planner plugin instance.
       */
      nav_core::BaseLocalPlanner::Ptr getCurrentLocalPlannerPlugin();

      /**
       * @brief Used to undo changes made by the last-executed recovery behavior
       */
      void revertRecoveryChanges();

      /**
       * @brief Resets the move_base and RecoveryManager's indices.
       */
      void resetRecoveryIndices();

      /**
       * @brief Timer callback used to trigger sending of action server feedback.
       * This is needed so we can get feedback messages during planning.
       */
      void asFeedbackTimerCallback(const ros::TimerEvent&);

      /**
       * @brief Custom preemption callback to be called when the action server detects preemption
       */
      void asPreemptCallback();

      /**
       * @brief Function that resets the recovery cycle counter and stores the
       * current pose of the robot based on the global costmap.
       */
      void resetRecoveryCycleState();

      /**
       * @brief Convenience function to return the current pose of the robot
       * based on a given costmap.
       * @param costmap Pointer to costmap object.
       * @param[out] robot_pose The pose of the robot on the costmap.
       * @return True if successful, false otherwise.
       */
      bool getCurrentRobotPose(costmap_2d::Costmap2DROS* costmap,
        geometry_msgs::PoseStamped& robot_pose);

      /**
       * @brief Function to decide if too many cycles of recovery has been executed
       * in short range and if so to force goal abort up the stack.
       * @return True if goal should be aborted, false otherwise.
       */
      bool decideOnForcedGoalAbort();

      tf::TransformListener& tf_;

      MoveBaseActionServer* as_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      std::map<std::string, boost::shared_ptr<nav_core::BaseGlobalPlanner> > global_planner_cache_;
      std::string robot_base_frame_, global_frame_;

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      unsigned int recovery_index_;
      int active_recovery_index_;  // Intentionally signed

      tf::Stamped<tf::Pose> global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      double conservative_reset_dist_, clearing_radius_;
      ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_, replan_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;

      //set up the planner's thread
      bool runPlanner_;
      boost::mutex planner_mutex_;
      boost::recursive_mutex clear_costmap_mutex_;
      boost::condition_variable planner_cond_;
      geometry_msgs::PoseStamped planner_goal_;
      geometry_msgs::PoseStamped last_failed_goal_;
      boost::thread* planner_thread_;


      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;

      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
      bool recovery_cleanup_requested_;

      ros::Timer as_feedback_timer_;

      nav_core::NavGoalMananger::Ptr goal_manager_;

      /**
       * @brief Smart pointer to container object to share costmaps and planners
       */
      nav_core::State::Ptr nav_core_state_;

      /**
       * @brief The last time the action server triggered the execute callback.
       *        Initial value is the start of the epoch
       */
      ros::Time last_execute_callback_;

      /**
       * @brief The number of seconds that are required to elapse between calls to
       *        the execute callback via the actions server. Calls faster than this
       *        will cause the goal to be aborted. Default 0.5 (seconds)
       */
      double minimum_goal_spacing_seconds_;

      /*
       * @brief Counter for the number of times move_base has gone into recovery.
       */
      int recovery_cycle_counter_;

      /**
       * @brief Counter cap for the number of times move_base has gone into recovery
       * but the robot hasn't moved in any meaningful way.
       */
      int recovery_cycle_counter_cap_;

      /**
       * @brief Euclidean distance cap on the amount of robot travel before resetting
       * the recovery cycle counters.
       */
      double recovery_cycle_move_cap_;

      /**
       * @brief Record of the last pose at which the recovery cycle counter is reset.
       */
      geometry_msgs::PoseStamped last_pose_at_recovery_reset_;
  };
};
#endif

