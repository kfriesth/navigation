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
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <autonomy_msgs_utils/attribute_helper.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>

namespace move_base {

  MoveBase::MoveBase(tf::TransformListener& tf) :
    tf_(tf),
    as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false),
    new_global_plan_(false), recovery_cleanup_requested_(false), nav_core_state_(new nav_core::State),
    recovery_cycle_counter_(0),
    recovery_cycle_counter_cap_(100),
    recovery_cycle_move_cap_(2.0)
  {
    goal_manager_.reset(new nav_core::NavGoalMananger);

    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    // register custom preemption callback so we can respond
    // to changes even if the action server thread is occupied
    as_->registerPreemptCallback(boost::bind(&MoveBase::asPreemptCallback, this));

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    private_nh.param("recovery_cycle_counter_cap", recovery_cycle_counter_cap_, 7);
    private_nh.param("recovery_cycle_move_cap", recovery_cycle_move_cap_, 3.0);

    // Load goal tolerances (these will eventually be dynamic and come from the action server)
    // The goal tolerances contructor loads from goal_tolerance_xy and goal_tolerance_yaw
    goal_tolerances_ = nav_core::GoalTolerances::Ptr(new nav_core::GoalTolerances(private_nh));

    default_goal_tolerances_ = nav_core::GoalTolerances::Ptr(new nav_core::GoalTolerances(private_nh));

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //for comanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::AugmentedMoveBaseActionGoal>("goal", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    private_nh.param<double>("minimum_goal_spacing_seconds", minimum_goal_spacing_seconds_, 0.5);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    // Create the FootprintSetCollection object
    footprint_set_collection_ = nav_core::FootprintSetCollection::Ptr(
      new nav_core::FootprintSetCollection(private_nh, planner_costmap_ros_->getCostmap()->getResolution() ) );

    curr_footprint_set_ = footprint_set_collection_->getSet("default");
    if (!curr_footprint_set_)
    {
      ROS_FATAL("move_base: No default footprints were specified on the parameter server for this robot.");
      exit(1);
    }

    //initialize the global planner
    try {
      //check if a non fully qualified name has potentially been passed in
      if(!bgp_loader_.isClassAvailable(global_planner)){
        std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i){
          if(global_planner == bgp_loader_.getName(classes[i])){
            //if we've found a match... we'll get the fully qualified name and break out of the loop
            ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                global_planner.c_str(), classes[i].c_str());
            global_planner = classes[i];
            break;
          }
        }
      }

      planner_ = getGlobalPlannerPlugin(global_planner);

      // Call the planner's reset method so that if it chooses to it can take actions based on it
      planner_->resetPlanner();

    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try {
      //check if a non fully qualified name has potentially been passed in
      if(!blp_loader_.isClassAvailable(local_planner)){
        std::vector<std::string> classes = blp_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i){
          if(local_planner == blp_loader_.getName(classes[i])){
            //if we've found a match... we'll get the fully qualified name and break out of the loop
            ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                local_planner.c_str(), classes[i].c_str());
            local_planner = classes[i];
            break;
          }
        }
      }

      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->setGoalManager(goal_manager_);
      propagateGoalTolerancesTo(tc_);
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);

    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    replan_srv_ = private_nh.advertiseService("replan", &MoveBase::replanService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we're all set up now so we can start the action server
    as_->start();

    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    // Create and start the timer for publishing action server feedback
    as_feedback_timer_ = nh.createTimer(ros::Duration(0.05), &MoveBase::asFeedbackTimerCallback, this);

    nav_core_state_->global_costmap_ = planner_costmap_ros_;
    nav_core_state_->local_costmap_ = controller_costmap_ros_;
    nav_core_state_->global_planner_ = planner_;
    nav_core_state_->local_planner_ = tc_;

    // Initialize the last faild goal so that we can see goal abort message
    last_failed_goal_.pose.position.x = FLT_MAX;

    // Initialize the recovery counters and indices
    resetRecoveryIndices();
    resetRecoveryCycleState();

    // Initalise diagnositcs
    state_diagnostics_.reset(new DiagnosticPublisher(private_nh, "Move_base state"));
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        //check if a non fully qualified name has potentially been passed in
        if(!bgp_loader_.isClassAvailable(config.base_global_planner)){
          std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
          for(unsigned int i = 0; i < classes.size(); ++i){
            if(config.base_global_planner == bgp_loader_.getName(classes[i])){
              //if we've found a match... we'll get the fully qualified name and break out of the loop
              ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                  config.base_global_planner.c_str(), classes[i].c_str());
              config.base_global_planner = classes[i];
              break;
            }
          }
        }

        // wait for the current planner to finish planning
        boost::unique_lock<boost::mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        runPlanner_ = false;
        recovery_cleanup_requested_ = true;
        state_ = PLANNING;
        recovery_trigger_ = PLANNING_R;
        publishZeroVelocity();

        planner_ = getGlobalPlannerPlugin(config.base_global_planner);

        // Call the planner's reset method so that if it chooses to it can take actions based on it
        planner_->resetPlanner();

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex)
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        //check if a non fully qualified name has potentially been passed in
        ROS_INFO("Loading local planner: %s", config.base_local_planner.c_str());
        if(!blp_loader_.isClassAvailable(config.base_local_planner)){
          std::vector<std::string> classes = blp_loader_.getDeclaredClasses();
          for(unsigned int i = 0; i < classes.size(); ++i){
            if(config.base_local_planner == blp_loader_.getName(classes[i])){
              //if we've found a match... we'll get the fully qualified name and break out of the loop
              ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                  config.base_local_planner.c_str(), classes[i].c_str());
              config.base_local_planner = classes[i];
              break;
            }
          }
        }
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->setGoalManager(goal_manager_);
        propagateGoalTolerancesTo(tc_);
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex)
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    last_config_ = config;

    // planners could have changes so updating pointers
    nav_core_state_->global_planner_ = planner_;
    nav_core_state_->local_planner_ = tc_;
  }

  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::AugmentedMoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;
    goal_manager_->setCurrentGoal(nav_core::NavGoal(*goal));

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    tf::Stamped<tf::Pose> global_pose;

    //clear the planner's costmap
    planner_costmap_ros_->getRobotPose(global_pose);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.getOrigin().x();
    double y = global_pose.getOrigin().y();
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    controller_costmap_ros_->getRobotPose(global_pose);

    clear_poly.clear();
    x = global_pose.getOrigin().x();
    y = global_pose.getOrigin().y();

    pt.x = x - size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_x / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    boost::unique_lock<boost::recursive_mutex> lock(clear_costmap_mutex_);

    //clear the costmaps
    planner_costmap_ros_->resetLayers();
    controller_costmap_ros_->resetLayers();

    // Call one update on the maps to reinitialize data
    planner_costmap_ros_->updateMap();
    controller_costmap_ros_->updateMap();
    return true;
  }

  bool MoveBase::replanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    // If there is an active goal, wake up the planning thread to kick a replan
    if(as_->isActive()){
      boost::unique_lock<boost::mutex> lock(planner_mutex_);
      runPlanner_ = true;
      planner_cond_.notify_one();
      lock.unlock();
    }
    return true;
  }

  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)){
      ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
      return false;
    }
    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id == "")
      tf::poseStampedTFToMsg(global_pose, start);
    else
      start = req.start;

    //update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;

    // NOTE: The implementation of makePlan is responsible for locking the costmap
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                goal_manager_->setCurrentGoal(nav_core::NavGoal(p));

                // NOTE: The implementation of makePlan is responsible for locking the costmap
                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    delete dsrv_;

    as_feedback_timer_.stop();
    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();

    recovery_behaviors_.clear();
  }

  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, int& planner_status){
    boost::unique_lock< boost::recursive_mutex > cm_lock(clear_costmap_mutex_);

    //check if the costmap is current before planning on it
    if (!planner_costmap_ros_->isCurrent())
    {
      ROS_DEBUG_NAMED("move_base", "Planner costmap ROS is not current, unable to create global plan");
      return false;
    }

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);
    goal_manager_->setCurrentGoal(nav_core::NavGoal(goal));

    // NOTE: The implementation of makePlan is responsible for locking the costmap
    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan, planner_status) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    tf::Stamped<tf::Pose> goal_pose, global_pose;
    poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try{
      tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    while(n.ok()){
      //check if we should run the planner (the mutex is locked)
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }

      // If we just swapped planners, we should clean up any recoveries from the last planner.
      // We do this here instead of in the execute thread to make use of the planner_mutex_
      // which ensures the variable is not being modified by reconfigureCB at the same time.
      if (recovery_cleanup_requested_)
      {
        recovery_cleanup_requested_ = false;
        revertRecoveryChanges();
        resetRecoveryIndices();
        resetRecoveryCycleState();
      }

      ros::Time start_time = ros::Time::now();

      //time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::PoseStamped planned_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      planner_plan_->clear();
      int planner_status = nav_core::status::UNDEFINED;
      bool gotPlan = n.ok() && makePlan(planned_goal, *planner_plan_, planner_status);

      if (planner_status == nav_core::status::FATAL)
      {
        // Hit an unrecoverable error, move_base should abort planning
        state_ = FAILED;
        failure_mode_ = PLANNING_F;
        runPlanner_ = false;
        publishZeroVelocity();
      }
      else if(gotPlan){
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();

        // Update the last_valid_plan_ time regardless, as it will keep us from triggering recovery behaviours if we
        // simply interrupted our last goal with a new one.
        last_valid_plan_ = ros::Time::now();

        // makePlan can take time. While planning, it's possible that we received a new goal. We should check for this
        // condition, and if it's true, we should carry on as if we did not succeed in planning this goal, which will
        // cause us to move on and re-start this thread for the new goal.

        if (!nav_core::NavGoal(planned_goal).equalPose(goal_manager_->currentGoal()))
        {
          ROS_INFO("Planned goal and latest received goal do not match. Assuming new goal issued; ignoring this "
                   "plan.");
        }
        else if (!goal_manager_->activeGoal())
        {
          ROS_INFO("Goal has become inactive; ignoring this plan.");
        }
        else
        {
          planner_plan_ = latest_plan_;
          latest_plan_ = temp_plan;
          new_global_plan_ = true;

          ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

          /* As soon as we find a plan even if during recovery, set the state to CONTROLLING.
           * In CONTROLLING state if isGoalReached returns true the cycle returns true so there's no
           * need to check if we have or have not reached the goal to change the state.
           */
          state_ = CONTROLLING;

          // Reset the failed goal record (so that if we fail on that goal again in the future we show a log message)
          last_failed_goal_.pose.position.x = FLT_MAX;

          if(planner_frequency_ <= 0)
            runPlanner_ = false;
        }

        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit
        lock.lock();
        if(ros::Time::now() >= attempt_end && runPlanner_){
          //we'll move into our obstacle clearing mode
          /* We're about to change the state to RECOVERY.
           * Since performing the recovery process is slower than this
           * while loop, we should set runPlanner_ to false to suspend
           * the planner thread or else we could plan in the middle of
           * the recovery (e.g. before the obstacles have completely been
           * disabled). We will reset this flag to true after recovery is done.
           */
          runPlanner_ = false;
          state_ = RECOVERY;
          recovery_trigger_ = PLANNING_R;
          publishZeroVelocity();
        }
        lock.unlock();
      }

      //take the mutex for the next iteration
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  void MoveBase::applyGoalToleranceAttributes(const move_base_msgs::AugmentedMoveBaseGoalConstPtr& move_base_goal,
                                              nav_core::GoalTolerances::Ptr tolerance)
  {
    // Get the XY goal tolerance with a default of the param file value
    autonomy_msgs::utils::getAttribute(*move_base_goal,
                                       "tolerance_xy",
                                       tolerance->goal_tolerance_xy_,
                                       default_goal_tolerances_->goal_tolerance_xy_);

    // Get the Yaw goal tolerance with a default of the param file value
    autonomy_msgs::utils::getAttribute(*move_base_goal,
                                       "tolerance_yaw",
                                       tolerance->goal_tolerance_yaw_,
                                       default_goal_tolerances_->goal_tolerance_yaw_);

    // if any relative tolerance is defined, we will enable relative tolerance checking
    float tolerance_left = 0;
    float tolerance_right = 0;
    float tolerance_front = 0;
    float tolerance_back = 0;
    bool use_relative_tolerances = false;

    use_relative_tolerances |= autonomy_msgs::utils::getAttribute(*move_base_goal, "tolerance_left", tolerance_left);
    use_relative_tolerances |= autonomy_msgs::utils::getAttribute(*move_base_goal, "tolerance_right", tolerance_right);
    use_relative_tolerances |= autonomy_msgs::utils::getAttribute(*move_base_goal, "tolerance_front", tolerance_front);
    use_relative_tolerances |= autonomy_msgs::utils::getAttribute(*move_base_goal, "tolerance_back", tolerance_back);

    if (use_relative_tolerances)
    {
      tolerance->setRelativeTolerances(tolerance_left, tolerance_right, tolerance_front, tolerance_back);
    }
  }

  void MoveBase::executeCb(const move_base_msgs::AugmentedMoveBaseGoalConstPtr& move_base_goal)
  {
    ros::Time ros_time_now = ros::Time::now();

    // CORE-3994
    // Protect against goals being sent too quickly by aborting them
    // This is a stop-gap until we can use a full action server
    const double consecutive_goal_time = (ros_time_now - last_execute_callback_).toSec();
    if (consecutive_goal_time < minimum_goal_spacing_seconds_)
    {
      ROS_ERROR_THROTTLE(1, "move_base: Aborting on goal because it was sent too soon after the last goal %gs < %gs",
                             consecutive_goal_time, minimum_goal_spacing_seconds_);
      abortGoal("Aborting on goal because it was sent too soon after the last goal");
      return;
    }
    last_execute_callback_ = ros_time_now;

    // The Action Server can call the callback on a null ptr. Before doing this it will
    // issue a ROS_ERROR with the text:
    //    Attempting to accept the next goal when a new goal is not available
    // For robustness, we need to check for that null ptr here and set the state to
    // aborted rather than crashing.
    // CORE-3682
    if (move_base_goal.get() == NULL)
    {
      abortGoal("Aborting on goal because it was a NULL pointer");
      return;
    }

    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      abortGoal("Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::PoseStamped goal = move_base_goal->target_pose;
    goal_manager_->setCurrentGoal(nav_core::NavGoal(goal));

    // Translate tolerance related attributes into goal tolerances
    applyGoalToleranceAttributes(move_base_goal, goal_tolerances_);

    // Update goal tolerance for this object and all designated propagated objects (planners, tracker, etc)
    setGoalTolerances(goal_tolerances_);

    //we have a goal so start the planner
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::Rate r(controller_frequency_);
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();

    ros::NodeHandle n;
    while(n.ok())
    {
      if(c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      if(as_->isPreemptRequested()){
        if(as_->isNewGoalAvailable()){
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::AugmentedMoveBaseGoalConstPtr new_goal_ptr = as_->acceptNewGoal();

          if (new_goal_ptr.get() == NULL)
          {
            abortGoal("Aborting on goal because it was NULL");
            return;
          }

          move_base_msgs::AugmentedMoveBaseGoal new_goal = *new_goal_ptr;
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            abortGoal("Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = new_goal.target_pose;
          goal_manager_->setCurrentGoal(nav_core::NavGoal(goal));

          //we'll make sure that we reset our state for the next execution cycle
          revertRecoveryChanges();
          resetRecoveryIndices();
          resetRecoveryCycleState();
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
        }
        else {
          goal_manager_->setActiveGoal(false);  // setting no active goal

          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      bool done = executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      if(done)
        return;

      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN_THROTTLE(5, "Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    abortGoal("Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  double MoveBase::distanceXYTheta(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    double diff_x = p1.pose.position.x - p2.pose.position.x;
    double diff_y = p1.pose.position.y - p2.pose.position.y;
    double diff_yaw = angles::shortest_angular_distance(tf::getYaw(p1.pose.orientation),
                                                        tf::getYaw(p2.pose.orientation));

    return ::sqrt(diff_x * diff_x + diff_y * diff_y + diff_yaw * diff_yaw);
  }

  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    tf::Stamped<tf::Pose> global_pose;
    planner_costmap_ros_->getRobotPose(global_pose);
    geometry_msgs::PoseStamped current_position;
    tf::poseStampedTFToMsg(global_pose, current_position);

    //check to see if we've moved far enough to reset our oscillation timeout
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index
      if(recovery_trigger_ == OSCILLATION_R)
      {
        revertRecoveryChanges();
        resetRecoveryIndices();
        resetRecoveryCycleState();
      }
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    if(new_global_plan_){
      //make sure to set the new plan flag to false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      if(!tc_->setPlan(*controller_plan_)){
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

        abortGoal("Failed to pass global plan to the controller.");
        return true;
      }
    }

    //the move_base state machine, handles the control logic for navigation
    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan
      case PLANNING:
        {
          state_diagnostics_->report(
              diagnostic_msgs::DiagnosticStatus::OK, 
              "Move_base is in planning state");
          boost::mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      case CONTROLLING:
      {
        state_diagnostics_->report(
            diagnostic_msgs::DiagnosticStatus::OK, 
            "Move_base is in controlling state");
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //check to see if we've reached our goal
        if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          as_->setSucceeded(move_base_msgs::AugmentedMoveBaseResult(), "Goal reached.");
          return true;
        }

        //check for an oscillation condition
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();
          state_ = RECOVERY;
          recovery_trigger_ = OSCILLATION_R;
        }

        int custom_status = nav_core::status::UNDEFINED;
        bool computeVelocityCommands_return = false;

        {
          // only holding lock while we compute velocity commands
          boost::unique_lock<boost::recursive_mutex> cm_lock(clear_costmap_mutex_);
          boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
          computeVelocityCommands_return = tc_->computeVelocityCommands(cmd_vel, custom_status);
        }

        // Revert the changes done by the recovery behaviour.
        // It is important that we do this regardless of computeVelocityCommands_return or else behaviours such as
        // disable obstacle layer could possibly never be reverted if we haven't moved.
        // This could result in rapid replan and non-zero cmd_vel cycles which in turn cause the breaks
        // to engage-disengage frequently.
        revertRecoveryChanges();

        if (computeVelocityCommands_return)
        {
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );

          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);

          // It is possible for computeVelocityCommands to return true when we are waiting for dynamic costmap logic
          // to timeout. In that case, custom_status == nav_core::status::WAIT. If we are in an OK state
          // where meaningful cmd_vel is being published then we can reset the indices here in move_base
          // as well as in the recovery_manager.
          if (custom_status == nav_core::status::OK)
          {
            last_valid_control_ = ros::Time::now();
            resetRecoveryIndices();
          }

          // Reset the failed goal record (so that if we fail on that goal again in the future we show a log message)
          last_failed_goal_.pose.position.x = FLT_MAX;
        }
        else {
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //check if we've tried to find a valid control for longer than our time limit
          if(ros::Time::now() > attempt_end){
            //we'll move into our obstacle clearing mode
            publishZeroVelocity();
            state_ = RECOVERY;
            recovery_trigger_ = CONTROLLING_R;
          }
          else{
            //otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }

        break;
      }

      //we'll try to clear out space with any user-provided recovery behaviors
      case RECOVERY:
      {
        state_diagnostics_->report(
            diagnostic_msgs::DiagnosticStatus::OK, 
            "Move_base is in recovery state");
        ROS_DEBUG_NAMED("move_base","In recovery state");

        // Decide if this cycle of recovery should proceed as usual or if we should
        // force goal abort
        bool force_abort = decideOnForcedGoalAbort();

        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        if (recovery_behavior_enabled_
          && recovery_index_ < recovery_behaviors_.size()
          && !force_abort)
        {
          if (planner_)
          {
            // This invokes the planner's prepareForPostRecovery method to prepare it for post-recovery actions.
            planner_->prepareForPostRecovery();
          }

          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
          active_recovery_index_ = recovery_index_;
          recovery_behaviors_[recovery_index_]->runBehavior();

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          /* Now that recovery process is finished, we can reset the flag
           * to allow the planner thread to proceed.
           */
          runPlanner_ = true;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
        }
        else {
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();
          state_ = FAILED;
          failure_mode_ = RECOVERY_F;
          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");
        }
        break;
      }
      case FAILED:
      {
        state_diagnostics_->report(
            diagnostic_msgs::DiagnosticStatus::OK, 
            "Move_base is in failed state");
        resetState();
        //  Disable the planner thread
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        //  We need to abort goal and log a message; decide if we should spit out a new log
        bool log_condition = (distanceXYTheta(planner_goal_, last_failed_goal_) >= 1e-3);

        if (failure_mode_ == RECOVERY_F)
        {
          if (recovery_trigger_ == CONTROLLING_R)
          {
            ROS_ERROR_COND(log_condition, "Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            abortGoal("Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if (recovery_trigger_ == PLANNING_R)
          {
            ROS_ERROR_COND(log_condition, "Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            abortGoal("Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if (recovery_trigger_ == OSCILLATION_R)
          {
            ROS_ERROR_COND(log_condition, "Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            abortGoal("Robot is oscillating. Even after executing recovery behaviors.");
          }
        }
        else if (failure_mode_ == PLANNING_F)
        {
          ROS_ERROR_COND(log_condition, "Fatal planning error.");
          abortGoal("Fatal planning error.");
        }

        // Record the failed goal so in the next cycle we don't log a new message
        last_failed_goal_ = planner_goal_;
        return true;
      }
      default:
        state_diagnostics_->report(
            diagnostic_msgs::DiagnosticStatus::WARN, 
            "Move_base is in unknown state");

        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        abortGoal("Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            behavior->setGoalManager(goal_manager_);
            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);

            //tell the behaviors how to access the planners
            nav_core::BaseLocalPlanner::FetchFunction get_local = boost::bind(&MoveBase::getCurrentLocalPlannerPlugin, this);
            behavior->setLocalPlannerFetchFunction(get_local);

            nav_core::BaseGlobalPlanner::FetchFunction get_global = boost::bind(&MoveBase::getCurrentGlobalPlannerPlugin, this);
            behavior->setGlobalPlannerFetchFunction(get_global);

            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->setGoalManager(goal_manager_);
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->setGoalManager(goal_manager_);
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->setGoalManager(goal_manager_);
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_)
        recovery_behaviors_.push_back(rotate);
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState()
  {
    ROS_DEBUG("MoveBase: Resetting state.");
    publishZeroVelocity();  // stop immediately

    goal_manager_->setActiveGoal(false);  // setting no active goal

    // Disable the planner thread
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    revertRecoveryChanges();
    resetRecoveryIndices();
    resetRecoveryCycleState();
    state_ = PLANNING;
    recovery_trigger_ = PLANNING_R;

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  boost::shared_ptr<nav_core::BaseGlobalPlanner> MoveBase::getGlobalPlannerPlugin(std::string plugin_name)
  {
    // Check if the current plugin already exists in the cache
    if (global_planner_cache_.find(plugin_name) == global_planner_cache_.end() )
    {
      // We do not have an instance of this planner in cache, so create one and cache it.
      ros::Time t = ros::Time::now();
      global_planner_cache_.insert(std::make_pair(plugin_name, bgp_loader_.createInstance(plugin_name) ) );
      global_planner_cache_[plugin_name]->setGoalManager(goal_manager_);
      global_planner_cache_[plugin_name]->setFootprintSet(&curr_footprint_set_);
      propagateGoalTolerancesTo(global_planner_cache_[plugin_name]);
      global_planner_cache_[plugin_name]->setNavCoreState(nav_core_state_);
      global_planner_cache_[plugin_name]->initialize(bgp_loader_.getName(plugin_name), planner_costmap_ros_);
      ROS_DEBUG("Created new global planner plugin %s in %f seconds.", plugin_name.c_str(), (ros::Time::now() - t).toSec() );
    }
    else
    {
      ROS_DEBUG("Got cached global planner plugin: %s.", plugin_name.c_str() );
    }

    // Return the cached plugin instance.
    return global_planner_cache_[plugin_name];
  }

  nav_core::BaseGlobalPlanner::Ptr MoveBase::getCurrentGlobalPlannerPlugin()
  {
    return planner_;
  }

  nav_core::BaseLocalPlanner::Ptr MoveBase::getCurrentLocalPlannerPlugin()
  {
    return tc_;
  }

  void MoveBase::revertRecoveryChanges()
  {
    if(recovery_behavior_enabled_ &&
      active_recovery_index_ >= 0 &&
      active_recovery_index_ < static_cast<int>(recovery_behaviors_.size()))
    {
      // First ensure that we're setting velocities to zero so that we're in a safe state
      // in case reverting a behaviour is time consuming
      publishZeroVelocity();

      // Now revert the changed caused by the active behaviour
      recovery_behaviors_[active_recovery_index_]->revertChanges();
    }
  }

  void MoveBase::resetRecoveryIndices()
  {
    recovery_index_ = 0;
    active_recovery_index_ = -1;
  }

  void MoveBase::asFeedbackTimerCallback(const ros::TimerEvent&)
  {
    if (as_->isActive() )
    {
      //update feedback to correspond to our curent position
      tf::Stamped<tf::Pose> global_pose;
      planner_costmap_ros_->getRobotPose(global_pose);
      geometry_msgs::PoseStamped current_position;
      tf::poseStampedTFToMsg(global_pose, current_position);

      //push the feedback out
      move_base_msgs::AugmentedMoveBaseFeedback feedback;
      feedback.base_position = current_position;
      as_->publishFeedback(feedback);
    }
  }

  void MoveBase::asPreemptCallback()
  {
    // Setting no active goal, when the preempt filters though it will
    // set a new active goal. For now we will allow behaviours to stop
    // due to no active goal
    goal_manager_->setActiveGoal(false);
  }

  bool MoveBase::getCurrentRobotPose(costmap_2d::Costmap2DROS* costmap,
    geometry_msgs::PoseStamped& robot_pose)
  {
    if (costmap == NULL)
    {
      return false;
    }

    tf::Stamped<tf::Pose> pose;
    if (!costmap->getRobotPose(pose) )
    {
      return false;
    }

    // Get the current pose of the robot
    tf::poseStampedTFToMsg(pose, robot_pose);

    return true;
  }

  void MoveBase::resetRecoveryCycleState()
  {
    ROS_INFO("MoveBase: Resetting recovery counters");

    // Reset the counter
    recovery_cycle_counter_ = 0;

    // Store the pose
    geometry_msgs::PoseStamped curr_pose;
    if (getCurrentRobotPose(planner_costmap_ros_, curr_pose) )
    {
      last_pose_at_recovery_reset_ = curr_pose;
    }
  }

  bool MoveBase::decideOnForcedGoalAbort()
  {
    bool abort = false;
    bool robot_has_moved_sufficiently = false;
    double travelled_dist = 0.0;

    // Decide if robot has moved sufficiently
    geometry_msgs::PoseStamped curr_pose;
    if (getCurrentRobotPose(planner_costmap_ros_, curr_pose) )
    {
      travelled_dist = distance(curr_pose, last_pose_at_recovery_reset_);
      robot_has_moved_sufficiently = (travelled_dist > recovery_cycle_move_cap_);
    }

    if (robot_has_moved_sufficiently)
    {
      ROS_INFO("MoveBase: Robot has moved sufficiently (%.2f with cap %.2f)."
        " Resetting recovery cycle counters.", travelled_dist, recovery_cycle_move_cap_);

      resetRecoveryCycleState();
    }
    else if (recovery_cycle_counter_ > recovery_cycle_counter_cap_)
    {
      ROS_INFO("MoveBase: Recovery cycle counter cap reached and "
        "robot has not moved enough. Forcing goal abort.");

      resetRecoveryCycleState();
      abort = true;
    }
    else
    {
      recovery_cycle_counter_++;

      ROS_INFO("MoveBase: Executing recovery cycle number %d/%d.",
        recovery_cycle_counter_, recovery_cycle_counter_cap_);
    }

    return abort;
  }

  void MoveBase::abortGoal(const std::string& abort_message)
  {
    // Until CORE-4329, aborts will be throttled to 1 second
    ros::Time next_valid_abort_time = last_abort_goal_ + ros::Duration(1.0);

    // sleep for difference between next abort time and now (negative duration will return immediately)
    (next_valid_abort_time - ros::Time::now()).sleep();

    // update last abort time for throttling
    last_abort_goal_ = ros::Time::now();

    // do the abort
    as_->setAborted(move_base_msgs::AugmentedMoveBaseResult(), abort_message);
    goal_manager_->setActiveGoal(false);  // setting no active goal
  }

};
