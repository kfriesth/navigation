/**
Software License Agreement (proprietary)

\file      diagnostic_publisher.cpp
\authors   Guillaume Autran <gautran@clearpathrobotics.com>
\copyright Copyright (c) 2016, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#include <string>
#include "move_base/diagnostic_publisher.h"


DiagnosticPublisher::DiagnosticPublisher(ros::NodeHandle& nh, const std::string& name)
    : nh_(nh)
    , timer_(nh_.createTimer(ros::Duration(1), &DiagnosticPublisher::timerCb, this))
    , updater_(new diagnostic_updater::Updater(nh))
    , name_(name)
    , level_(diagnostic_msgs::DiagnosticStatus::OK)
    , message_("Initialized.")
{
  updater_->setHardwareID("none");
  updater_->add(name_, this, &DiagnosticPublisher::diagnosticCb);
}

void DiagnosticPublisher::timerCb(const ros::TimerEvent& event)
{
  updater_->update();
}

void DiagnosticPublisher::diagnosticCb(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.summary(level_, message_);
}

void DiagnosticPublisher::report(const char& level, const std::string& message)
{
  bool force_update = (level_ == level) ? false : true;

  level_ = level;
  message_ = message;

  if (force_update)
  {
    updater_->force_update();
  }
  else
  {
    updater_->update();
  }
}
