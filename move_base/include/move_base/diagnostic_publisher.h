/**
Software License Agreement (proprietary)

\file      diagnostic_publisher.h
\authors   Peiyi Chen <pchen@clearpathrobotics.com>
\copyright Copyright (c) 2016, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, is not permitted without the
express permission of Clearpath Robotics.
*/
#ifndef MOVE_BASE_DIAGNOSTIC_PUBLISHER_H
#define MOVE_BASE_DIAGNOSTIC_PUBLISHER_H

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <string>
#include <map>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>


/**
 * Class for convenient publishing of diagnostic status messages.
 * The format used to fill the diagnostic status messages matches the format expected by error_handler node.
 */
class DiagnosticPublisher
{
public:
  typedef boost::shared_ptr<DiagnosticPublisher> Ptr;

  ~DiagnosticPublisher();

  DiagnosticPublisher(ros::NodeHandle& nh, const std::string& name);

  void report(const char& level, const std::string& message);

private:
  DiagnosticPublisher();
  DiagnosticPublisher(const DiagnosticPublisher& copy);
  DiagnosticPublisher& operator=(const DiagnosticPublisher& copy);

  void diagnosticCb(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void timerCb(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  ros::Timer timer_;
  boost::scoped_ptr<diagnostic_updater::Updater> updater_;
  std::string name_;
  char level_;
  std::string message_;
};

#endif  // MOVE_BASE_DIAGNOSTIC_PUBLISHER_H
