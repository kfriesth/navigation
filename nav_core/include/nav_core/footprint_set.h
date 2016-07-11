#ifndef NAV_CORE_FOOTPRINT_SET_H
#define NAV_CORE_FOOTPRINT_SET_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Point.h>
#include <vector>
#include <string>
#include <costmap_2d/footprint.h>
#include <map>

namespace nav_core
{

namespace Footprint
{
namespace Mode
{
enum Index
{
  // List of footprint modes starting with the default normal mode of value -1.
  // New modes can be added to this after the NORMAL mode. Order does not matter.
  NORMAL = -1,
  NORMAL_PADDED,
  NARROW,
  NARROW_PADDED,
  DOCK,
  DOCK_PADDED,
  // Index of the last item and total number of items
  // Note: Do NOT modify theirs values or order!
  MODE_LASTINDEX,
  MODE_COUNT
};
}  // namespace Mode

namespace Param
{
  const static std::string NORMAL = "footprint_normal";
  const static std::string NARROW = "footprint_narrow";
  const static std::string DOCK   = "footprint_dock";
}  // namespace Param

struct Coefficients
{
  double front_c0, front_c1, front_c2;
  double left_c0, left_c1, left_c2;
  double right_c0, right_c1, right_c2;
  double rear_c0, rear_c1, rear_c2;
};
}  // namespace Footprint

/**
 * This class defines and manages a single set of footprints for planner and tracker.
 */
class FootprintSet
{
public:
  typedef boost::shared_ptr<FootprintSet> Ptr;
  typedef std::vector<geometry_msgs::Point> Polygon;

public:
  /**
   * @brief Constructor function.
   * @param[in] nh Nodehandle whose namespace will be used to pull in footprint parameters.
   * @param[in] set_name Name of the footprint set that should be retrieved and saved in this object.
   * @param[in] padding The padding applied to padded planner footprints in meters.
   */
  FootprintSet(ros::NodeHandle& nh, std::string set_name = "default", double padding = 0.0)
    : nh_(nh), footprint_set_(set_name), planner_footprint_padding_(padding)
  {
    initialize();
  }

  /**
   * @brief Function used to retrieve footprints used by the planner from the planner_footprints_.
   * @param[in] idx_mode The Footprint::Mode enum corresponding to which footprint to return.
   * @param[out] footprint The requested footprint polygon.
   * @return True on success. False on error.
   */
  bool getPlannerFootprint(Footprint::Mode::Index idx_mode, Polygon& footprint)
  {
    try
    {
      footprint = planner_footprints_.at(idx_mode);
    }
    catch (std::out_of_range&)
    {
      ROS_ERROR("FootprintSet %s: Requested planner footprint with mode id '%d' does not exist.",
                footprint_set_.c_str(), idx_mode);
      return false;
    }

    return true;
  }

  /**
   * @brief Function used to retrieve footprints used by the tracker for collision checking.
   * @param[in] idx_mode The Footprint::Mode enum corresponding to which footprint to return.
   * @param[in] velocity The velocity of the robot in m/s used to calculate the dynamic footprint expansions.
   * @param[out] footprint The requested footprint.
   * @param[in] dynamic_length Enables (TRUE) or disables (FALSE) dynamic length expansion of the footprint.
   * @param[in] dynamic_width Enables (TRUE) or disables (FALSE) dynamic width expansion of the footprint.
   * @return True on success. False on error.
   */
  bool getTrackerFootprint(Footprint::Mode::Index idx_mode, double velocity, Polygon& footprint,
                           bool dynamic_length = false, bool dynamic_width = false)
  {
    double speed = fabs(velocity);
    double speed_sq = speed * speed;

    Footprint::Coefficients coefficients;
    try
    {
      coefficients = tracker_footprint_coefficients_.at(idx_mode);
    }
    catch (std::out_of_range&)
    {
      ROS_ERROR("FootprintSet %s: Requested tracker footprint with mode id '%d' does not exist.",
                footprint_set_.c_str(), idx_mode);
      return false;
    }

    double padding = 0;

    if (idx_mode == Footprint::Mode::NORMAL_PADDED ||
        idx_mode == Footprint::Mode::NARROW_PADDED ||
        idx_mode == Footprint::Mode::DOCK_PADDED)
    {
      padding = tracker_footprint_padding_;
    }

    // Assumes a rectangular footprint
    double front = coefficients.front_c0 + padding;
    double rear = coefficients.rear_c0 + padding;
    double left = coefficients.left_c0 + padding;
    double right = coefficients.right_c0 + padding;

    if (dynamic_length)
    {
      if (velocity > 0)
      {
        front += coefficients.front_c1 * speed + coefficients.front_c2 * speed_sq;
      }
      if (velocity < 0)
      {
        rear += coefficients.rear_c1 * speed + coefficients.rear_c2 * speed_sq;
      }
    }

    if (dynamic_width)
    {
      left += coefficients.left_c1 * speed + coefficients.left_c2 * speed_sq;
      right += coefficients.right_c1 * speed + coefficients.right_c2 * speed_sq;
    }

    rear *= -1.0;
    right *= -1.0;

    // Construct the footprint
    footprint.clear();
    geometry_msgs::Point pt;
    pt.z = 0.0;

    pt.x = front;
    pt.y = left;
    footprint.push_back(pt);
    pt.x = front;
    pt.y = right;
    footprint.push_back(pt);
    pt.x = rear;
    pt.y = right;
    footprint.push_back(pt);
    pt.x = rear;
    pt.y = left;
    footprint.push_back(pt);

    return true;
  }

  /**
   * @brief Get the footpring clearing padding for the specified footprint mode.
   * @param[in] idx_mode The enum for the footprint mode.
   * @param[out] footprint_clearing_padding The retrieved footprint_clearing_padding parameter.
   * @return True on success. False on error.
   */
  bool getFootprintClearingPadding(Footprint::Mode::Index idx_mode, double& footprint_clearing_padding)
  {
    try
    {
      footprint_clearing_padding = footprint_clearing_paddings_.at(idx_mode);
    }
    catch (std::out_of_range&)
    {
      ROS_ERROR("FootprintSet %s: Requested footprint clearing padding with mode id '%d' does not exist.",
                footprint_set_.c_str(), idx_mode);
      return false;
    }

    return true;
  }

  /**
   * @brief Function used to set planner footprints. This overrides default initializations from the param server.
   * @param[in] idx_mode The Footprint::Mode enum corresponding to which footprint to set.
   * @param[in] footprint The new footprint to use.
   */
  void setPlannerFootprint(Footprint::Mode::Index idx_mode, Polygon& footprint)
  {
    planner_footprints_[idx_mode] = footprint;
  }

  /**
   * @brief Function used to set the footprint padding applied to padded planner footprints.
   * Footprints must be re-created via a call to initialize() for this new padding to take effect.
   * @param[in] padding The new padding for planner footprints in meters.
   */
  void setPlannerFootprintPadding(double padding)
  {
    planner_footprint_padding_ = padding;
  }

  /**
   * @brief Function used to set the footprint padding applied to the padded tracker footprints.
   * @param[in] padding The new padding for tracker footprints in meters.
   */
  void setTrackerFootprintPadding(double padding)
  {
    tracker_footprint_padding_ = padding;
  }

  /**
   * @brief Initialization function that pulls in the relevant parameters and constructs the footprints.
   * This function can also be used to regenerate the footprints after any parameter changes.
   */
  void initialize()
  {
    planner_footprints_.clear();
    tracker_footprint_coefficients_.clear();

    // First pull in all the planner footprints (these are the ones defined for the costmaps)
    XmlRpc::XmlRpcValue footprint_xml;
    Polygon footprint;

    if (nh_.getParam("footprint_sets/" + footprint_set_ + "/planner/" + Footprint::Param::NORMAL, footprint_xml) )
    {
      footprint = costmap_2d::makeFootprintFromXMLRPC(footprint_xml, footprint_set_ + "/planner/" + Footprint::Param::NORMAL);
      planner_footprints_[Footprint::Mode::NORMAL] = footprint;

      // Create padded version of normal planner footprint
      costmap_2d::padFootprint(footprint, planner_footprint_padding_);
      planner_footprints_[Footprint::Mode::NORMAL_PADDED] = footprint;
    }

    if (nh_.getParam("footprint_sets/" + footprint_set_ + "/planner/" + Footprint::Param::NARROW, footprint_xml) )
    {
      footprint = costmap_2d::makeFootprintFromXMLRPC(footprint_xml, footprint_set_ + "/planner/" + Footprint::Param::NARROW);
      planner_footprints_[Footprint::Mode::NARROW] = footprint;

      // Create padded version of narrow planner footprint
      costmap_2d::padFootprint(footprint, planner_footprint_padding_);
      planner_footprints_[Footprint::Mode::NARROW_PADDED] = footprint;
    }

    if (nh_.getParam("footprint_sets/" + footprint_set_ + "/planner/" + Footprint::Param::DOCK, footprint_xml) )
    {
      footprint = costmap_2d::makeFootprintFromXMLRPC(footprint_xml, footprint_set_ + "/planner/" + Footprint::Param::DOCK);
      planner_footprints_[Footprint::Mode::DOCK] = footprint;

      // Create padded version of dock planner footprint
      costmap_2d::padFootprint(footprint, planner_footprint_padding_);
      planner_footprints_[Footprint::Mode::DOCK_PADDED] = footprint;
    }

    // Next we pull in all the tracker footprints (these are the ones whose size vary based on velocity)
    Footprint::Coefficients coefficients;
    double footprint_clearing_padding;

    std::string param_path_prefix = "footprint_sets/" + footprint_set_ + "/tracker/";
    readTrackerFootprintCoefficients(param_path_prefix + Footprint::Param::NORMAL,
                                     planner_footprints_[Footprint::Mode::NORMAL],
                                     coefficients);
    tracker_footprint_coefficients_[Footprint::Mode::NORMAL] = coefficients;
    tracker_footprint_coefficients_[Footprint::Mode::NORMAL_PADDED] = coefficients;
    nh_.param<double>(param_path_prefix + Footprint::Param::NORMAL + "/footprint_clearing_padding",
                      footprint_clearing_padding, 0.0);
    footprint_clearing_paddings_[Footprint::Mode::NORMAL] = footprint_clearing_padding;
    footprint_clearing_paddings_[Footprint::Mode::NORMAL_PADDED] = footprint_clearing_padding;

    readTrackerFootprintCoefficients(param_path_prefix + Footprint::Param::NARROW,
                                     planner_footprints_[Footprint::Mode::NARROW],
                                     coefficients);
    tracker_footprint_coefficients_[Footprint::Mode::NARROW] = coefficients;
    tracker_footprint_coefficients_[Footprint::Mode::NARROW_PADDED] = coefficients;
    nh_.param<double>(param_path_prefix + Footprint::Param::NARROW + "/footprint_clearing_padding",
                      footprint_clearing_padding, 0.0);
    footprint_clearing_paddings_[Footprint::Mode::NARROW] = footprint_clearing_padding;
    footprint_clearing_paddings_[Footprint::Mode::NARROW_PADDED] = footprint_clearing_padding;

    readTrackerFootprintCoefficients(param_path_prefix + Footprint::Param::DOCK,
                                     planner_footprints_[Footprint::Mode::DOCK],
                                     coefficients);
    tracker_footprint_coefficients_[Footprint::Mode::DOCK] = coefficients;
    tracker_footprint_coefficients_[Footprint::Mode::DOCK_PADDED] = coefficients;
    nh_.param<double>(param_path_prefix + Footprint::Param::DOCK + "/footprint_clearing_padding",
                      footprint_clearing_padding, 0.0);
    footprint_clearing_paddings_[Footprint::Mode::DOCK] = footprint_clearing_padding;
    footprint_clearing_paddings_[Footprint::Mode::DOCK_PADDED] = footprint_clearing_padding;
  }

private:
  /**
   * @brief Function used to read in the coefficients of the dynamic footprint used by the tracker.
   * @param[in] param_path The relatively parameter path prefix where the coefficients are stored on the param server.
   * @param[in] default_footprint The default values (footprint) to use if parameters are undefined.
   * @param[out] coefficients The struct containing the footprint coefficients.
   */
  void readTrackerFootprintCoefficients(std::string param_path, Polygon default_footprint, Footprint::Coefficients& coefficients)
  {
    double xmin, ymin, xmax, ymax;
    findFootprintBounds(default_footprint, xmin, ymin, xmax, ymax);

    nh_.param<double>(param_path + "/front_c0", coefficients.front_c0, fabs(xmax) );
    nh_.param<double>(param_path + "/front_c1", coefficients.front_c1, 0.0);
    nh_.param<double>(param_path + "/front_c2", coefficients.front_c2, 0.0);
    nh_.param<double>(param_path + "/left_c0", coefficients.left_c0, fabs(ymax) );
    nh_.param<double>(param_path + "/left_c1", coefficients.left_c1, 0.0);
    nh_.param<double>(param_path + "/left_c2", coefficients.left_c2, 0.0);
    nh_.param<double>(param_path + "/right_c0", coefficients.right_c0, fabs(ymin) );
    nh_.param<double>(param_path + "/right_c1", coefficients.right_c1, 0.0);
    nh_.param<double>(param_path + "/right_c2", coefficients.right_c2, 0.0);
    nh_.param<double>(param_path + "/rear_c0", coefficients.rear_c0, fabs(xmin) );
    nh_.param<double>(param_path + "/rear_c1", coefficients.rear_c1, 0.0);
    nh_.param<double>(param_path + "/rear_c2", coefficients.rear_c2, 0.0);
  }

  /**
   * @brief Given a footprint polygon, this function finds the minimum and maximum x and y bounds.
   * @param[in] footprint The footprint polygon.
   * @param[out] xmin The lower bound of the footprint along the x axis.
   * @param[out] ymin The lower bound of the footprint along the y axis.
   * @param[out] xmax The upper bound of the footprint along the x axis.
   * @param[out] ymax The upper bound of the footprint along the y axis.
   */
  void findFootprintBounds(Polygon footprint, double& xmin, double& ymin, double& xmax, double& ymax)
  {
    xmin = DBL_MAX; ymin = DBL_MAX;
    xmax = DBL_MIN; ymax = DBL_MIN;

    for (size_t i = 0; i < footprint.size(); ++i)
    {
      xmin = std::min(xmin, footprint[i].x);
      ymin = std::min(ymin, footprint[i].y);
      xmax = std::max(xmax, footprint[i].x);
      ymax = std::max(ymax, footprint[i].y);
    }
  }

private:
  ros::NodeHandle nh_;
  std::string footprint_set_;

  std::map<Footprint::Mode::Index, Polygon> planner_footprints_;
  std::map<Footprint::Mode::Index, Footprint::Coefficients> tracker_footprint_coefficients_;
  std::map<Footprint::Mode::Index, double> footprint_clearing_paddings_;

  double planner_footprint_padding_;
  double tracker_footprint_padding_;
};

/**
 * This class manages a collection of FootprintSet objects.
 */
class FootprintSetCollection
{
public:
  typedef boost::shared_ptr<FootprintSetCollection> Ptr;

public:
  /**
   * Constructor function. Initializes the footprint sets in the collection from the param server.
   * @param[in] private_nh The rosNodeHandle whose namespace is used to retrieve the footprint parameters.
   * @param[in] padding The padding to be applied to padded versions of planner footprints.
   */
  FootprintSetCollection(ros::NodeHandle& private_nh, double padding = 0.0)
  {
    // Load the footprint sets from the parameter server
    XmlRpc::XmlRpcValue footprint_sets_xml;
    if (private_nh.getParam("footprint_sets", footprint_sets_xml) )
    {
      if (footprint_sets_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        for (XmlRpc::XmlRpcValue::iterator it = footprint_sets_xml.begin(); it != footprint_sets_xml.end(); ++it)
        {
          nav_core::FootprintSet::Ptr footprint_set_ptr = nav_core::FootprintSet::Ptr(
            new nav_core::FootprintSet(private_nh, it->first, padding) );
          footprint_set_map_.insert(std::make_pair(it->first, footprint_set_ptr) );
        }
      }
    }
  }

  /**
   * @brief Function used to retrieve the specified FootprintSet from the collection.
   * @param id[in] String id of the footprint set to retrieve.
   * @return Pointer to the footprint set requested. NULL pointer is returned if set does not exist.
   */
  nav_core::FootprintSet::Ptr getSet(std::string id)
  {
    try
    {
      return footprint_set_map_.at(id);
    }
    catch (std::out_of_range&)
    {
      return nav_core::FootprintSet::Ptr();
    }
  }

private:
  std::map<std::string, nav_core::FootprintSet::Ptr> footprint_set_map_;
};

}  // namespace nav_core

#endif  // NAV_CORE_FOOTPRINT_SET_H
