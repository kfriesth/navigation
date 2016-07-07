#ifndef NAV_CORE_FOOTPRINT_SET_H
#define NAV_CORE_FOOTPRINT_SET_H

#include <ros/ros.h>
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Point.h>
#include <vector>
#include <string>
#include <costmap_2d/footprint.h>

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
  double side_c0, side_c1, side_c2;
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
   * @param[in] index The Footprint::Mode enum corresponding to which footprint to return.
   * @param[out] footprint The requested footprint polygon.
   * @return True on success. False on error.
   */ 
  bool getPlannerFootprint(int index, Polygon& footprint)
  {
    Footprint::Mode::Index idx_mode = static_cast<Footprint::Mode::Index>(index);

    try
    {
      footprint = planner_footprints_.at(idx_mode);
    }
    catch (std::out_of_range&)
    {
      ROS_ERROR("FootprintSet %s: Requested planner footprint with mode id '%d' does not exist.",
                footprint_set_.c_str(), index);
      return false;
    }

    return true;
  }

  /**
   * @brief Function used to set planner footprints. This overrides default initializations from the param server.
   * @param[in] index The footprint::Mode enum corresponding to which footprint to set.
   * @param[in] footprint The new footprint to use.
   */
  void setPlannerFootprint(int index, Polygon& footprint)
  {
    Footprint::Mode::Index idx_mode = static_cast<Footprint::Mode::Index>(index);
    
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
    std::string param_path_prefix = "footprint_sets/" + footprint_set_ + "/tracker";
    readTrackerFootprintCoefficients(param_path_prefix + Footprint::Param::NORMAL, coefficients);
    tracker_footprint_coefficients_[Footprint::Mode::NORMAL] = coefficients;
    readTrackerFootprintCoefficients(param_path_prefix + Footprint::Param::NARROW, coefficients);
    tracker_footprint_coefficients_[Footprint::Mode::NARROW] = coefficients;
    readTrackerFootprintCoefficients(param_path_prefix + Footprint::Param::DOCK, coefficients);
    tracker_footprint_coefficients_[Footprint::Mode::DOCK] = coefficients;
  }

private:
  /**
   * @brief Function used to read in the coefficients of the dynamic footprint used by the tracker.
   * @param[in] param_path The relatively parameter path prefix where the coefficients are stored on the param server.
   * @param[out] coefficients The struct containing the footprint coefficients.
   */
  void readTrackerFootprintCoefficients(std::string param_path, Footprint::Coefficients& coefficients)
  {
    nh_.param<double>(param_path + "/front_c0", coefficients.front_c0, 0.0);
    nh_.param<double>(param_path + "/front_c1", coefficients.front_c1, 0.0);
    nh_.param<double>(param_path + "/front_c2", coefficients.front_c2, 0.0);
    nh_.param<double>(param_path + "/side_c0", coefficients.side_c0, 0.0);
    nh_.param<double>(param_path + "/side_c1", coefficients.side_c1, 0.0);
    nh_.param<double>(param_path + "/side_c2", coefficients.side_c2, 0.0);
    nh_.param<double>(param_path + "/rear_c0", coefficients.rear_c0, 0.0);
    nh_.param<double>(param_path + "/rear_c1", coefficients.rear_c1, 0.0);
    nh_.param<double>(param_path + "/rear_c2", coefficients.rear_c2, 0.0);
  } 
 
private:
  ros::NodeHandle nh_;
  std::string footprint_set_;

  boost::unordered_map<Footprint::Mode::Index, Polygon> planner_footprints_;
  boost::unordered_map<Footprint::Mode::Index, Footprint::Coefficients> tracker_footprint_coefficients_;

  double planner_footprint_padding_;
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
  boost::unordered_map<std::string, nav_core::FootprintSet::Ptr> footprint_set_map_;
};

}  // namespace nav_core

#endif  // NAV_CORE_FOOTPRINT_SET_H
