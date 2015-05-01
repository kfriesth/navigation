/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 * Author: David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_COSTMAP_2D_LAYER_H_
#define COSTMAP_2D_COSTMAP_2D_LAYER_H_

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include <geometry_msgs/Point32.h>

namespace costmap_2d
{
class LayeredCostmap;
class Layer;

/**
 * @class AABB
 * @brief Axis aligned bounding box
 */
class AABB
{
public:
  /**
   * @brief Create an empty Axis Aligned Bounding Box with both points at the origin
   */
  AABB();

  /**
   * @brief Copy constructor
   */
  AABB(const AABB& r);

  /**
   * @brief Create an empty Axis Aligned Bounding Box with both points at the given location
   */
  AABB(int px, int py);

  /**
   * @brief Create an Axis Aligned Bounding Box that bounds the given points
   */
  AABB(int px1, int py1, int px2, int py2);

  /**
   * @brief Create an Axis Aligned Bounding Box about given points
   */
  AABB(const std::vector<geometry_msgs::Point>& pts);

  /**
   * @brief Determine if this AABB is the same as another with possible tolerance
   * @param r AABB to compare against
   * @param tol Tolerance of compairson (default 0)
   */
  bool same(const AABB& r, int tol=0);

  void copyTo(AABB& dst) const;

  void expandBoundingBox(const AABB& bb);

  /**
   * @brief Expand the bounding box by a constant value
   * @param r expansion margin
   */
  void expandBoundingBox(int r);

  bool inside(int px, int py, int margin = 0) const;
  bool inside(const AABB& bb, int margin = 0) const;

  /**
   * @brief Clip the passed AABB against the calling AABB
   * @param bb AABB to be clipped
   * @param clipped destination AABB representing the clipped bounding box
   */
  void clip(const AABB& bb, AABB& clipped) const;

  /**
   * @brief Make sure max coordinates are not smaller than min coordinates.
   */
  void clampBounds();

  /**
   * @brief Compute how much of the given AABB is inside the calling AABB as a ratio of areas
   * @param bb The AABB to use as a test
   */
  double ratioInside(const AABB& bb) const;

  int area() const;

  int xn() const;
  int yn() const;

  int x0() const;
  int y0() const;

  /**
   * @brief Determine if this AABB intersects another AABB
   * @param bb AABB to compare against
   * @param margin extra margin applied to the AABBs. A positive margin will make side by side AABBs appear to intersect. Default 0.
   */
  bool intersect(const AABB& bb, int margin = 0) const;

  bool initialized() const {return initialized_;}
  void setInitialized(bool b) {initialized_ = b;}


  /**
   * @brief Static method that will take a vector of bounding boxes and modify it so that overlapping boxes are merged
   * @param vec_aabb the vector of boxes that will be merged. This vector is modified to contain the solution
   * @param margin extra margin applied to the AABBs. A positive margin will make side by side AABBs appear to intersect. Default 0.
   */
  static void mergeIntersecting(std::vector<AABB>& vec_aabb, int margin=0);

  int min_x, min_y;
  int max_x, max_y;
  bool initialized_;
};

/**
 * @class LayerActions
 * @brief Auxiliary storage object to track actions taken by the layer plugins to help with optimization.
 */
class LayerActions
{
public:
  LayerActions() {}

  enum Action{
    NONE,
    OVERWRITE,
    TRUEOVERWRITE,
    MODIFY,
    MAX
  };

  // define an action in terms of the destination map
  void addAction(const AABB& r, Costmap2D *map, LayerActions::Action a, const char* file, unsigned int line);

  // define an action from one map to another
  void addAction(const AABB& src_rect, Costmap2D *src_map, const AABB& dst_rect, Costmap2D *dst_map, LayerActions::Action a, const char* file, unsigned int line);
  void clear();

  void copyTo(LayerActions& la_dest);
  void appendActionTo(LayerActions& la_dest, int action_index);

  void copyToWithMatchingDest(LayerActions& la_dest, Costmap2D* match_pattern);

  int size() {return (int)v_actions.size();}

  LayerActions::Action actionAt(int idx);
  bool actionIs(int idx, Action a);
  bool validIndex(int idx);

  Costmap2D* destinationCostmapAt(int idx);
  Costmap2D* sourceCostmapAt(int idx);

  const AABB& sourceAABBAt(int idx);
  const AABB& destinationAABBAt(int idx);


  /**
   * @brief Look at the sequence of actions and determine if some can be merged
   * @param tolerance Some rectanges may not quite overlap but if they are within the given tolerance then we will merge them
   */
  void optimize(int tolerance=0);

  void writeToFile(const char* filename);

  // These define the actions that we have collected
  // we have the source and destination rectangle with the corresponding maps
  // and the action type
  std::vector<AABB>                 v_source_rect;
  std::vector<AABB>                 v_destination_rect;
  std::vector<Costmap2D*>           v_source_maps;
  std::vector<Costmap2D*>           v_destination_maps;
  std::vector<LayerActions::Action> v_actions;

  std::vector<std::string> v_action_file;
  std::vector<int>         v_action_line;
private:
  AABB nullAABB;
};

class Layer
{
public:
  Layer();

  void initialize( LayeredCostmap* parent, std::string name, tf::TransformListener *tf );

  /**
   * @brief This is called by the LayeredCostmap to poll this plugin as to how
   *        much of the costmap it needs to update. Each layer can increase
   *        the size of this bounds.
   *
   * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
   * by Lu et. Al, IROS 2014.
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y) {}

  /**
   * @brief Actually update the underlying costmap, only within the bounds
   *        calculated during UpdateBounds().
   */
  virtual void updateCosts(LayerActions* layer_actions, Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {}

  /** @brief Stop publishers. */
  virtual void deactivate() {}

  /** @brief Restart publishers if they've been stopped. */
  virtual void activate() {}

  virtual void reset() {}

  virtual ~Layer() {}

  /**
   * @brief Check to make sure all the data in the layer is up to date.
   *        If the layer is not up to date, then it may be unsafe to
   *        plan using the data from this layer, and the planner may
   *        need to know.
   *
   *        A layer's current state should be managed by the protected
   *        variable current_.
   * @return Whether the data in the layer is up to date.
   */
  bool isCurrent() const
  {
    return current_;
  }

  /** @brief Implement this to make this layer match the size of the parent costmap. */
  virtual void matchSize() {}

  std::string getName() const
  {
    return name_;
  }

  /** @brief Convenience function for layered_costmap_->getFootprint(). */
  const std::vector<geometry_msgs::Point>& getFootprint() const;

  /** @brief LayeredCostmap calls this whenever the footprint there
   * changes (via LayeredCostmap::setFootprint()).  Override to be
   * notified of changes to the robot's footprint. */
  virtual void onFootprintChanged() {}

protected:
  /** @brief This is called at the end of initialize().  Override to
   * implement subclass-specific initialization.
   *
   * tf_, name_, and layered_costmap_ will all be set already when this is called. */
  virtual void onInitialize() {}

  LayeredCostmap* layered_costmap_;
  bool current_;
  bool enabled_; ///< Currently this var is managed by subclasses.  TODO: make this managed by this class and/or container class.
  std::string name_;
  tf::TransformListener* tf_;

private:
  std::vector<geometry_msgs::Point> footprint_spec_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_LAYER_H_
