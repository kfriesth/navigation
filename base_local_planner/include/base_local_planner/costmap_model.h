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

#ifndef BASE_LOCAL_PLANNER_COSTMAP_MODEL
#define BASE_LOCAL_PLANNER_COSTMAP_MODEL

#include <base_local_planner/world_model.h>
// For obstacle data access
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <vector>

namespace base_local_planner
{

/**
 * @brief List of checks that are understood by the CostmapModel. FootprintLoop is the default
 *        if none are explicitly supplied to the CostmapModel constructor.
 */
enum FootprintCheckMethod
{
  FootprintLoop,
  FootprintLoopDiamond,
  FootprintLoopDiamondX,
  EveryPixel
};

/**
 * @class CostmapModel
 * @brief A class that implements the WorldModel interface to provide grid
 * based collision checks for the trajectory controller using the costmap.
 */
class CostmapModel : public WorldModel
{
public:
  /**
   * @brief  Constructor for the CostmapModel
   * @param costmap The costmap that should be used
   * @return
   */
  CostmapModel(const costmap_2d::Costmap2D& costmap, FootprintCheckMethod = FootprintLoop);

  /**
   * @brief  Destructor for the world model
   */
  virtual ~CostmapModel() {}
  using WorldModel::footprintCost;

  /**
   * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
   *         using the method defined in the object constructor (FootprintLoop by default)
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @return Positive if all the points lie outside the footprint, negative otherwise
   */
  virtual double footprintCost(const geometry_msgs::Point& position,
                               const std::vector<geometry_msgs::Point>& footprint,
                               double inscribed_radius, double circumscribed_radius);

  /**
   * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
   *         using a specified method
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @param  method Method to use for the foorprint check
   * @return Positive if all the points lie outside the footprint, negative otherwise
   */
  virtual double footprintCost(const geometry_msgs::Point& position,
                               const std::vector<geometry_msgs::Point>& footprint,
                               double inscribed_radius, double circumscribed_radius,
                               FootprintCheckMethod method);

  /**
   * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
   *         using a specified method
   * @param  x X coordinates of base pose
   * @param  y Y coordinates of base pose
   * @param  theta yaw of the base pose
   * @param  footprint_spec The specification of the footprint of the robot in world coordinates
   * @param  method Method to use for the foorprint check
   * @param  inscribed_radius The radius of the inscribed circle of the robot (Default: 0)
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot (Default: 0)
   * @return
   */
  double footprintCost(double x, double y, double theta,
                       const std::vector<geometry_msgs::Point>& footprint_spec,
                       FootprintCheckMethod method, double inscribed_radius = 0.0, double circumscribed_radius=0.0);

  /**
   * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
   *         by looking along the edges of the given footprint at a position for a lethal obstacle.
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @return Positive if all the points lie outside the footprint, negative otherwise
   */
  virtual double footprintCostLoop(const geometry_msgs::Point& position,
                                   const std::vector<geometry_msgs::Point>& footprint,
                                   double inscribed_radius, double circumscribed_radius);

  /**
   * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
   *         by checking every pixel in the footprint at a position for a lethal obstacle
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @return Positive if all the points lie outside the footprint, negative otherwise
   */
  virtual double footprintCostEveryPixel(const geometry_msgs::Point& position,
                                         const std::vector<geometry_msgs::Point>& footprint,
                                         double inscribed_radius, double circumscribed_radius);

  /**
   * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
   *         by checking the footprint perimeter and the centers of adjacent edges joined together
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @return Positive if all the points lie outside the footprint, negative otherwise
   */
  virtual double footprintCostDiamond(const geometry_msgs::Point& position,
                                      const std::vector<geometry_msgs::Point>& footprint,
                                      double inscribed_radius, double circumscribed_radius);

  /**
   * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
   *         by checking the footprint perimeter, the centers of adjacent edges joined together and next nearest
   *         vertices joined together
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @return Positive if all the points lie outside the footprint, negative otherwise
   */
  virtual double footprintCostDiamondX(const geometry_msgs::Point& position,
                                       const std::vector<geometry_msgs::Point>& footprint,
                                       double inscribed_radius, double circumscribed_radius);

  /**
   * @brief  Checks if any obstacles lie along the lines connecting the midpoints of adjacent edges
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @return Positive if all the points lie outside the footprint, negative otherwise
   */
  virtual double diamondCost(const geometry_msgs::Point& position,
                             const std::vector<geometry_msgs::Point>& footprint,
                             double inscribed_radius, double circumscribed_radius);

  /**
   * @brief  Check if an obstacle lies along the perimeter of the footprint
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @return Positive if an obstacle is not found on the perimeter, negative otherwise
   */
  virtual double loopCost(const geometry_msgs::Point& position,
                          const std::vector<geometry_msgs::Point>& footprint,
                          double inscribed_radius, double circumscribed_radius);

  /**
   * @brief  Checks if any obstacles lie along the lines next nearest vertices on the footprint loop
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @return Positive if all the points lie outside the footprint, negative otherwise
   */
  virtual double xCost(const geometry_msgs::Point& position,
                       const std::vector<geometry_msgs::Point>& footprint,
                       double inscribed_radius, double circumscribed_radius);

  /**
   * @brief  Check if the center cost is above the inscribed cost
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @return Positive if the position cost is less than inscribed cost, negative otherwise
   */
  virtual double centerCost(const geometry_msgs::Point& position,
                            const std::vector<geometry_msgs::Point>& footprint,
                            double inscribed_radius, double circumscribed_radius);

  FootprintCheckMethod check_method_;  ///< @brief Which method to use for checks

protected:
  /**
   * @brief Raytrace the polygon perimeter and collect the cells in polygon_cells
   * @param polygon polygon to trace
   * @param polygon_cells set of cells making up the perimeter
   */
  void polygonOutlineCells(const std::vector<costmap_2d::MapLocation>& polygon,
                           std::vector<costmap_2d::MapLocation>& polygon_cells) const;

  /**
   * @brief Collect the points inside a polygon
   * @param polygon polygon containing cells
   * @param polygon_cells cells in polygon
   */
  void convexFillCells(const std::vector<costmap_2d::MapLocation>& polygon,
                       std::vector<costmap_2d::MapLocation>& polygon_cells) const;

  /**
   * @brief Rasterizes a line in the costmap grid and collect coordinates
   * @param x0 The x position of the first cell in grid coordinates
   * @param y0 The y position of the first cell in grid coordinates
   * @param x1 The x position of the second cell in grid coordinates
   * @param y1 The y position of the second cell in grid coordinates
   * @param polygon_cells set of cells making up the line
   * @param max_length maximum number of cells to count (default UINT_MAX)
   */
  void raytraceCells(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                     std::vector<costmap_2d::MapLocation>& polygon_cells,
                     unsigned int max_length = UINT_MAX) const;

  /**
   * @brief A 2D implementation of Bresenham's raytracing algorithm. Collects cells.
   */
  void bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b,
                   int offset_a, int offset_b, unsigned int offset,
                   unsigned int max_length,
                   std::vector<costmap_2d::MapLocation>& polygon_cells) const;

  /**
   * @brief maxCellValue find the maximum cell value in a polygon. Lethal is considered max and will
   *        trigger an early return
   * @param polygon polygon to consider
   * @return maximum cell value inside the polygon
   */
  unsigned char maxCellValue(const std::vector<costmap_2d::MapLocation>& polygon) const;

  /**
   * @brief get sign of input (except zero which is negative in this case)
   * @param x input
   * @return 1 if input is greater than 0, else -1
   */
  inline int sign(int x) const
  {
    return x > 0 ? 1 : -1;
  }

  /**
   * @brief  Rasterizes a line in the costmap grid and checks for collisions
   * @param x0 The x position of the first cell in grid coordinates
   * @param y0 The y position of the first cell in grid coordinates
   * @param x1 The x position of the second cell in grid coordinates
   * @param y1 The y position of the second cell in grid coordinates
   * @return A positive cost for a legal line... negative otherwise
   */
  double lineCost(int x0, int x1, int y0, int y1);

  /**
   * @brief  Rasterizes a line in the costmap grid and checks for collisions after conversion to map coordinates
   * @param x0 The x position of the first cell in grid coordinates
   * @param y0 The y position of the first cell in grid coordinates
   * @param x1 The x position of the second cell in grid coordinates
   * @param y1 The y position of the second cell in grid coordinates
   * @return A positive cost for a legal line... negative otherwise
   */
  double realLineCost(double x0, double x1, double y0, double y1);

  /**
   * @brief  Checks the cost of a point in the costmap
   * @param x The x position of the point in cell coordinates
   * @param y The y position of the point in cell coordinates
   * @return A positive cost for a legal point... negative otherwise
   */
  double pointCost(int x, int y);

  /**
   * @brief Checks the cost of a point in the costmap after converting from global coordinates
   * @param x The x position of the point in cell coordinates
   * @param y The y position of the point in cell coordinates
   * @return A positive cost for a legal point... negative otherwise
   */
  double realPointCost(double x, double y);

  const costmap_2d::Costmap2D& costmap_;  ///< @brief Allows access of costmap obstacle information
};

}  // namespace base_local_planner
#endif  // BASE_LOCAL_PLANNER_COSTMAP_MODEL
