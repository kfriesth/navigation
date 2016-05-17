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

#include <base_local_planner/line_iterator.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <vector>
#include <algorithm>

namespace base_local_planner
{

CostmapModel::CostmapModel(const costmap_2d::Costmap2D& ma, base_local_planner::FootprintCheckMethod method)
  : costmap_(ma), check_method_(method)
{
}

// The main method that uses the supplied check_method in the object constructor
double CostmapModel::footprintCost(const geometry_msgs::Point& position,
                                   const std::vector<geometry_msgs::Point>& footprint,
                                   double inscribed_radius,
                                   double circumscribed_radius)
{
  switch (check_method_)
  {
  case FootprintLoop:
  {
    return footprintCostLoop(position, footprint, inscribed_radius, circumscribed_radius);
  }

  case FootprintLoopDiamond:
  {
    return footprintCostDiamond(position, footprint, inscribed_radius, circumscribed_radius);
  }

  case FootprintLoopDiamondX:
  {
    return footprintCostDiamondX(position, footprint, inscribed_radius, circumscribed_radius);
  }

  case EveryPixel:
  {
    return footprintCostEveryPixel(position, footprint, inscribed_radius, circumscribed_radius);
  }
  }

  // If we don't understand the method then we will return an obstacle
  return -1.0;
}

void CostmapModel::polygonOutlineCells(const std::vector<costmap_2d::MapLocation>& polygon,
                                       std::vector<costmap_2d::MapLocation>& polygon_cells) const
{
  for (unsigned int i = 0; i < polygon.size() - 1; ++i)
  {
    raytraceCells(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
  }

  if (!polygon.empty())
  {
    unsigned int last_index = polygon.size() - 1;
    // we also need to close the polygon by going from the last point to the first
    raytraceCells(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
  }
}

// TODO(jmercer): The following is copied from costmap_2d for robustness
// but it iterates through the cells column by column which is against the
// direction of memory. The following code should be changed to iterate along
// the X direction.
void CostmapModel::convexFillCells(const std::vector<costmap_2d::MapLocation>& polygon,
                                   std::vector<costmap_2d::MapLocation>& polygon_cells) const
{
  // The following is copied from costmap_2d with style changes and explicit adds to polygon_cells
  // rather than using a custom action.

  // we need a minimum polygon of a triangle
  if (polygon.size() < 3)
  {
    return;
  }

  // first get the cells that make up the outline of the polygon
  polygonOutlineCells(polygon, polygon_cells);

  // quick bubble sort to sort points by x
  costmap_2d::MapLocation swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1)
  {
    if (polygon_cells[i].x > polygon_cells[i + 1].x)
    {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;

      if (i > 0)
      {
        --i;
      }
    }
    else
    {
      ++i;
    }
  }

  i = 0;
  costmap_2d::MapLocation min_pt;
  costmap_2d::MapLocation max_pt;
  unsigned int min_x = polygon_cells[0].x;
  unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

  // walk through each column and mark cells inside the polygon
  for (unsigned int x = min_x; x <= max_x; ++x)
  {
    if (i >= polygon_cells.size() - 1)
    {
      break;
    }

    if (polygon_cells[i].y < polygon_cells[i + 1].y)
    {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    }
    else
    {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    i += 2;
    while (i < polygon_cells.size() && polygon_cells[i].x == x)
    {
      if (polygon_cells[i].y < min_pt.y)
      {
        min_pt = polygon_cells[i];
      }
      else if (polygon_cells[i].y > max_pt.y)
      {
        max_pt = polygon_cells[i];
      }
      ++i;
    }

    costmap_2d::MapLocation pt;
    // loop though cells in the column
    for (unsigned int y = min_pt.y; y < max_pt.y; ++y)
    {
      pt.x = x;
      pt.y = y;
      polygon_cells.push_back(pt);
    }
  }
}

void CostmapModel::raytraceCells(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
                                 std::vector<costmap_2d::MapLocation>& polygon_cells, unsigned int max_length) const
{
  int dx = x1 - x0;
  int dy = y1 - y0;

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);

  int offset_dx = sign(dx);
  int offset_dy = sign(dy) * costmap_.getSizeInCellsX();

  unsigned int offset = y0 * costmap_.getSizeInCellsX() + x0;

  // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
  double dist = hypot(dx, dy);
  double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

  // if x is dominant
  if (abs_dx >= abs_dy)
  {
    int error_y = abs_dx / 2;
    bresenham2D(abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx), polygon_cells);
    return;
  }

  // otherwise y is dominant
  int error_x = abs_dy / 2;
  bresenham2D(abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy), polygon_cells);
}

unsigned char CostmapModel::maxCellValue(const std::vector<costmap_2d::MapLocation>& polygon_cells) const
{
  unsigned char max_cell = costmap_2d::FREE_SPACE;

  for (size_t k = 0; k < polygon_cells.size(); k++)
  {
    const costmap_2d::MapLocation& m = polygon_cells[k];
    const unsigned char val_k = costmap_.getCost(m.x, m.y);

    if (val_k == costmap_2d::LETHAL_OBSTACLE)
    {
      return costmap_2d::LETHAL_OBSTACLE;
    }

    if (val_k < costmap_2d::LETHAL_OBSTACLE)
    {
      max_cell = std::max<unsigned char>(max_cell, val_k);
    }
  }

  return max_cell;
}

void CostmapModel::bresenham2D(unsigned int abs_da, unsigned int abs_db, int error_b,
                               int offset_a, int offset_b, unsigned int offset, unsigned int max_length,
                               std::vector<costmap_2d::MapLocation>& polygon_cells) const
{
  costmap_2d::MapLocation m;

  unsigned int end = std::min(max_length, abs_da);
  for (unsigned int i = 0; i < end; ++i)
  {
    costmap_.indexToCells(offset, m.x, m.y);
    polygon_cells.push_back(m);

    offset += offset_a;
    error_b += abs_db;
    if ((unsigned int)error_b >= abs_da)
    {
      offset += offset_b;
      error_b -= abs_da;
    }
  }

  costmap_.indexToCells(offset, m.x, m.y);
  polygon_cells.push_back(m);
}

double CostmapModel::footprintCostLoop(const geometry_msgs::Point& position,
                                       const std::vector<geometry_msgs::Point>& footprint,
                                       double inscribed_radius, double circumscribed_radius)
{
  // check for inscribed at center
  const double cost = centerCost(position, footprint, inscribed_radius, circumscribed_radius);
  if (cost < 0)
  {
    return -1.0;
  }

  const double loop_cost = loopCost(position, footprint, inscribed_radius, circumscribed_radius);
  if (loop_cost < 0)
  {
    return -1.0;
  }

  return std::max<double>(cost, loop_cost);
}

double CostmapModel::footprintCostDiamond(const geometry_msgs::Point& position,
                                          const std::vector<geometry_msgs::Point>& footprint,
                                          double inscribed_radius, double circumscribed_radius)
{
  const double loop_cost = footprintCostLoop(position, footprint, inscribed_radius, circumscribed_radius);

  if (loop_cost < 0)
  {
    return -1.0;
  }

  const double diamond_cost = diamondCost(position, footprint, inscribed_radius, circumscribed_radius);

  if (diamond_cost < 0)
  {
    return -1.0;
  }

  return std::max<double>(loop_cost, diamond_cost);
}

double CostmapModel::footprintCostDiamondX(const geometry_msgs::Point& position,
                                           const std::vector<geometry_msgs::Point>& footprint,
                                           double inscribed_radius, double circumscribed_radius)
{
  const double footprint_diamond = footprintCostDiamond(position, footprint, inscribed_radius, circumscribed_radius);

  if (footprint_diamond < 0)
  {
    return -1.0;
  }

  const double x_cost = xCost(position, footprint, inscribed_radius, circumscribed_radius);

  if (x_cost < 0)
  {
    return -1.0;
  }

  return std::max<double>(x_cost, footprint_diamond);
}

double CostmapModel::footprintCostEveryPixel(const geometry_msgs::Point& position,
                                             const std::vector<geometry_msgs::Point>& footprint,
                                             double inscribed_radius,
                                             double circumscribed_radius)
{
  // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
  std::vector<costmap_2d::MapLocation> map_polygon;
  for (unsigned int i = 0; i < footprint.size(); ++i)
  {
    costmap_2d::MapLocation loc;
    if (!costmap_.worldToMap(footprint[i].x, footprint[i].y, loc.x, loc.y))
    {
      // footprint lies outside map bounds
      return costmap_2d::LETHAL_OBSTACLE;
    }
    map_polygon.push_back(loc);
  }

  std::vector<costmap_2d::MapLocation> polygon_cells;

  // get the cells that fill the polygon
  convexFillCells(map_polygon, polygon_cells);

  unsigned char max_cell = maxCellValue(polygon_cells);

  if (max_cell == costmap_2d::LETHAL_OBSTACLE)
  {
    return -1.0;
  }
  return max_cell;
}

double CostmapModel::loopCost(const geometry_msgs::Point& position,
                              const std::vector<geometry_msgs::Point>& footprint,
                              double inscribed_radius, double circumscribed_radius)
{
  const size_t n = footprint.size();
  double cost = costmap_2d::FREE_SPACE;

  // check cost of lines made from adjacent vertices
  for (size_t i = 0; i < n; i++)
  {
    size_t j = (i + 1) % n;

    const double line_cost = realLineCost(footprint[i].x, footprint[j].x, footprint[i].y, footprint[j].y);

    if (line_cost < 0)
    {
      return -1.0;
    }

    cost = std::max(line_cost, cost);
  }

  return cost;
}

double CostmapModel::diamondCost(const geometry_msgs::Point& position,
                                 const std::vector<geometry_msgs::Point>& footprint,
                                 double inscribed_radius, double circumscribed_radius)
{
  const size_t n = footprint.size();
  if (n == 0)
  {
    return realPointCost(position.x, position.y);
  }

  if (n == 1)
  {
    return realPointCost(footprint[0].x, footprint[0].y);
  }

  double cost = costmap_2d::FREE_SPACE;
  // join the midpoints of adjacent edges and check their lines
  for (size_t i = 0; i < footprint.size(); i ++)
  {
    size_t j = (i+1) % footprint.size();
    size_t k = (i+2) % footprint.size();

    double x0 = 0.5 * (footprint[i].x + footprint[j].x);
    double x1 = 0.5 * (footprint[j].x + footprint[k].x);
    double y0 = 0.5 * (footprint[i].y + footprint[j].y);
    double y1 = 0.5 * (footprint[j].y + footprint[k].y);

    const double line_cost = realLineCost(x0, x1, y0, y1);

    if (line_cost < 0)
    {
      return -1;
    }

    cost = std::max<double>(cost, line_cost);
  }

  return cost;
}

double CostmapModel::xCost(const geometry_msgs::Point& position,
                           const std::vector<geometry_msgs::Point>& footprint,
                           double inscribed_radius, double circumscribed_radius)
{
  const size_t n = footprint.size();
  if (n == 0)
  {
    return realPointCost(position.x, position.y);
  }

  if (n == 1)
  {
    return realPointCost(footprint[0].x, footprint[0].y);
  }

  if (n == 2)
  {
    return realLineCost(footprint[0].x, footprint[1].x, footprint[0].y, footprint[1].y);
  }

  double cost = costmap_2d::FREE_SPACE;
  // Join the verts of next nearest vertices on the loop.
  // This will turn a rectangle into an X
  //                a pentagon into a 5 pointed star
  //                a hexagon into 2 overlapping triangles, etc
  for (size_t i = 0; i < n - 2; i ++)
  {
    size_t j = (i + 2) % n;

    const double line_cost = realLineCost(footprint[i].x, footprint[j].x, footprint[i].y, footprint[j].y);

    if (line_cost < 0)
    {
      return -1;
    }

    cost = std::max<double>(cost, line_cost);
  }

  return cost;
}

// calculate the cost of a ray-traced line
double CostmapModel::lineCost(int x0, int x1, int y0, int y1)
{
  double line_cost = costmap_2d::FREE_SPACE;

  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance())
  {
    const double point_cost = pointCost(line.getX(), line.getY());

    if (point_cost < 0)
    {
      return -1;
    }

    line_cost = std::max<double>(line_cost, point_cost);
  }

  return line_cost;
}

double CostmapModel::pointCost(int x, int y)
{
  unsigned char cost = costmap_.getCost(x, y);

  if (cost == costmap_2d::LETHAL_OBSTACLE)
  {
    return -1.0;
  }

  return cost;
}

double CostmapModel::realLineCost(double x0, double x1, double y0, double y1)
{
  unsigned int cx0, cy0;
  unsigned int cx1, cy1;

  if (!costmap_.worldToMap(x0, y0, cx0, cy0))
  {
    return -1;
  }

  if (!costmap_.worldToMap(x1, y1, cx1, cy1))
  {
    return -1;
  }

  return lineCost(cx0, cx1, cy0, cy1);
}

double CostmapModel::realPointCost(double x, double y)
{
  // used to put things into grid coordinates
  unsigned int cell_x, cell_y;

  // get the cell coord of the center point of the robot
  if (!costmap_.worldToMap(x, y, cell_x, cell_y))
  {
    return -1;
  }

  unsigned char cost = costmap_.getCost(cell_x, cell_y);

  if (cost == costmap_2d::LETHAL_OBSTACLE)
  {
    return -1;
  }
  return cost;
}

double CostmapModel::centerCost(const geometry_msgs::Point& position,
                                const std::vector<geometry_msgs::Point>& footprint,
                                double inscribed_radius, double circumscribed_radius)
{
  // used to put things into grid coordinates
  unsigned int cell_x, cell_y;

  // get the cell coord of the center point of the robot
  if (!costmap_.worldToMap(position.x, position.y, cell_x, cell_y))
  {
    return -1;
  }

  unsigned char cost = costmap_.getCost(cell_x, cell_y);

  if ((cost == costmap_2d::LETHAL_OBSTACLE) |
      (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
  {
    return -1;
  }

  return cost;
}

}  // namespace base_local_planner
