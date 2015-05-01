/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "costmap_2d/layer.h"
#include <stdio.h>

#include <string>     // for string
#include <algorithm>  // for min
#include <vector>     // for vector<>

namespace costmap_2d
{

Layer::Layer()
  : layered_costmap_(NULL)
  , current_(false)
  , enabled_(false)
  , name_()
  , tf_(NULL)
{}

void Layer::initialize(LayeredCostmap* parent, std::string name, tf::TransformListener *tf)
{
  layered_costmap_ = parent;
  name_ = name;
  tf_ = tf;
  onInitialize();
}


const std::vector<geometry_msgs::Point>& Layer::getFootprint() const
{
  return layered_costmap_->getFootprint();
}

void LayerActions::addAction(const AABB& r, Costmap2D* map, LayerActions::Action a, const char *file, unsigned int line)
{
  v_actions.push_back(a);

  // we either don't know the source map or there is no source map
  // for instance, the footprint source is a polygon that gets written
  v_source_maps.push_back(0);
  v_source_rect.push_back(AABB());

  v_destination_maps.push_back(map);
  v_destination_rect.push_back(AABB(r));

  v_action_file.push_back(std::string(file));
  v_action_line.push_back(line);
}

void LayerActions::addAction(const AABB& src_rect, Costmap2D* src_map,
                             const AABB& dst_rect, Costmap2D* dst_map,
                             LayerActions::Action a, const char *file, unsigned int line)
{
  v_actions.push_back(a);

  v_source_maps.push_back(src_map);
  v_source_rect.push_back(src_rect);

  v_destination_maps.push_back(dst_map);
  v_destination_rect.push_back(dst_rect);

  v_action_file.push_back(std::string(file));
  v_action_line.push_back(line);
}

void LayerActions::clear()
{
  v_actions.clear();
  v_source_maps.clear();
  v_source_rect.clear();
  v_destination_maps.clear();
  v_destination_rect.clear();

  v_action_file.clear();
  v_action_file.clear();
}

void LayerActions::copyTo(LayerActions& la_dest)
{
  la_dest.clear();

  const int n = v_actions.size();
  for (int i = 0; i < n; i++)
  {
    appendActionTo(la_dest, i);
  }
}

void LayerActions::appendActionTo(LayerActions &la_dest, int action_index)
{
  la_dest.v_actions.push_back(v_actions[action_index]);
  la_dest.v_source_maps.push_back(v_source_maps[action_index]);
  la_dest.v_source_rect.push_back(v_source_rect[action_index]);
  la_dest.v_destination_maps.push_back(v_destination_maps[action_index]);
  la_dest.v_destination_rect.push_back(v_destination_rect[action_index]);

  la_dest.v_action_file.push_back(v_action_file[action_index]);
  la_dest.v_action_line.push_back(v_action_line[action_index]);
}

void LayerActions::copyToWithMatchingDest(LayerActions &la_dest, Costmap2D *match_pattern)
{
  la_dest.clear();

  const int n = v_actions.size();
  for (int i = 0; i < n; i++)
  {
    if (v_destination_maps[i] == match_pattern)
      appendActionTo(la_dest, i);
  }
}

LayerActions::Action LayerActions::actionAt(int idx)
{
  if (validIndex(idx))
    return v_actions[idx];
  return NONE;
}

bool LayerActions::actionIs(int idx, LayerActions::Action a)
{
  return actionAt(idx) == a;
}

bool LayerActions::validIndex(int idx)
{
  return idx >= 0 && idx < size();
}

Costmap2D* LayerActions::destinationCostmapAt(int idx)
{
  if (validIndex(idx))
    return v_destination_maps[idx];
  return 0;
}

Costmap2D* LayerActions::sourceCostmapAt(int idx)
{
  if (validIndex(idx))
    return v_source_maps[idx];
  return 0;
}

const AABB& LayerActions::sourceAABBAt(int idx)
{
  if (validIndex(idx))
    return v_source_rect[idx];
  return nullAABB;
}

const AABB& LayerActions::destinationAABBAt(int idx)
{
  if (validIndex(idx))
    return v_destination_rect[idx];
  return nullAABB;
}

// looking to merge sets of actions that occur in the same space
void LayerActions::optimize(int tol)
{
  const int n = v_actions.size();

  if (!n)
    return;  // no work to do

  std::vector<bool> got_it;  // if this data is in our new list
  got_it.resize(n, false);

  LayerActions mergedActions;

  // looking to merge changes: MODIFYs, MAXs and OVERWRITEs
  // will not merge TRUEOVERWITES
  for (int i = 0; i < n; i++)
  {
    if (!got_it[i])
    {
      got_it[i] = true;  // don't need to consider this any more
      AABB merged_rect = AABB(v_destination_rect[i]);

      if (v_actions[i] != TRUEOVERWRITE)  // then we can look to merge it
      {
        for (int j = 0; j < n; j++)
        {
          // making sure the actions we are looking to merge operate on the same Costmap2D
          if (v_destination_maps[i] == v_destination_maps[j])
          {
            if (!got_it[j])  // then we can consider this
            {
              const AABB& aabb_j = v_destination_rect[j];
              if (merged_rect.intersect(aabb_j, tol))
              {
                merged_rect.expandBoundingBox(aabb_j);
                got_it[j] = true;  // this is now covered in the merged AABB
              }
            }
          }
        }
      }

      // now we have an updated representation of the rectangle.
      // adding it to the merged list
      mergedActions.addAction(merged_rect, v_destination_maps[i], v_actions[i],
                              v_action_file[i].c_str(), v_action_line[i]);
    }
  }
  // now have a list of updated actions
}



static void _snprintRectCM(char* buf, int n, const AABB& r, Costmap2D* cm)
{
  if (cm)
  {
    // 57 characters:
    snprintf(buf, n, "Cosmap2D(%p) [ (%6d,%6d), (%6d,%6d) ]",
                      reinterpret_cast<void*>(cm), r.min_x, r.min_y, r.max_x, r.max_y);
  }
  else
  {
    snprintf(buf, n, "%57s", "Non-Costmap2D data");
  }
}

void LayerActions::writeToFile(const char *filename)
{
  FILE* f = fopen(filename, "w");
  if (!f)
    return;

  fprintf(f, "LayerActions (%p)\n", reinterpret_cast<void*>(this));
  const int n = v_actions.size();

  char buf[64];

  for (int i = 0; i < n; i++)
  {
    fprintf(f, "(%s:%04i):\n", v_action_file[i].c_str(), v_action_line[i]);

    switch (v_actions[i])
    {
      case OVERWRITE:     fprintf(f, "%13s ", "OVERWRITE"); break;
      case TRUEOVERWRITE: fprintf(f, "%13s ", "TRUEOVERWRITE"); break;
      case MODIFY:        fprintf(f, "%13s ", "MODIFY"); break;
      case MAX:           fprintf(f, "%13s ", "MAX"); break;
    }

    _snprintRectCM(buf, 64, v_source_rect[i], v_source_maps[i]);
    fprintf(f, "%s", buf);
    fprintf(f, " => ");
    _snprintRectCM(buf, 64, v_destination_rect[i], v_destination_maps[i]);
    fprintf(f, "%s", buf);
    fprintf(f, "\n");
  }

  fclose(f);
}

AABB::AABB()
  : min_x(0), max_x(0), min_y(0), max_y(0), initialized_(false)
{
}

AABB::AABB(const AABB &r)
{
  r.copyTo(*this);
}

bool AABB::same(const AABB &r, int tol)
{
  if (!initialized() || !r.initialized())
    return false;
  if (tol)
  {
    if (abs(r.min_x - min_x) > tol) return false;
    if (abs(r.min_y - min_y) > tol) return false;
    if (abs(r.max_x - max_x) > tol) return false;
    if (abs(r.max_y - max_y) > tol) return false;
    return true;
  }
  if (r.min_x != min_x) return false;
  if (r.min_y != min_y) return false;
  if (r.max_x != max_x) return false;
  if (r.max_y != max_y) return false;
  return true;
}

void AABB::copyTo(AABB &dst) const
{
  dst.min_x = min_x;
  dst.min_y = min_y;
  dst.max_x = max_x;
  dst.max_y = max_y;
  dst.setInitialized(initialized());
}

AABB::AABB(int px, int py)
{
  initialized_ = true;
  min_x = px;
  min_y = py;
  max_x = px;
  max_y = py;
}

AABB::AABB(int px1, int py1, int px2, int py2)
{
  initialized_ = true;
  min_x = std::min(px1, px2);
  max_x = std::max(px1, px2);
  min_y = std::min(py1, py2);
  max_y = std::max(py1, py2);
}


void AABB::expandBoundingBox(const AABB &r)
{
  if (initialized_)
  {
    min_x = std::min(min_x, r.min_x);
    max_x = std::max(max_x, r.max_x);
    min_y = std::min(min_y, r.min_y);
    max_y = std::max(max_y, r.max_y);
  }
  else
  {
    r.copyTo(*this);
  }
}

void AABB::expandBoundingBox(int r)
{
  if (initialized_)
  {
    min_x -= r;
    min_y -= r;
    max_x += r;
    max_y += r;
  }
}

bool AABB::inside(int px, int py, int margin) const
{
  if (px < min_x+margin) return false;
  if (px > max_x-margin) return false;
  if (py < min_y+margin) return false;
  if (py > max_y-margin) return false;
  return true;
}

bool AABB::inside(const AABB& bb, int margin) const
{
  return
      inside(bb.min_x, bb.min_y, margin) &&
      inside(bb.max_x, bb.max_y, margin);
}

void AABB::clip(const AABB &bb, AABB &clipped) const
{
  bb.copyTo(clipped);
  clipped.min_x = std::max(min_x, clipped.min_x);
  clipped.min_y = std::max(min_y, clipped.min_y);
  clipped.max_x = std::min(max_x, clipped.max_x);
  clipped.max_y = std::min(max_y, clipped.max_y);

  clipped.clampBounds();
}

void AABB::clampBounds()
{
  if (max_x < min_x)
    max_x = min_x;
  if (max_y < min_y)
    max_y = min_y;
}

double AABB::ratioInside(const AABB &bb) const
{
  AABB clipped;
  clip(bb, clipped);

  return static_cast<double>(area()) / static_cast<double>(clipped.area());
}

int AABB::area() const
{
  if (!initialized_)
    return 0;

  return (max_x - min_x) * (max_y - min_y);
}

int AABB::xn() const
{
  if (!initialized_)
    return 0;
  return max_x - min_x;
}

int AABB::yn() const
{
  if (!initialized_)
    return 0;
  return max_y - min_y;
}

bool AABB::intersect(const AABB &bb, int margin) const
{
  margin *= 2;  // since margin applied to both
  if (bb.max_x + margin < min_x) return false;
  if (bb.min_x - margin > max_x) return false;
  if (bb.max_y + margin < min_y) return false;
  if (bb.min_y - margin > max_y) return false;
  return true;
}

void AABB::mergeIntersecting(std::vector<AABB> &vec_aabb, int margin)
{
  const int n = vec_aabb.size();
  std::vector<AABB> cpy;
  for (int i = 0; i < n; i++)
    cpy.push_back(vec_aabb.at(i));
  vec_aabb.clear();

  std::vector<bool> got_it;  // if this data is in our new list
  got_it.resize(n, false);

  for (int i = 0; i < n; i++)
  {
    if (!got_it[i])
    {
      AABB bb = cpy[i];
      got_it[i] = true;

      for (int j = i+1; j < n; j++)
      {
        if (!got_it[j])
        {
          if (bb.intersect(cpy[j], margin))
          {
            bb.expandBoundingBox(cpy[j]);
            got_it[j] = true;
          }
        }
      }
      vec_aabb.push_back(bb);
    }
  }
}

int AABB::x0() const
{
  if (!initialized_)
    return 0;
  return min_x;
}

int AABB::y0() const
{
  if (!initialized_)
    return 0;
  return min_y;
}

}  // end namespace costmap_2d
