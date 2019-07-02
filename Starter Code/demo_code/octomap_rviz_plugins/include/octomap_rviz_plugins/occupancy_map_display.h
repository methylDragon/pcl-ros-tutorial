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
 *
 * Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */

#ifndef RVIZ_OCCUPANCY_MAP_DISPLAY_H
#define RVIZ_OCCUPANCY_MAP_DISPLAY_H

#ifndef Q_MOC_RUN 

#include <qobject.h>

#include <ros/ros.h>

#include "rviz/default_plugin/map_display.h"

#include <octomap_msgs/Octomap.h>

#include <octomap/OcTreeStamped.h>

#include <message_filters/subscriber.h>

#endif

namespace octomap_rviz_plugin
{

class OccupancyMapDisplay: public rviz::MapDisplay
{
Q_OBJECT
public:
  OccupancyMapDisplay();
  virtual ~OccupancyMapDisplay();

private Q_SLOTS:
  void updateTopic();
  void updateTreeDepth();

protected:
  virtual void onInitialize();
  virtual void subscribe();
  virtual void unsubscribe();

  virtual void handleOctomapBinaryMessage(const octomap_msgs::OctomapConstPtr& msg) = 0;

  boost::shared_ptr<message_filters::Subscriber<octomap_msgs::Octomap> > sub_;

  unsigned int octree_depth_;
  rviz::IntProperty* tree_depth_property_;
};

template <typename OcTreeType>
class TemplatedOccupancyMapDisplay: public OccupancyMapDisplay {
protected:
    void handleOctomapBinaryMessage(const octomap_msgs::OctomapConstPtr& msg);
};

} // namespace rviz

 #endif
