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

#ifndef RVIZ_OCCUPANCY_GRID_DISPLAY_H
#define RVIZ_OCCUPANCY_GRID_DISPLAY_H

#ifndef Q_MOC_RUN 
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>

#include <octomap_msgs/Octomap.h>

#include <octomap/OcTreeStamped.h>
#include <octomap/ColorOcTree.h>

#include <rviz/display.h>
#include "rviz/ogre_helpers/point_cloud.h"

#endif

namespace rviz {
class RosTopicProperty;
class IntProperty;
class EnumProperty;
class FloatProperty;
}

namespace octomap_rviz_plugin
{

class OccupancyGridDisplay : public rviz::Display
{
Q_OBJECT
public:
  OccupancyGridDisplay();
  virtual ~OccupancyGridDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

private Q_SLOTS:
  void updateQueueSize();
  void updateTopic();
  void updateTreeDepth();
  void updateOctreeRenderMode();
  void updateOctreeColorMode();
  void updateAlpha();
  void updateMaxHeight();
  void updateMinHeight();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  void subscribe();
  void unsubscribe();

  virtual void incomingMessageCallback(const octomap_msgs::OctomapConstPtr& msg) = 0;

  void setColor( double z_pos, double min_z, double max_z, double color_factor, rviz::PointCloud::Point& point);

  void clear();

  virtual bool updateFromTF();

  typedef std::vector<rviz::PointCloud::Point> VPoint;
  typedef std::vector<VPoint> VVPoint;

  boost::shared_ptr<message_filters::Subscriber<octomap_msgs::Octomap> > sub_;

  boost::mutex mutex_;

  // point buffer
  VVPoint new_points_;
  VVPoint point_buf_;
  bool new_points_received_;

  // Ogre-rviz point clouds
  std::vector<rviz::PointCloud*> cloud_;
  std::vector<double> box_size_;
  std_msgs::Header header_;

  // Plugin properties
  rviz::IntProperty* queue_size_property_;
  rviz::RosTopicProperty* octomap_topic_property_;
  rviz::EnumProperty* octree_render_property_;
  rviz::EnumProperty* octree_coloring_property_;
  rviz::IntProperty* tree_depth_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* max_height_property_;
  rviz::FloatProperty* min_height_property_;

  u_int32_t queue_size_;
  uint32_t messages_received_;
  double color_factor_;
};

template <typename OcTreeType>
class TemplatedOccupancyGridDisplay: public OccupancyGridDisplay {
protected:
  void incomingMessageCallback(const octomap_msgs::OctomapConstPtr& msg);
  void setVoxelColor(rviz::PointCloud::Point& newPoint, typename OcTreeType::NodeType& node, double minZ, double maxZ);
  ///Returns false, if the type_id (of the message) does not correspond to the template paramter
  ///of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);
};

} // namespace octomap_rviz_plugin

#endif //RVIZ_OCCUPANCY_GRID_DISPLAY_H
