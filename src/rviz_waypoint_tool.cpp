#include "rviz_waypoint_tool.h"
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/string_property.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>

namespace ros_simple_waypoints
{
RvizWaypointTool::RvizWaypointTool() : moving_waypoint_node_(NULL), current_waypoint_property_(NULL)
{
  shortcut_key_ = 'w';
}

RvizWaypointTool::~RvizWaypointTool()
{
  for (unsigned i = 0; i < waypoint_nodes_.size(); i++)
  {
    scene_manager_->destroySceneNode(flag_nodes_[i]);
  }
}
}