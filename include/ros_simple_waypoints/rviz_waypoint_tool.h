#ifndef RVIZ_WAYPOINT_TOOL_H
#define RVIZ_WAYPOINT_TOOL_H

#include <ros/ros.h>
#include <rviz/tool.h>

namespace Ogre
{
class SceneNode;
class Vector3;
}

namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class StringProperty;
class Arrow;
}

namespace ros_simple_waypoints
{
class RvizWaypointTool : public rviz::Tool
{
Q_OBJECT

private Q_SLOTS:
  void updateTopic();

private:
  std::vector<Ogre::SceneNode*> waypoint_nodes_;
  Ogre::SceneNode* moving_waypoint_node_;
  std::string waypoint_resource_;
  rviz::VectorProperty* current_waypoint_property_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  rviz::StringProperty* topic_property_;

public:
  RvizWaypointTool();
  ~RvizWaypointTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config& config) const;

protected:
  void onPoseSet(double x, double y, double theta);
};
} // end namespace ros_simple_waypoints

#endif // RVIZ_WAYPOINT_TOOL_H