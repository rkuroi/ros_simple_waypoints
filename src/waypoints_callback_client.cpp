#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros_simple_waypoints/CancelCurrentWaypoint.h>
#include <ros_simple_waypoints/ClearWaypointsSequence.h>
#include <ros_simple_waypoints/LoadWaypointsSequence.h>
#include <ros_simple_waypoints/SaveWaypointsSequence.h>
#include <ros_simple_waypoints/StartWaypointsSequence.h>
#include <ros_simple_waypoints/PublishWaypointsSequence.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// helper function to output a goal to YAML format 
YAML::Emitter& operator << (YAML::Emitter& out, const move_base_msgs::MoveBaseGoal& goal)
{
	out << YAML::BeginMap;

  out << YAML::Key << "header" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "seq" << YAML::Value << goal.target_pose.header.seq;
  out << YAML::Key << "time" << YAML::Value << 0; // Time is irrelevant for all save purposes
  out << YAML::Key << "frame_id" << YAML::Value << goal.target_pose.header.frame_id << YAML::EndMap; // end header map

  out << YAML::Key << "pose" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "x" << YAML::Value << goal.target_pose.pose.position.x;
  out << YAML::Key << "y" << YAML::Value << goal.target_pose.pose.position.y;
  out << YAML::Key << "z" << YAML::Value << goal.target_pose.pose.position.z << YAML::EndMap; // end position map
  out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "x" << YAML::Value << goal.target_pose.pose.orientation.x;
  out << YAML::Key << "y" << YAML::Value << goal.target_pose.pose.orientation.y;
  out << YAML::Key << "z" << YAML::Value << goal.target_pose.pose.orientation.z;
  out << YAML::Key << "w" << YAML::Value << goal.target_pose.pose.orientation.w << YAML::EndMap; // end orientation map
  out << YAML::EndMap; // end pose map

  out << YAML::EndMap; // end global mamp
	return out;
}

// helper function to output a sequence of goals to YAML format 
YAML::Emitter& operator << (YAML::Emitter& out, const std::list<move_base_msgs::MoveBaseGoal>& wp_list)
{
  out << YAML::BeginMap;
  out << YAML::Key << "waypoints_sequence" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "nb_waypoints" << YAML::Value << wp_list.size();
  out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
	for (auto it = wp_list.begin(); it != wp_list.end(); ++it)
  {
    out << *it;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap; // end waypoints_sequence map
  out << YAML::EndMap; // end global map
	return out;
}

// helper function to get a goal from YAML format 
move_base_msgs::MoveBaseGoal loadWaypointFromYAML(const YAML::Node& wp_node)
{
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.seq = wp_node["header"]["seq"].as<unsigned int>();
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = wp_node["header"]["frame_id"].as<std::string>();

  goal.target_pose.pose.position.x = wp_node["pose"]["position"]["x"].as<double>();
  goal.target_pose.pose.position.y = wp_node["pose"]["position"]["y"].as<double>();
  goal.target_pose.pose.position.z = wp_node["pose"]["position"]["z"].as<double>();
  goal.target_pose.pose.orientation.x = wp_node["pose"]["orientation"]["x"].as<double>();
  goal.target_pose.pose.orientation.y = wp_node["pose"]["orientation"]["y"].as<double>();
  goal.target_pose.pose.orientation.z = wp_node["pose"]["orientation"]["z"].as<double>();
  goal.target_pose.pose.orientation.w = wp_node["pose"]["orientation"]["w"].as<double>();

  return goal;
}

// helper function to get a sequence of goals from YAML format 
std::list<move_base_msgs::MoveBaseGoal> loadWaypointsListFromYAML(const YAML::Node& wp_list_node)
{
  std::list<move_base_msgs::MoveBaseGoal> wp_list;
  for (size_t i = 0; i < wp_list_node.size(); i++)
  {
    wp_list.push_back(loadWaypointFromYAML(wp_list_node[i]));
  }
  return wp_list;
}

class WaypointsCallbackNode
{
private:
  std::list<move_base_msgs::MoveBaseGoal> waypoints_list_;
  std::list<move_base_msgs::MoveBaseGoal>::iterator current_waypoints_it_;
  std::list<move_base_msgs::MoveBaseGoal>::iterator last_waypoint_it_;
  unsigned int current_waypoint_index_;
  MoveBaseClient mbc_;
  ros::Subscriber waypoints_definition_sub_;
  ros::Publisher waypoints_export_pub_;
  ros::ServiceServer cancel_current_waypoint_server_;
  ros::ServiceServer clear_waypoints_sequence_server_;
  ros::ServiceServer load_waypoints_sequence_server_;
  ros::ServiceServer save_waypoints_sequence_server_;
  ros::ServiceServer start_waypoints_sequence_server_;
  ros::ServiceServer publish_waypoints_sequence_server_;
public:
  WaypointsCallbackNode() : mbc_("move_base", true)
  {
    ros::NodeHandle nh;

    //wait for the action server to come up
    while(!mbc_.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Server came up. Ready to send goals");

    // subscribe to a topic on which the goals are published by another node (i.e. rviz)
    waypoints_definition_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("simple_waypoints_definition", 1, &WaypointsCallbackNode::addWaypoint, this);

    // publish the list of waypoints for visualization
    waypoints_export_pub_ = nh.advertise<geometry_msgs::PoseArray>("current_waypoints_array", 1);

    // advertise all servers needed for the user to give commands
    cancel_current_waypoint_server_ = nh.advertiseService("cancel_current_waypoint", &WaypointsCallbackNode::cancelCurrentWaypoint, this);
    clear_waypoints_sequence_server_ = nh.advertiseService("clear_waypoints_sequence", &WaypointsCallbackNode::clearWaypointsSequence, this);
    load_waypoints_sequence_server_ = nh.advertiseService("load_waypoints_sequence", &WaypointsCallbackNode::loadWaypointsSequence, this);
    save_waypoints_sequence_server_ = nh.advertiseService("save_waypoints_sequence", &WaypointsCallbackNode::saveWaypointsSequence, this);
    start_waypoints_sequence_server_ = nh.advertiseService("start_waypoints_sequence", &WaypointsCallbackNode::startWaypointsSequence, this);
    publish_waypoints_sequence_server_ = nh.advertiseService("publish_waypoints_sequence", &WaypointsCallbackNode::publishWaypointsSequence, this);
  }

  ~WaypointsCallbackNode()
  {
  }

  // Called once when the goal completes. Sends next goal in the list, unless the last was reached
  void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    if (state == state.SUCCEEDED)
    {
      ++current_waypoints_it_; current_waypoint_index_++;
      if (current_waypoints_it_ == waypoints_list_.end())
      {
        ROS_INFO("End of waypoints sequence reached");
      }
      else
      {
        ROS_INFO("Sending next waypoint");
        sendCurrentWaypoint();
      }
    }
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("Goal just went active");
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
  {
    ROS_INFO_THROTTLE(5, "Rallying waypoint %u out of %lu", current_waypoint_index_, waypoints_list_.size());
  }

  void sendCurrentWaypoint()
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = current_waypoints_it_->target_pose.header.frame_id;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = current_waypoints_it_->target_pose.pose;
    mbc_.sendGoal(goal, 
      boost::bind(&WaypointsCallbackNode::doneCb, this, _1, _2),
      boost::bind(&WaypointsCallbackNode::activeCb, this),
      boost::bind(&WaypointsCallbackNode::feedbackCb, this, _1));
  }

  // Cancel current waypoint and either try to go on to the next or cancel the whole sequence
  bool cancelCurrentWaypoint(ros_simple_waypoints::CancelCurrentWaypointRequest& req, ros_simple_waypoints::CancelCurrentWaypointResponse& res)
  {
    res.success = false;
    if (req.skip)
    {
      ++current_waypoints_it_;
      if (current_waypoints_it_ == waypoints_list_.end())
      {
        ROS_INFO("Cannot skip last waypoint, aborting instead");
        mbc_.cancelGoal();
        res.success = true;
      }
      else
      {
        current_waypoint_index_++;
        sendCurrentWaypoint();
        ROS_INFO("Skipped waypoint");
        res.success = true;        
      }
    }
    else
    {
      mbc_.cancelGoal();
      ROS_INFO("Cancelled current goal and aborted sequence");
      res.success = true;
    }
    return res.success;
  }

  // Append a waypoint to the current list
  void addWaypoint(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    move_base_msgs::MoveBaseGoal new_waypoint;
    new_waypoint.target_pose.header.frame_id = msg->header.frame_id;
    new_waypoint.target_pose.header.stamp = ros::Time::now();
    new_waypoint.target_pose.pose = msg->pose;
    waypoints_list_.push_back(new_waypoint);
    // adapt goal orientation of previous waypoint to face the new one
    if (waypoints_list_.size()>1) // if there is a previous one
    {
      double delta_x, delta_y, heading;
      delta_x = waypoints_list_.back().target_pose.pose.position.x - last_waypoint_it_->target_pose.pose.position.x;
      delta_y = waypoints_list_.back().target_pose.pose.position.y - last_waypoint_it_->target_pose.pose.position.y;
      heading = atan2(delta_y, delta_x);
      last_waypoint_it_->target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
      ++last_waypoint_it_;
    }
    else
    {
      last_waypoint_it_ = waypoints_list_.begin();
    }   
    ROS_INFO("Added new waypoint to list");
  }

  // Erase the current list of waypoints
  bool clearWaypointsSequence(ros_simple_waypoints::ClearWaypointsSequenceRequest& req, ros_simple_waypoints::ClearWaypointsSequenceResponse& res)
  {
    try
    {
      waypoints_list_.clear();
      ROS_INFO("Cleared waypoints list");
      res.success = true;
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      res.success = false;
    }
    return res.success;
  }

  // Start sending the sequence
  bool startWaypointsSequence(ros_simple_waypoints::StartWaypointsSequenceRequest& req, ros_simple_waypoints::StartWaypointsSequenceResponse& res)
  {
    if (!waypoints_list_.empty())
    {
      current_waypoints_it_ = waypoints_list_.begin();
      current_waypoint_index_ = 1;
      sendCurrentWaypoint();
      res.success = true;
    }
    else
    {
      ROS_INFO("Waypoints list is empty!");
      res.success = false;
    }
    return res.success;
  }

  // Reset the iterator to start from the beginning of the sequence
  void resetIndex()
  {
    if (!waypoints_list_.empty())
    {
      current_waypoints_it_ = waypoints_list_.begin();
      current_waypoint_index_ = 1;
    }
  }

  // load the sequence of waypoints to a YAML file
  bool loadWaypointsSequence(ros_simple_waypoints::LoadWaypointsSequenceRequest& req, ros_simple_waypoints::LoadWaypointsSequenceResponse& res)
  {
    std::string filepath = req.filepath; // TODO: sanitize input
    try
    {
      YAML::Node root = YAML::LoadFile(filepath);
      YAML::Node waypoints_seq = root["waypoints_sequence"]["waypoints"];
      waypoints_list_ = loadWaypointsListFromYAML(waypoints_seq);
      ROS_INFO("Loaded sequence from %s", filepath.c_str());
      res.success = true;
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      res.success = false;
    }
    return res.success;
  }

  // Save the sequence of waypoints to a YAML file
  bool saveWaypointsSequence(ros_simple_waypoints::SaveWaypointsSequenceRequest& req, ros_simple_waypoints::SaveWaypointsSequenceResponse& res)
  {
    std::string filepath = req.filepath; // TODO: sanitize input
    try
    {
      YAML::Emitter yaml_output;
      yaml_output << waypoints_list_;
      std::ofstream fout(filepath);
      fout << yaml_output.c_str();
      ROS_INFO("Saved sequence to %s", filepath.c_str());
      res.success = true;
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      res.success = false;
    }
    return res.success;
  }

  bool publishWaypointsSequence(ros_simple_waypoints::PublishWaypointsSequenceRequest& req, ros_simple_waypoints::PublishWaypointsSequenceResponse& res)
  {
    if (!waypoints_list_.empty())
    {
      try
      {
        geometry_msgs::PoseArray wp_array;
        geometry_msgs::Pose wp;
        auto wp_it = waypoints_list_.begin();
        wp_array.header.frame_id = wp_it->target_pose.header.frame_id; // TODO: ensure frame_id is the same for all points
        wp_array.header.stamp = ros::Time::now();
        while (wp_it != waypoints_list_.end())
        {
          wp.orientation = wp_it->target_pose.pose.orientation;
          wp.position = wp_it->target_pose.pose.position;
          wp_array.poses.push_back(wp);
          ++wp_it;
        }
        waypoints_export_pub_.publish(wp_array);
        res.success = true;
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
        res.success = false;
      }
    }
    else
    {
      ROS_INFO("Cannot publish empty waypoints list");
      res.success = false;
    }
    return res.success;
  }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "ros_simple_waypoints_callback");
  WaypointsCallbackNode waypoints_cb_node; 
  ros::spin();
  return 0;
}