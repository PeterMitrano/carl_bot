/*!
 * \create_parking_spots.h
 * \brief Creates clickable parking spots for web interface/rviz
 *
 * create_parking_spots runs a ROS interactive marker server. It creates clickable markers that send move_base goals to carl. 
 * The positions of the markers are defined in the rail_collada_models package, and the ilab_description must be loaded on the parameter
 * server for parking spots to be generated.
 *
 * \author Peter Mitrano, WPI - pdmitrano@wpi.edu
 * \date February 11, 2015
 */

#ifndef CLICKABLE_INTERACTION_H
#define CLICKABLE_INTERACTION_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <actionlib/client/simple_action_client.h>
#include <carl_moveit/MoveToPoseAction.h>
#include <carl_moveit/ArmAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <tf/transform_listener.h>
#include <carl_safety/Error.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <urdf/model.h>
//#include <carl_moveit/MoveToPoseGoal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<carl_moveit::MoveToPoseAction> ArmClient;
typedef actionlib::SimpleActionClient<carl_moveit::ArmAction> CommonActionsClient;
typedef boost::shared_ptr<urdf::Link> Link_ptr;
typedef std::map<std::string, Link_ptr> LinkMap;

/*!
 * \class ParkingSpots
 * \brief Creates clickable parking spots for carl
 */

class ClickableInteraction
{

public:

  /**
  * \brief Constructor
  *	This starts the server, waits for param server and create clickable parking spots
  */
  ClickableInteraction();

private:

  MoveBaseClient move_base_client_; //!< uses move base client to set nav goals
  ArmClient arm_client_; //!< uses arm client to set arm goals
  CommonActionsClient common_actions_client_;//!< home the arm before navigation anywhere!
  ros::Publisher safetyErrorPublisher_; //!< used to send feedback to the web-interface
  tf::TransformListener listener_; //!< uses tf to find closest parking spot
  std::map<std::string, boost::shared_ptr<urdf::Link> > parking_links_;
  std::map<std::string, boost::shared_ptr<urdf::Link> > surface_links_;

  interactive_markers::InteractiveMarkerServer server_; //!< creates marker server for clickable parking spots

  /**
  * /brief Creates an parking marker for the given link
  * @param frame_id the frame id, or link name, to be used to create the marker
  * @return the interactive marker to be used as a clickable parking spot
  */
  visualization_msgs::InteractiveMarker createParkingSpot(std::string frame_id);

  /**
  * /brief waits for actionlib servers and adds them to the interactive marker server
  */
  void initializeMarkers();

  /**
  * /brief Creates a surface marker for the given link
  * @param frame_id the frame id, or link name, to be used to create the marker
   * @param geom a pointer to the type of geometry to be used for the surface marker
  * @return the interactive marker to be used as a clickable surface
  */
  visualization_msgs::InteractiveMarker createSurface(std::string frame_id, const urdf::Geometry *geom);

  /**
  * /brief Callback function for when floor marker is clicked. Sets move_base goal to carl
  * @param f contains information about where the marker was clicked, if it was a press or release, and the marker pose
  */
  void onParkingClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f);

  /**
  * /brief Callback function for when surface marker is clicked. Sets move_base goal to carl & goal to arm. Carl will first navigate to the location, then point to the clicked spot.
  * @param f contains information about where the marker was clicked (eg if it was a press or release, and the marker pose)
  */
  void onSurfaceClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f);

  /**
  * /brief moves carl to a given pose. used in the on click functions
  * @return true if successful, false if failed
  */
  bool moveToPose(geometry_msgs::Pose new_pose, std_msgs::Header header);

  /**
  * /brief Check if a given link name ends with nav_goal_link, and should therefore have a parking spot made at its location
  * @param link_name the frame id or name of the link
  * @return if the link ends in nav_goal_link and needs a marker
  */
  bool isParkingSpot(std::string link_name);

  /**
  * /brief Check if a given link name ends with surface_link, and should therefore have a surface made at its location. Size of the marker is taken from the surfaces.yaml file in the rail_collada_models packge
  * @param link_name the frame id or name of the link
  * @return if the link ends in surface_link and needs a marker
  */
  bool isSurface(std::string link_name);
};

#endif