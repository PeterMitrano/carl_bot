#include <carl_interactive_manipulation/clickable_interaction.h>

/**
\brief runs an interacive marker server creates clickable navigation goals infront of furniture
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "clickable_interaction");

  ClickableInteraction parkingSpots;

  ros::spin();

  return EXIT_SUCCESS;
}

ClickableInteraction::ClickableInteraction()
    : move_base_client_("move_base", true),
      arm_client_("carl_moveit_wrapper/move_to_pose", true),
      server_("clickable_markers"),
      common_actions_client_("carl_moveit_wrapper/common_actions/arm_action", true)

{
  ros::NodeHandle node;
  safetyErrorPublisher_ = node.advertise<carl_safety::Error>("carl_safety/error", 1);

  ROS_INFO("waiting for servers...");

  //first, connect to the two actionlib servers
  bool status = move_base_client_.waitForServer();

  if (!status)
  {
    ROS_INFO("Couldn't connect to move_base.");
  }
  else
  {
    ROS_INFO("connected to move_base");

    status = arm_client_.waitForServer();

    if (!status)
    {
      ROS_INFO("Couldn't connect to carl_moveit_wrapper/move_to_pose.");
    }
    else
    {
      ROS_INFO("connected to carl_moveit_wrapper/move_to_pose");
      initializeMarkers();

    }
  }
}

void ClickableInteraction::initializeMarkers()
{

  urdf::Model ilab;

  //load the urdf with all the furniture off the param server
  if (!ilab.initParam("/ilab_description"))
  {
    ROS_INFO("couldn't find /ilab_description on param server.");
  }
  else
  {

    std::map<std::string, boost::shared_ptr<urdf::Link> > links = ilab.links_;
    std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator itr;

    //go through all links and filter out the ones that end in "nav_goal_link"
    for (itr = links.begin(); itr != links.end(); itr++)
    {
      std::string link_name = itr->first;
      Link_ptr link = itr->second;
      std::pair<std::string, Link_ptr> link_pair = make_pair(link_name, link);

      if (isParkingSpot(link_name))
      {
        ROS_INFO("creating parking spot %s", link_name.c_str());
        visualization_msgs::InteractiveMarker marker = createParkingSpot(link_name);
        parking_links_.insert(link_pair);
        server_.insert(marker, boost::bind(&ClickableInteraction::onParkingClick, this, _1));
      }
      else if (isSurface(link_name))
      {
        ROS_INFO("creating surface %s", link_name.c_str());
        visualization_msgs::InteractiveMarker marker = createSurface(link_name, link->collision->geometry.get());
        surface_links_.insert(link_pair);
        server_.insert(marker, boost::bind(&ClickableInteraction::onSurfaceClick, this, _1));
      }
    }

    ROS_INFO("creating clickable nav goals & pointing spots...");

    //when these are called the markers will actually appear
    server_.applyChanges();
  }
}

void ClickableInteraction::onParkingClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f)
{
  if (f->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
  {
    bool success = moveToPose(f->pose, f->header);
    if (success)
    {
      ROS_INFO("finished successfully!");
    }
    else
    {
      ROS_INFO("timed out! goal not reached");
      carl_safety::Error error;
      error.message = "Goal not reached! Navigation timed out";
      error.severity = 0;
      safetyErrorPublisher_.publish(error);
    }
  }
}

void ClickableInteraction::onSurfaceClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &f)
{
  if (f->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
  {
    std::string surface_link_name = f->header.frame_id;
    ROS_INFO("%s clicked at (%f,%f) ", surface_link_name.c_str(), f->mouse_point.x, f->mouse_point.y);

    //first, set the robot to the nearest nav goal
    LinkMap::iterator itr;

    //go through all the nav goals, find the closest one
    float min_dist = 1000; //you have to be at most 0.6 meter away
    Link_ptr closest_parking_spot;

    tf::StampedTransform transform;

    for (itr = parking_links_.begin(); itr != parking_links_.end(); itr++)
    {
      std::string nav_link_name = itr->first;
      Link_ptr parking_spot = itr->second;

      if (isParkingSpot(nav_link_name))
      {
        listener_.lookupTransform(nav_link_name, surface_link_name, ros::Time(0), transform);

        // ignore z, we're planning in 2D here
        float x = transform.getOrigin().x();
        float y = transform.getOrigin().y();
        float dist = sqrtf(x * x + y * y);

        if (dist < min_dist)
        {
          min_dist = dist;
          closest_parking_spot = parking_spot;
        }
      }
    }

    ROS_INFO("geometry isn't null");

    if (closest_parking_spot == NULL)
    {
      ROS_INFO("no closest nav goal found");
      carl_safety::Error error;
      error.message = "No navigation goal found within reach of surface!";
      error.severity = 0;
      safetyErrorPublisher_.publish(error);
    }
    else
    {
      ROS_INFO("closest nav goal is %s", closest_parking_spot->name.c_str());

      geometry_msgs::Pose base_pose;
      base_pose.orientation.x = 0;
      base_pose.orientation.y = 0;
      base_pose.orientation.z = 1;
      base_pose.orientation.w = 1;

      std_msgs::Header header;
      header.frame_id = closest_parking_spot->name;
      bool success = moveToPose(base_pose, header);

      if (success)
      {
        //set arm goal now!
        carl_moveit::MoveToPoseGoal goal;
        goal.pose.header = f->header;
        goal.pose.pose = f->pose;
        goal.pose.pose.position.x += f->mouse_point.x;
        goal.pose.pose.position.y += f->mouse_point.y;
        goal.pose.pose.position.z += 0.2;

        tf::StampedTransform tf;
        listener_.lookupTransform("jaco_link_hand", goal.pose.header.frame_id, ros::Time(0), tf);
        ROS_INFO("transform from hand to frame is %f %f %f", tf.getOrigin().getX(), tf.getOrigin().getY(),
                 tf.getOrigin().getZ());

        arm_client_.sendGoal(goal);

        success = move_base_client_.waitForResult();

        if (success < 0)
        {
          carl_safety::Error error;
          error.message = "Arm pose not reached! It probably timed out";
          error.severity = 0;
          safetyErrorPublisher_.publish(error);
        }
      }
      else
      {
        carl_safety::Error error;
        error.message = "Driving goal not reached! It probably timed out";
        error.severity = 0;
        safetyErrorPublisher_.publish(error);
      }
    }
  }
}

bool ClickableInteraction::moveToPose(geometry_msgs::Pose arm_pose, std_msgs::Header header)
{

  //HOME THE ARM FIRST
  carl_moveit::ArmGoal homeGoal;
  homeGoal.action = carl_moveit::ArmGoal::READY;
  common_actions_client_.sendGoal(homeGoal);


  geometry_msgs::PoseStamped target_pose;
  target_pose.header = header;
  target_pose.pose = arm_pose;

  //create goal
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = target_pose;

  //send action goal
  move_base_client_.sendGoal(goal);

  return move_base_client_.waitForResult();
}

bool ClickableInteraction::isParkingSpot(std::string link_name)
{
  std::string search_param = "nav_goal_link";
  return link_name.find(search_param, link_name.length() - search_param.length()) != std::string::npos;
}

bool ClickableInteraction::isSurface(std::string link_name)
{
  std::string search_param = "surface_link";
  return link_name.find(search_param, link_name.length() - search_param.length()) != std::string::npos;
}

visualization_msgs::InteractiveMarker ClickableInteraction::createParkingSpot(std::string frame_id)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.scale = 1;
  int_marker.name = frame_id + "_parking_spot";

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.name = "button";

  visualization_msgs::Marker box;
  box.type = visualization_msgs::Marker::CUBE;
  box.scale.x = 0.15;
  box.scale.y = 0.15;
  box.scale.z = 0.05;
  box.color.r = 0;
  box.color.g = 0.5;
  box.color.b = 0.25;
  box.color.a = 1;

  control.markers.push_back(box);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  return int_marker;
}

//this theoreticall supports surfaces of different models types, although I've only tested it with boxes so far
visualization_msgs::InteractiveMarker ClickableInteraction::createSurface(std::string frame_id,
                                                                          const urdf::Geometry *geom)
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.scale = 1;
  int_marker.name = frame_id + "_surface";

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.name = "button";

  //create marker & common fields
  visualization_msgs::Marker marker;
  marker.color.r = 0;
  marker.color.g = 0.25;
  marker.color.b = 0.5;
  marker.color.a = 1;

  switch (geom->type)
  {
    case urdf::Geometry::BOX:
    {
      ROS_INFO("surface box...");
      urdf::Vector3 dim = dynamic_cast<const urdf::Box *>(geom)->dim;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = dim.x;
      marker.scale.y = dim.y;
      marker.scale.z = dim.z;
      break;
    }
    case urdf::Geometry::SPHERE:
    {
      double rad = dynamic_cast<const urdf::Sphere *>(geom)->radius;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = rad;
      marker.scale.y = rad;
      marker.scale.z = rad;
      break;
    }
    case urdf::Geometry::CYLINDER:
    {
      double rad = dynamic_cast<const urdf::Cylinder *>(geom)->radius;
      double len = dynamic_cast<const urdf::Cylinder *>(geom)->length;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.scale.x = rad;
      marker.scale.y = rad;
      marker.scale.z = len;
      break;
    }
    case urdf::Geometry::MESH:
    {
      const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh *>(geom);
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      if (!mesh->filename.empty())
      {
        marker.mesh_resource = mesh->filename;
      }
      else
        ROS_WARN("Empty mesh filename");
      break;
    }
    default:
    {
      ROS_ERROR("Unknown geometry type: %d", (int) geom->type);
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 0.05;
    }
  }


  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  return int_marker;
}
