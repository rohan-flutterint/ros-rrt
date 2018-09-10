/*
For class E599 Special Topics in Autonomous Robotics
ISE, Indiana University
Lantao Liu
9/21/2017
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_demo");

  //create a ros handle (pointer) so that you can call and use it
  ros::NodeHandle n;

  //in <>, it specified the type of the message to be published
  //in (), first param: topic name; second param: size of queued messages, at least 1 
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("some_chatter", 10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  //each second, ros "spins" and draws 20 frames
  ros::Rate loop_rate(20);

  int frame_count = 0;
  float f = 0.0;

  while (ros::ok())
  {
    //first create a string typed (std_msgs) message
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Frame index: " << frame_count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str()); //printing on screen

    //publisher publishes messages, the msg type must be consistent with definition advertise<>(); 
    chatter_pub.publish(msg);

  /******************** From here, we are defining and drawing two obstacles in the workspace **************************/

    // define two obstacles
    visualization_msgs::Marker obst1, obst2; 

    // Set obst1 and obst2 as a Cube and Cylinder, respectively
    obst1.type = visualization_msgs::Marker::CUBE;
    obst2.type = visualization_msgs::Marker::CYLINDER;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    obst1.header.frame_id = obst2.header.frame_id = "map";  //NOTE: this should be "paired" to the frame_id entry in Rviz
    obst1.header.stamp = obst2.header.stamp = ros::Time::now();

    // Set the namespace and id 
    obst1.ns = obst2.ns = "obstacles";
    obst1.id = 0;
    obst2.id = 1;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    obst1.action = obst2.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker 
    obst1.scale.x = obst1.scale.y = obst1.scale.z = 1.0; //1x1x1 here means each side of the cube is 1m long
    obst2.scale.x = obst2.scale.y = obst2.scale.z = 1.0; //1x1x1 here means the cylinder as diameter 1m and height 1m

    // Set the pose of the marker. since a side of the obstacle obst1 is 1m as defined above, now we place the obst1 center at (1, 2, 0.5). z-axis is height
    obst1.pose.position.x = 1;
    obst1.pose.position.y = 2;
    obst1.pose.position.z = 0.5; 
    obst1.pose.orientation.x = 0.0;
    obst1.pose.orientation.y = 0.0;
    obst1.pose.orientation.z = 0.0;
    obst1.pose.orientation.w = 1.0;	//(x, y, z, w) is a quaternion, ignore it here

    obst2.pose.position.x = -2;
    obst2.pose.position.y = -1;
    obst2.pose.position.z = 0.5;
    obst2.pose.orientation = obst1.pose.orientation;

    // Set the color red, green, blue. if not set, by default the value is 0
    obst1.color.r = 0.0f; 
    obst1.color.g = 1.0f;
    obst1.color.b = 0.0f;
    obst1.color.a = 1.0;		//be sure to set alpha to something non-zero, otherwise it is transparent
    obst2.color = obst1.color;

    obst1.lifetime = obst2.lifetime = ros::Duration();

    // publish these messages to ROS system
    marker_pub.publish(obst1);
    marker_pub.publish(obst2);

  /************************* From here, we are using points, lines, to draw a tree structure *** ******************/

    //we use static here since we want to incrementally add contents in these mesgs, otherwise contents in these msgs will be cleaned in every ros spin.
    static visualization_msgs::Marker vertices, edges;

    vertices.type = visualization_msgs::Marker::POINTS;
    edges.type = visualization_msgs::Marker::LINE_LIST;

    vertices.header.frame_id = edges.header.frame_id = "map";
    vertices.header.stamp = edges.header.stamp = ros::Time::now();
    vertices.ns = edges.ns = "vertices_and_lines";
    vertices.action = edges.action = visualization_msgs::Marker::ADD;
    vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

    vertices.id = 0;
    edges.id = 1;

    // POINTS markers use x and y scale for width/height respectively
    vertices.scale.x = 0.05;
    vertices.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    edges.scale.x = 0.02; //tune it yourself

    // Points are green
    vertices.color.g = 1.0f;
    vertices.color.a = 1.0;

    // Line list is red
    edges.color.r = 1.0;
    edges.color.a = 1.0;

    geometry_msgs::Point p0;	// root vertex
    p0.x = p0.y = p0.z = 0;	
    int num_slice = 20;		// e.g., to create 20 edges 
    float length = 1;		//length of each edge

    static int slice_index = 0;
    int herz = 10;		//every 10 ROS frames we draw an edge
    if(frame_count % herz == 0 && edges.points.size()<= 2*num_slice)
    {  
      geometry_msgs::Point p;

      float angle = slice_index*2*M_PI/num_slice;
      slice_index ++ ;
      p.x = length * cos(angle);
      p.y = length * sin(angle);
      p.z = 0;

      vertices.points.push_back(p);	//for drawing vertices
      edges.points.push_back(p0);	//for drawing edges. The line list needs two points for each line
      edges.points.push_back(p);
    }

    //publish msgs
    marker_pub.publish(vertices);
    marker_pub.publish(edges);


  /******************** From here, we are defining and drawing a simple robot **************************/

    // a simple sphere represents a robot
    static visualization_msgs::Marker rob;
    static visualization_msgs::Marker path;
    rob.type = visualization_msgs::Marker::SPHERE;
    path.type = visualization_msgs::Marker::LINE_STRIP;

    rob.header.frame_id = path.header.frame_id = "map";  //NOTE: this should be "paired" to the frame_id entry in Rviz, the default setting in Rviz is "map"
    rob.header.stamp = path.header.stamp = ros::Time::now();
    rob.ns = path.ns = "rob";
    rob.id = 0;
    path.id = 1;
    rob.action = path.action = visualization_msgs::Marker::ADD;
    rob.lifetime = path.lifetime = ros::Duration();

    rob.scale.x = rob.scale.y = rob.scale.z = 0.3; 

    rob.color.r = 1.0f;
    rob.color.g = 0.5f;
    rob.color.b = 0.5f;
    rob.color.a = 1.0; 

    // path line strip is blue
    path.color.b = 1.0;
    path.color.a = 1.0;

    path.scale.x = 0.02; 
    path.pose.orientation.w = 1.0;

    int num_slice2 = 200;		// divide a circle into segments
    static int slice_index2 = 0;
    if(frame_count % 2 == 0 && path.points.size() <= num_slice2)  //update every 2 ROS frames
    {
      geometry_msgs::Point p;

      float angle = slice_index2*2*M_PI/num_slice2;
      slice_index2 ++ ;
      p.x = 4 * cos(angle) - 0.5;  	//some random circular trajectory, with radius 4, and offset (-0.5, 1, .05)
      p.y = 4 * sin(angle) + 1.0;
      p.z = 0.05;
      
      rob.pose.position = p;
      path.points.push_back(p);		//for drawing path, which is line strip type
    }

    marker_pub.publish(rob);
    marker_pub.publish(path);


  /******************** To here, we finished displaying our components **************************/
 
    // check if there is a subscriber. Here our subscriber will be Rviz
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please run Rviz in another terminal.");
      sleep(1);
    }

    //ros spins, force ROS frame to refresh/update once
    ros::spinOnce();

    loop_rate.sleep();
    ++frame_count;
  }

  return 0;
}


