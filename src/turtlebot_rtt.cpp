#include <iostream>
#include <cmath>
#include "Node.h"
#include "RRT.h"
#include "Obstacle.h"
#include "StateSpace.h"
#include <stdlib.h>
#include <cstdio>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

using namespace e503;

int main(int argc, char **argv) {
    float GOAL_X = 28;
    float GOAL_Y = 10;

    ros::init(argc, argv, "turtlebot_rrt");

    //create a ros handle (pointer) so that you can call and use it
    ros::NodeHandle n;

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    //each second, ros "spins" and draws 20 frames
    ros::Rate loop_rate(20);

    int frame_count = 0;
    float f = 0.0;

    float EPSILON = 0.2;
    float robot_scale_x = 1.5;
    float robot_scale_y = 1;
    float obstaclePaddingCSpace = (robot_scale_x > robot_scale_y) ? robot_scale_x/2 : robot_scale_y/2;
    obstaclePaddingCSpace += 0.05;
    // for generating a random number
    srand (static_cast <unsigned> (time(NULL)));
    StateSpace stateSpace(0, 30, 0, 30);
    stateSpace.addObstacle(new Obstacle(19, 21 , 4, 26, obstaclePaddingCSpace, obstaclePaddingCSpace));
    stateSpace.addObstacle(new Obstacle(10, 14 , 10, 18, obstaclePaddingCSpace, obstaclePaddingCSpace));
    stateSpace.addObstacle(new Obstacle(10, 14, 0, 8, obstaclePaddingCSpace, obstaclePaddingCSpace));
    stateSpace.addObstacle(new Obstacle(10, 14 , 20, 30, obstaclePaddingCSpace, obstaclePaddingCSpace));
    Node *startNode = new Node(0, 0, 0);
    Node *goalNode = new Node(GOAL_X,GOAL_Y,0);
    RRT rrt;
    rrt.insert(startNode, 0);
    std::vector<Node *> shortestPath;
    std::vector<Node *> roughShortestPath;
    std::vector<Node *> robotPath;
    std::vector<Node *> smoothenedRobotPath;

    // the goalNode is found
    bool goalFound = false;
    float goalFindTolerance = 1.0;

    bool shortestPathDrawn = false;

    while (ros::ok())
    {   /* *//******************** From here, we are defining and drawing two obstacles in the workspace **************************/

        // define two obstacles
        visualization_msgs::Marker obst1, obst2, obst3, obst4, start, goal;

        // Set obst1 and obst2 as a Cube and Cylinder, respectively
        obst1.type = visualization_msgs::Marker::CUBE;
        obst2.type = visualization_msgs::Marker::CUBE;
        obst3.type = visualization_msgs::Marker::CUBE;
        obst4.type = visualization_msgs::Marker::CUBE;
        start.type = visualization_msgs::Marker::SPHERE;
        goal.type = visualization_msgs::Marker::SPHERE;

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        obst1.header.frame_id = obst2.header.frame_id = obst3.header.frame_id = obst4.header.frame_id= start.header.frame_id = goal.header.frame_id = "map";
        obst1.header.stamp = obst2.header.stamp = obst3.header.stamp= obst4.header.stamp= start.header.stamp = goal.header.stamp = ros::Time::now();

        // Set the namespace and id
        obst1.ns = obst2.ns = obst3.ns = obst4.ns =  "obstacles";
        obst1.id = 0;
        obst2.id = 1;
        obst3.id = 2;
        obst4.id = 3;
        start.ns = goal.ns = "markers";
        start.id = 2;
        goal.id = 3;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        obst1.action = obst2.action = obst3.action = obst4.action =  start.action = goal.action = visualization_msgs::Marker::ADD;

        start.scale.x = start.scale.y = goal.scale.x = goal.scale.y = 0.5;
        goal.scale.z = start.scale.z = 0.1;
                // Set the scale of the marker
        obst1.scale.x = 2.0;
        obst1.scale.y = 22.0;
        obst1.scale.z = obst2.scale.z = obst3.scale.z=  obst4.scale.z= 0.5;
        obst2.scale.x = 4.0;
        obst2.scale.y = 8.0;

        obst3.scale.x = 4.0;
        obst3.scale.y = 8.0;

        obst4.scale.x = 4.0;
        obst4.scale.y = 10.0;

        start.pose.position.x = startNode->x;
        start.pose.position.y = startNode->y;
        start.pose.position.z = 0;
        goal.pose.position.x = goalNode->x;
        goal.pose.position.y = goalNode->y;
        goal.pose.position.z = 0;


        // Set the pose of the marker. since a side of the obstacle obst1 is 1m as defined above, now we place the obst1 center at (1, 2, 0.5). z-axis is height
        obst1.pose.position.x = 20;
        obst1.pose.position.y = 15;
        obst1.pose.position.z = 0;
        obst1.pose.orientation.x = 0.0;
        obst1.pose.orientation.y = 0.0;
        obst1.pose.orientation.z = 0.0;
        obst1.pose.orientation.w = 1.0;	//(x, y, z, w) is a quaternion, ignore it here

        obst2.pose.position.x = 12;
        obst2.pose.position.y = 14;
        obst2.pose.position.z = 0;

        obst4.pose.position.x = 12;
        obst4.pose.position.y = 25;
        obst4.pose.position.z = 0;

        obst3.pose.position.x = 12;
        obst3.pose.position.y = 4;
        obst3.pose.position.z = 0;
        start.pose.orientation = goal.pose.orientation = obst4.pose.orientation = obst3.pose.orientation =  obst2.pose.orientation = obst1.pose.orientation;

        // Set the color red, green, blue. if not set, by default the value is 0
        obst1.color.r = 1.0f;
        obst1.color.g = 1.0f;
        obst1.color.b = 1.0f;
        obst1.color.a = 1.0;		//be sure to set alpha to something non-zero, otherwise it is transparent

        obst2.color = obst3.color = obst4.color = obst1.color;

        start.color.r = 0.0f;
        start.color.g = 0.0f;
        start.color.b = 1.0f;
        start.color.a = 1.0;

        goal.color.r = 1.0f;
        goal.color.g = 0.0f;
        goal.color.b = 0.0f;
        goal.color.a = 1.0;


        obst1.lifetime = obst2.lifetime = obst3.lifetime = obst4.lifetime = start.lifetime = goal.lifetime = ros::Duration();

        // publish these messages to ROS system
        marker_pub.publish(obst1);
        marker_pub.publish(obst2);
        marker_pub.publish(obst3);
        marker_pub.publish(obst4);
        marker_pub.publish(start);
        marker_pub.publish(goal);
        /************************* From here, we are using points, lines, to draw a tree structure *** ******************/

        //we use static here since we want to incrementally add contents in these mesgs, otherwise contents in these msgs will be cleaned in every ros spin.
        static visualization_msgs::Marker vertices, edges, sp_vertices, sp_edges;

        vertices.type = sp_vertices.type= visualization_msgs::Marker::POINTS;
        edges.type = sp_edges.type = visualization_msgs::Marker::LINE_LIST;

        vertices.header.frame_id = edges.header.frame_id = sp_edges.header.frame_id = sp_vertices.header.frame_id = "map";
        vertices.header.stamp = edges.header.stamp = sp_edges.header.stamp = sp_vertices.header.stamp = ros::Time::now();
        vertices.ns = edges.ns = "vertices_and_lines";
        sp_edges.ns = sp_vertices.ns = "shortest_path";
        vertices.action = edges.action = sp_edges.action = sp_vertices.action = visualization_msgs::Marker::ADD;
        vertices.pose.orientation.w = edges.pose.orientation.w = sp_vertices.pose.orientation.w = sp_edges.pose.orientation.w = 1.0;

        vertices.id = 0;
        edges.id = 1;
        sp_vertices.id = 0;
        sp_edges.id = 1;

        // POINTS markers use x and y scale for width/height respectively
        vertices.scale.x = 0.05;
        vertices.scale.y = 0.05;

        sp_vertices.scale.x = 0.1;
        sp_vertices.scale.y = 0.1;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        edges.scale.x = 0.02; //tune it yourself

        sp_edges.scale.x = 0.1;

        // Points are green
        vertices.color.g = 1.0f;
        vertices.color.a = 1.0;

        sp_vertices.color.r = 1.0f;
        sp_vertices.color.a = 1.0;

        // Line list is red
        edges.color.r = 1.0;
        edges.color.a = 1.0;

        sp_edges.color.g = 1.0;
        sp_edges.color.a = 1.0;

        geometry_msgs::Point p0 = startNode->getNodeAsPoint();
        float length = 1;		//length of each edge

        // find the goal using RTT
        if (!goalFound) {
            int herz = 1;        //every  ROS frames we draw an edge
            if (frame_count % herz == 0) {

                // adding goalNode bias
                double randomNumber = ((double) rand() / (RAND_MAX));
                Node *randomNode = NULL;
                if (randomNumber > 0.5) {
                    randomNode = stateSpace.genRandomNodeInSpace();
                } else {
                    randomNode = goalNode;
                }
                Node *closestNode = rrt.getNearestNeighbor(randomNode);
                Node *newNode = rrt.extendNode(closestNode, randomNode, EPSILON);
                if (!stateSpace.isObstructed(newNode) & !stateSpace.edgeIsObstructed(closestNode, newNode)) {
                    rrt.insert(newNode, closestNode);
                    vertices.points.push_back(newNode->getNodeAsPoint());    //for drawing vertices
                    edges.points.push_back(closestNode->getNodeAsPoint());    //for drawing edges. The line list needs two points for each line
                    edges.points.push_back(newNode->getNodeAsPoint());

                    // if the goal is close enough
                    if (newNode->closeTo(goalNode, goalFindTolerance)) {
                        rrt.insert(goalNode, newNode);
                        goalFound = true;
                        edges.points.push_back(newNode->getNodeAsPoint());
                        edges.points.push_back(goalNode->getNodeAsPoint());

                        //calculate shortest path
                        roughShortestPath = rrt.extractShortestPath(goalNode);
                        std::reverse(roughShortestPath.begin(), roughShortestPath.end());
                        //smoothenedRobotPath = roughShortestPath;

                        smoothenedRobotPath = stateSpace.smoothenPath(roughShortestPath, EPSILON/2);
                        shortestPath = smoothenedRobotPath;
                    }
                }
            }
        }
        // Draw the shortest path
        int herz = 2;
        if ((frame_count % herz == 0) & goalFound & !shortestPathDrawn) {
            if (!shortestPath.empty()) {
                Node *pathNode = shortestPath.back();
                Node *prevNode = pathNode->parent;
                prevNode = (prevNode == 0) ? startNode : prevNode;
                sp_vertices.points.push_back(pathNode->getNodeAsPoint());
                sp_edges.points.push_back(prevNode->getNodeAsPoint());
                sp_edges.points.push_back(pathNode->getNodeAsPoint());
                prevNode = pathNode;
                shortestPath.pop_back();
            }
            else {
                shortestPathDrawn = true;
                robotPath = rrt.processFinalRobotPath(smoothenedRobotPath);
            }
        }

        //publish msgs
        marker_pub.publish(vertices);
        marker_pub.publish(sp_vertices);
        marker_pub.publish(edges);
        marker_pub.publish(sp_edges);
/*


        */
/******************** From here, we are defining and drawing a simple robot **************************/


        // a simple sphere represents a robot
        static visualization_msgs::Marker rob;
        static visualization_msgs::Marker path;
        rob.type = visualization_msgs::Marker::CUBE;
        path.type = visualization_msgs::Marker::LINE_STRIP;

        rob.header.frame_id = path.header.frame_id = "map";  //NOTE: this should be "paired" to the frame_id entry in Rviz, the default setting in Rviz is "map"
        rob.header.stamp = path.header.stamp = ros::Time::now();
        rob.ns = path.ns = "rob";
        rob.id = 0;
        path.id = 1;
        rob.action = path.action = visualization_msgs::Marker::ADD;
        rob.lifetime = path.lifetime = ros::Duration();

        rob.scale.x = robot_scale_x;
        rob.scale.y = robot_scale_y;
        rob.scale.z = 0.3;

        rob.color.r = 1.0f;
        rob.color.g = 0.5f;
        rob.color.b = 0.5f;
        rob.color.a = 1.0;

        // path line strip is blue
        path.color.b = 1.0;
        path.color.a = 1.0;

        path.scale.x = 0.02;
        path.pose.orientation.w = 1.0;

        if(frame_count % 10 == 0 && !robotPath.empty() & shortestPathDrawn)  //update every 2 ROS frames
        {
            Node *robotNode = robotPath.back();
            geometry_msgs::Point p = robotNode->getNodeAsPoint();
            rob.pose.position = p;
            const tf::Quaternion &quaternion = tf::createQuaternionFromYaw(robotNode->theta);
            rob.pose.orientation.w = quaternion.w();
            rob.pose.orientation.x = quaternion.x();
            rob.pose.orientation.y = quaternion.y();
            rob.pose.orientation.z = quaternion.z();
            path.points.push_back(p);		//for drawing path, which is line strip type
            robotPath.pop_back();
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
}
