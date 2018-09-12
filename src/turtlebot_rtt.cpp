#include <iostream>
#include <cmath>
#include "Node.h"
#include "RRT.h"
#include "StateSpace.h"
#include <stdlib.h>
#include <cstdio>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace e503;

int main(int argc, char **argv) {

    ros::init(argc, argv, "turtlebot_rrt");

    //create a ros handle (pointer) so that you can call and use it
    ros::NodeHandle n;

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    //each second, ros "spins" and draws 20 frames
    ros::Rate loop_rate(20);

    int frame_count = 0;
    float f = 0.0;

    // for generating a random number
    srand (static_cast <unsigned> (time(NULL)));
    StateSpace stateSpace(0, 20, 0, 20);
    Node *start = new Node(0, 0, 0);
    Node *goal = new Node(18,18,0);
    RRT rtt;
    rtt.insert(start, 0);

    // the goal is found
    bool goalFound = false;
    float goalFindTolerance = 1.0;

    while (ros::ok())
    {   /* *//******************** From here, we are defining and drawing two obstacles in the workspace **************************/

        // define two obstacles
        visualization_msgs::Marker obst1, obst2;

        // Set obst1 and obst2 as a Cube and Cylinder, respectively
        obst1.type = visualization_msgs::Marker::CUBE;
        obst2.type = visualization_msgs::Marker::CUBE;

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
        obst1.scale.x = obst1.scale.y = obst1.scale.z = 2.0; //1x1x1 here means each side of the cube is 1m long
        obst2.scale.x = obst2.scale.y = obst2.scale.z = 3.0; //1x1x1 here means the cylinder as diameter 1m and height 1m

        // Set the pose of the marker. since a side of the obstacle obst1 is 1m as defined above, now we place the obst1 center at (1, 2, 0.5). z-axis is height
        obst1.pose.position.x = 12;
        obst1.pose.position.y = 14;
        obst1.pose.position.z = 0;
        obst1.pose.orientation.x = 0.0;
        obst1.pose.orientation.y = 0.0;
        obst1.pose.orientation.z = 0.0;
        obst1.pose.orientation.w = 1.0;	//(x, y, z, w) is a quaternion, ignore it here

        obst2.pose.position.x = 10;
        obst2.pose.position.y = 20;
        obst2.pose.position.z = 0;
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

        geometry_msgs::Point p0 = start->getNodeAsPoint();
        float length = 1;		//length of each edge

        if (!goalFound) {
            int herz = 10;        //every 10 ROS frames we draw an edge
            if (frame_count % herz == 0) {

                // adding goal bias
                double randomNumber = ((double) rand() / (RAND_MAX));
                Node *randomNode = NULL;
                if (randomNumber > 0.5) {
                    randomNode = stateSpace.genRandomNodeInSpace();
                } else {
                    randomNode = goal;
                }
                Node *closestNode = rtt.getNearestNeighbor(randomNode);
                Node *newNode = rtt.extendNode(closestNode, randomNode, 1.0);
                if (!stateSpace.isObstructed(newNode)) {
                    rtt.insert(newNode, closestNode);
                    if (newNode->closeTo(goal, goalFindTolerance)) {
                        rtt.insert(goal, newNode);
                        goalFound = true;
                        std::cout << "GOAL FOUND!";
                    }
                    vertices.points.push_back(newNode->getNodeAsPoint());    //for drawing vertices
                    edges.points.push_back(closestNode->getNodeAsPoint());    //for drawing edges. The line list needs two points for each line
                    edges.points.push_back(newNode->getNodeAsPoint());
                }
            }
        }

        //publish msgs
        marker_pub.publish(vertices);
        marker_pub.publish(edges);
/*


        */
/******************** From here, we are defining and drawing a simple robot **************************//*


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

*/

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

    /*
    // for generating a random number
    srand (static_cast <unsigned> (time(NULL)));

    StateSpace stateSpace(0, 20, 0, 20);
    Node *randomNode = stateSpace.genRandomNodeInSpace();
    printf("Random Node: (%.2f, %.2f, %.2f)\n", randomNode->x, randomNode->y, randomNode->theta);

    RRT tree;
    Node *root = new Node(3, 4, 45);
    Node *node2 = new Node(5, 6, 45);
    Node *node3 = new Node(8, 4, 90);
    Node *node4 = new Node(14, 20, 45);
    Node *node5 = new Node(18, 2, 5);
    tree.insert(root, 0);
    tree.insert(node2, root);
    tree.insert(node4, node2);
    tree.insert(node3, root);
    tree.insert(node5, root);

    float epsilon = 1;

    Node *closestNode = tree.getNearestNeighbor(randomNode);
    printf("Closest Node: (%.2f, %.2f, %.2f)\n", closestNode->x, closestNode->y, closestNode->theta);

    RRT rtt;
    Node *newNode = rtt.extendNode(closestNode, randomNode, epsilon);
    printf("New Node: (%.2f, %.2f, %.2f)\n", newNode->x, newNode->y, newNode->theta);
    //tree.printTree(root);
    return 0;*/
}