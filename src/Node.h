//
// Created by Sachith Danushka Withana on 9/6/18.
//

#ifndef RTT_NODE_H
#define RTT_NODE_H
#include <vector>
#include <geometry_msgs/Point.h>
namespace e503 {
    class Node {
    public:
        float x, y, theta;
        Node *parent;
        std::vector<Node *> children;
        geometry_msgs::Point p;
        Node(float x, float y, float theta);
        geometry_msgs::Point getNodeAsPoint();
        Node(geometry_msgs::Point);
        bool printNode();
        bool closeTo(Node *node, float distanceTolerance);
        bool equals(Node* node);
        void calculateGradient(Node *node);
    };
}

#endif //RTT_NODE_H
