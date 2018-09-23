//
// Created by Sachith Danushka Withana on 9/6/18.
//

#include "Node.h"
#include <cstdlib>
#include <cstdio>
#include <cmath>

namespace e503 {
    Node::Node(float x, float y, float theta) {
        this->x = x;
        this->y = y;
        this->theta = theta;
        parent = 0;
        p.x = x;
        p.y = y;
        p.z = theta;
    }

    Node::Node(geometry_msgs::Point) {
        this->x = p.x;
        this->y = p.y;
        this->theta = p.z;
    }

    geometry_msgs::Point Node::getNodeAsPoint() {
        return p;
    }

    bool Node::printNode() {
        printf(" Node: (%.2f, %.2f, %.2f) \n", this->x, this->y, this->theta);
    };

    bool Node::closeTo(Node *node, float distanceTolerance) {
        return (abs(this->x - node->x) < distanceTolerance) &  (abs(this->y - node->y) < distanceTolerance);
    }

    bool Node::equals(Node *node) {
        return this->x == node->x & this->y == node->y & this->theta == node->theta;
    }

    void Node::calculateGradient(Node *node) {
        this->theta = atan((node->y - y)/(node->x - x));
    }
}