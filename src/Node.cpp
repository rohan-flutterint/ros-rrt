//
// Created by Sachith Danushka Withana on 9/6/18.
//

#include "Node.h"
#include <cstdlib>
#include <cstdio>

namespace e503 {
    Node::Node(float x, float y, float theta) {
        this->x = x;
        this->y = y;
        this->theta = theta;
        parent = NULL;
        p.x = x;
        p.y = y;
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

}