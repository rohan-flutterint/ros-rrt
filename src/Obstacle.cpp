//
// Created by swithana on 9/12/18.
//

#include "Obstacle.h"

namespace e503 {
    Obstacle::Obstacle(float min_x, float max_x, float min_y, float max_y, float robotScaleX, float robotScaleY) {
        this->min_x = min_x - robotScaleX;
        this->min_y = min_y - robotScaleY;
        this->max_x = max_x + robotScaleX;
        this->max_y = max_y + robotScaleY;
    }

    bool Obstacle::isWithinObstacle(Node *node) {
        return node->x >= min_x & node->x <= max_x & node->y >= min_y & node->y <= max_y;
    }

    bool Obstacle::lineCrossingObstacle(Node *node1, Node *node2) {

    }
}