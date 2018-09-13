//
// Created by swithana on 9/12/18.
//

#include <cmath>
#include "Obstacle.h"

namespace e503 {
    Obstacle::Obstacle(float min_x, float max_x, float min_y, float max_y, float robotScaleX, float robotScaleY) {
        this->min_x = min_x - robotScaleX;
        this->min_y = min_y - robotScaleY;
        this->max_x = max_x + robotScaleX;
        this->max_y = max_y + robotScaleY;
        this->obstacle_obstructed_radius = calculateObstacleRadius();
        this->center_x = (max_y + min_y)/2;
        this->center_y = (max_y + min_y)/2;
    }

    bool Obstacle::isWithinObstacle(Node *node) {
        return node->x >= min_x & node->x <= max_x & node->y >= min_y & node->y <= max_y;
    }

    bool Obstacle::edgeWithinObstacle(Node *node1, Node *node2) {
       float dx = node2->x - node1->x;
       float dy = node2->y - node1->y;
       float dcx = (center_x - node1->x);
       float dcy = (center_y - node1->y);
       double innerProduct = dcx *dx + dcy*dy;
       //check if the center projection on the line is within the line segment using inner product
       if (!(0 <= innerProduct && innerProduct <= dx*dx + dy*dy))
           return false;
       //if the center projection is on the line segment, then check the distance
       else {
           float theta = acos(innerProduct/(sqrt(dx*dx + dy*dy)*sqrt(dcx*dcx + dcy*dcy)));
           // minDistance = |cx| sin(theta)
           float minDistance = sqrt(dcx*dcx + dcy*dcy) * sin(theta);
           if (minDistance <= obstacle_obstructed_radius)
               return true;
           else
               return false;
       }
    }

    float Obstacle::calculateObstacleRadius() {
        return sqrt(pow((max_x - min_x)/2, 2) + pow((max_y - min_y)/2, 2));
    }
}