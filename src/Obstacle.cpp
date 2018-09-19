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
      float d1 = (node2->y - node1->y)*min_x + (node1->x - node2->x)*min_y + (node2->x * node1->y - node2->y * node1->x);
      float d2 = (node2->y - node1->y)*min_x + (node1->x - node2->x)*max_y + (node2->x * node1->y - node2->y * node1->x);
      float d3 = (node2->y - node1->y)*max_x + (node1->x - node2->x)*min_y + (node2->x * node1->y - node2->y * node1->x);
      float d4 = (node2->y - node1->y)*max_x + (node1->x - node2->x)*max_y + (node2->x * node1->y - node2->y * node1->x);

      // brushes the obstacle
      if (d1 == 0 || d2 == 0 || d3 == 0 || d4 == 0)
          return true;
      else if (d1 < 0 & d2 < 0 & d3 < 0 & d4 < 0)
          return false;
      else if (d1 > 0 & d2 > 0 & d3 > 0 & d4 > 0)
          return false;
      else if (node1->x < min_x & node2->x < min_x)
          return false;
      else if (node1->y < min_y & node2->y < min_y)
          return false;
      else if (node1->y > max_y & node2->y > max_y)
          return false;
      else if (node1->x > max_x & node2->x > max_x)
          return false;
      else
          return true;
    }

    float Obstacle::calculateObstacleRadius() {
        return sqrt(pow((max_x - min_x)/2, 2) + pow((max_y - min_y)/2, 2));
    }
}