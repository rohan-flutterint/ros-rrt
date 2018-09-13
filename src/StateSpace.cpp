//
// Created by Sachith Danushka Withana on 9/5/18.
//
#include <stdlib.h>
#include <time.h>
#include "StateSpace.h"
#include <iostream>

namespace e503 {
    StateSpace::StateSpace(float min_x, float max_x, float min_y, float max_y) {
        this->min_x = min_x;
        this->min_y = min_y;
        this->max_x = max_x;
        this->max_y = max_y;
    }

    //todo: check if the node exists in the tree already
    Node * StateSpace::genRandomNodeInSpace(){
        return new Node(StateSpace::generateRandomNumber(min_y, max_y),
                StateSpace::generateRandomNumber(min_y, max_y), 0);
    }

    float StateSpace::generateRandomNumber(float min, float max) {
        return  (min + (rand() / (RAND_MAX/(max - min))));
    }

    bool StateSpace::isWithinStateSpace(Node *node) {
        return ((node->x >= min_x && node->x <= max_x) && (node->y >= min_y && node->y <= max_y));
    }

    bool StateSpace::isObstructed(Node *node) {
        if (!isWithinStateSpace(node))
            return true;
        else {
            for(int i = 0; i < obstacles.size(); ++i) {
                if (obstacles[i]->isWithinObstacle(node)) {
                    return true;
                }
            }
        }
        return false;
    }

    void StateSpace::addObstacle(Obstacle *obstacle) {
        obstacles.push_back(obstacle);
    }

    bool StateSpace::edgeIsObstructed(Node *nearestNode, Node *newNode) {
        for(int i = 0; i < obstacles.size(); ++i)    {
            if (obstacles[i]->edgeWithinObstacle(nearestNode, newNode)) {
                return true;
            }
        }
        return false;
    }

    std::vector<Node *> StateSpace::smoothenPath(std::vector<Node *> roughShortestPath) {
        std::vector<Node *> robotPath;
        int i = 0;
        int pathSize = roughShortestPath.size();
        // take the first node (start Node)
        Node *currentNode = roughShortestPath[i];
        while (i < pathSize - 1) {
            int j = i + 1;
            while (j < pathSize && !edgeIsObstructed(currentNode, roughShortestPath[j])) {
                j++;
            }
            robotPath.push_back(currentNode);
            currentNode = roughShortestPath[j - 1];
            i = j -1;
        }
        robotPath.push_back(currentNode);
        return robotPath;
    }
}