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

    std::vector<Node *> StateSpace::smoothenPath(std::vector<Node *> roughShortestPath, float interpolEpsilon) {
        std::vector<Node *> robotPath;
        // assumes the goal is at the back
        // going from goal up
        Node *firstNode = roughShortestPath.back();
        Node *prev = firstNode->parent;
        Node *next = prev->parent;
        robotPath.push_back(firstNode);
        while (next != 0) {
            if (edgeIsObstructed(firstNode, next)) {
                robotPath.push_back(prev);
                firstNode = prev;
                prev = prev->parent;
                next = prev->parent;
            }
            else {
                prev = next;
                next = next->parent;
                firstNode->parent = prev;
            }
        }
        // adding the start
        robotPath.push_back(roughShortestPath.front());
        std::reverse(robotPath.begin(), robotPath.end());
        std::cout<< "printing the robotPath\n";
        for (int i = 0; i < robotPath.size(); ++i) {
            robotPath.at(i)->printNode();
        }
        return generateInterploatedPath(robotPath.back(), interpolEpsilon);
    }

    std::vector<Node *> StateSpace::generateInterploatedPath(Node *goalNode, float interpolEpsilon) {
        std::vector<Node *> interpolatedPath;
        Node *currentNode = goalNode;
        while (currentNode->parent != 0) {
            interpolatedPath.push_back(currentNode);
            Node *parentNode = currentNode->parent;

            Node *n1 = new Node(parentNode->x + 0.25 * (currentNode->x - parentNode->x),
                    parentNode->y + 0.25 * (currentNode->y - parentNode->y), 0 );
            currentNode->parent = n1;
            Node *n2 = new Node(parentNode->x + 0.5 * (currentNode->x - parentNode->x),
                                parentNode->y + 0.5 * (currentNode->y - parentNode->y), 0 );

            Node *n3 = new Node(parentNode->x + 0.75 * (currentNode->x - parentNode->x),
                                parentNode->y + 0.75 * (currentNode->y - parentNode->y), 0 );

            currentNode->parent = n3;
            n3->parent = n2;
            n2->parent = n1;
            n1->parent = parentNode;
            interpolatedPath.push_back(n3);
            interpolatedPath.push_back(n2);
            interpolatedPath.push_back(n1);

            currentNode = parentNode;
        }
        interpolatedPath.push_back(currentNode);
        std::reverse(interpolatedPath.begin(), interpolatedPath.end());
        return interpolatedPath;
    }
}