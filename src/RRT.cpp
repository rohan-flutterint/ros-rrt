#include <cstdio>
#include "Node.h"
#include "RRT.h"
#include <cmath>
#include <limits>
#include <iostream>

//
// Created by Sachith Danushka Withana on 9/3/18.
//
namespace e503 {

    RRT::RRT() {
        root = NULL;
    };

    void RRT::insert(Node *newNode, Node *leaf) {
        if (root != NULL) {
            leaf->children.push_back(newNode);
            newNode->parent = leaf;
        } else {
            root = newNode;
        }
    }

    void RRT::printTree(Node *leaf) {
        if (leaf == NULL) {
            return;
        } else {
            printf("Node: (%.1f,%.1f,%.1f)\n", leaf->x, leaf->y, leaf->theta);
            int i = 0;
            for (i = 0; i < leaf->children.size(); i++) {
                printTree(leaf->children.at(i));
            }
        }
    }

    Node * RRT::getNearestNeighbor(Node *randomNode) {
        Node *closestNode = NULL;
        if (root != NULL) {
            // initialize the first node to be root
            closestNode = root;
            double minDistance = std::numeric_limits<double>::max();
            std::vector<Node *> treeNodes;
            treeNodes.push_back(root);

            /*
             * Calculate the euclidean distance for each node in the tree and get the min
             * This is done in a breadth-first manner.
            */
            Node *currentNode = NULL;
            double currentNodeDistance = minDistance;
            while (!treeNodes.empty()) {
                currentNode = treeNodes.front();
                currentNodeDistance = getEuclideanDistance(currentNode, randomNode);
                /*printf("Current Node: (%.2f, %.2f, %.2f) and the distance from Random Node: %f\n", currentNode->x,
                       currentNode->y, currentNode->theta, currentNodeDistance);*/
                if (minDistance > currentNodeDistance) {
                    minDistance = currentNodeDistance;
                    closestNode = currentNode;
                }
                // insert the children of that node to the search list
                treeNodes.insert(treeNodes.end(), currentNode->children.begin(), currentNode->children.end());
                // delete the processed node
                treeNodes.erase(treeNodes.begin());
            }
        }
        return closestNode;
    }

    double RRT::getEuclideanDistance(Node *node1, Node *node2) {
        double powerTotal = pow(node1->x - node2->x, 2) + pow(node1->y - node2->y, 2);
        return sqrt(powerTotal);
    }

    Node * RRT::extendNode(Node *currentNode, Node *randomNode, float epsilon) {
        float vectorGradient = tanf((randomNode->y - currentNode->y)/(randomNode->x - currentNode->x));
        float x = currentNode->x + epsilon * cos(vectorGradient);
        float y = currentNode->y + epsilon * sin(vectorGradient);
        //todo: change this
        return new Node(x, y, 0);
    }

}
