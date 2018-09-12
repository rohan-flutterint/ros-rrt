//
// Created by Sachith Danushka Withana on 9/3/18.
//

#ifndef RTT_TREE_H
#define RTT_TREE_H

#include "Node.h"

namespace e503 {

    class RRT {
    public:
        RRT();

        void insert(Node *newNode, Node *leaf);

        void printTree(Node *leaf);

        Node *getNearestNeighbor(Node *randomNode);

        Node *extendNode(Node *currentNode, Node *randomNode, float epsilon);

        std::vector<Node *> extractShortestPath(Node *goal);

    private:
        Node *root;

        double getEuclideanDistance(Node *node1, Node *node2);
    };
}
#endif //RTT_TREE_H
