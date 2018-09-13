//
// Created by Sachith Danushka Withana on 9/5/18.
//

#ifndef RTT_GRIDSPACE_H
#define RTT_GRIDSPACE_H

#include "Node.h"
#include "Obstacle.h"

namespace e503 {
    /*
     * Assumption: The statespace is a cube. (length >= width)
     */
    class StateSpace {
    public:
        // defines the state space
        StateSpace(float min_x, float max_x, float min_y, float max_y);

        // generates a random c-space configuration in the state space
        Node * genRandomNodeInSpace();

        // check if a state configuration is obstructed
        bool isObstructed(Node *node);

        // add an obstacle to the statespace
        void addObstacle(Obstacle *obstacle);

        bool edgeIsObstructed(Node *nearestNode, Node *newNode);

    private:
        float min_x, max_x, min_y, max_y;

        std::vector<Obstacle *> obstacles;

        //generate random float
        float generateRandomNumber(float min, float max);

        //check if the node is within the statespace
        bool isWithinStateSpace(Node *node);

    };
}
#endif //RTT_GRIDSPACE_H
