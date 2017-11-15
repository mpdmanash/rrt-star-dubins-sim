#ifndef RRTSTAR_H
#define RRTSTAR_H

#include "obstacles.h"
#include <stdlib.h>
#include <vector>
#include <math.h>

using namespace std;
using namespace Eigen;

struct Node {
    vector<Node *> children;
    Node *parent;
    Vector2f position;
    double cost;
};

class RRTSTAR
{
public:
    RRTSTAR();
    void initialize();
    Node* getRandomNode();
    Node* nearest(Vector2f point);
    void near(Vector2f point, float radius, vector<Node *>& out_nodes);
    double distance(Vector2f &p, Vector2f &q);
    double Cost(Node *q);
    double PathCost(Node *qFrom, Node *qTo);
    Vector2f newConfig(Node *q, Node *qNearest);
    void add(Node *qNearest, Node *qNew);
    bool reached();
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void deleteNodes(Node *root);
    Obstacles *obstacles;
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector2f startPos, endPos;
    int max_iter;
    int step_size;
};

#endif // RRTSTAR_H
