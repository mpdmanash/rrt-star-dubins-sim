#include "rrtstar.h"

RRTSTAR::RRTSTAR()
{
    obstacles = new Obstacles;
    startPos.x() = START_POS_X;
    startPos.y() = START_POS_Y;
    endPos.x() = END_POS_X;
    endPos.y() = END_POS_Y;
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->cost = 0.0;
    lastNode = root;
    nodes.push_back(root);
    step_size = 3;
    max_iter = 3000;
}

/**
 * @brief Initialize root node of RRTSTAR.
 */
void RRTSTAR::initialize()
{
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->cost = 0.0;
    lastNode = root;
    nodes.push_back(root);
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRTSTAR::getRandomNode()
{
    Node* ret;
    Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
    if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT) {
        ret = new Node;
        ret->position = point;
        return ret;
    }
    return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRTSTAR::distance(Vector2f &p, Vector2f &q)
{
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRTSTAR::nearest(Vector2f point)
{
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

/**
 * @brief Get neighborhood nodes of a given configuration/position.
 * @param point
 * @param radius
 * @param out_nodes
 * @return
 */
void RRTSTAR::near(Vector2f point, float radius, vector<Node *>& out_nodes)
{
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < radius) {
            out_nodes.push_back(nodes[i]);
        }
    }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
Vector2f RRTSTAR::newConfig(Node *q, Node *qNearest)
{
    Vector2f to = q->position;
    Vector2f from = qNearest->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size * intermediate;
    return ret;
}

/**
 * @brief Return trajectory cost.
 * @param q
 * @return
 */
double RRTSTAR::Cost(Node *q)
{
    return q->cost;
}

/**
 * @brief Compute path cost.
 * @param qFrom
 * @param qTo
 * @return
 */
double RRTSTAR::PathCost(Node *qFrom, Node *qTo)
{
    return distance(qTo->position, qFrom->position);
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRTSTAR::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNew->cost = qNearest->cost + PathCost(qNearest, qNew);
    qNearest->children.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRTSTAR::reached()
{
    if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD)
        return true;
    return false;
}

void RRTSTAR::setStepSize(int step)
{
    step_size = step;
}

void RRTSTAR::setMaxIterations(int iter)
{
    max_iter = iter;
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRTSTAR::deleteNodes(Node *root)
{
    for(int i = 0; i < (int)root->children.size(); i++) {
        deleteNodes(root->children[i]);
    }
    delete root;
}
