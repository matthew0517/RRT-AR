// @author Matthew Wallace
// No special compolation instructions.

#include <random>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
using namespace std;
vector<vector<float>> goals;
float rad;
float closePenalty;
// Function to check if a point is within r of the goal. 
int checkInGoal(float x, float y) {
    for (int i =0; (unsigned) i < goals.size(); i++) {
        vector<float> elem = goals[i];
        float dist = (elem[0] - x)*(elem[0] - x) + (elem[1] - y)*(elem[1] - y);
        if (dist < rad) {
            return i+1;
        }
    }
    return 0;
}

struct Node {
    vector<float> cord;
    int parent;
    int pos;
    int inGoal;
    float cost;
    Node() {
        cord = {0.0,0.0};
        parent = -1;
        pos = -100;
        inGoal = 0;
        cost = 0.0;
    }
    Node(float xOrigin, float yOrigin, int parental, int posIn, int inGoalIn, float costIn) {
        cord = {xOrigin,yOrigin};
        parent = parental;
        pos = posIn;
        inGoal = inGoalIn;
        cost = costIn;
    }
};

int add(int i, int j) {
    return i + j;
}

Node closest(vector<Node> const &searchTree, float x, float y) {
    float minDist = numeric_limits<float>::max();
    Node sol;
    for (Node elem : searchTree) {
        float dist = (elem.cord[0] - x)*(elem.cord[0] - x) + (elem.cord[1] - y)*(elem.cord[1] - y);
        if (dist < minDist && !elem.inGoal) {
            minDist = dist;
            sol = elem;
        }
    }
    return sol;
}

// Author:ChatGPT
Node steer(float x, float y, Node node, int count) {
    // Calculate the distance between the current node's coordinates and the given coordinates
    float distance = sqrt(pow(x - node.cord[0], 2) + pow(y - node.cord[1], 2));

    if (distance <= rad) {
        // If the distance is within the radius, create a new node at the given coordinates without projection
        Node newNode = Node(x, y, node.pos, count, checkInGoal(x, y), distance);
        return newNode;
    } else {
        // Calculate the angle between the current node's coordinates and the projected coordinates
        float angle = atan2(y - node.cord[1], x - node.cord[0]);

        // Calculate the projected x and y coordinates on the circle
        float projectedX = node.cord[0] + rad * cos(angle);
        float projectedY = node.cord[1] + rad * sin(angle);

        // Create a new node with the projected coordinates and set its parent to the input node
        Node projectedNode = Node(projectedX, projectedY, node.pos, count, checkInGoal(projectedX, projectedY), rad);
        return projectedNode;
    }
}

float calcCost(Node input, vector<Node> const &searchTree) {
    float ans = input.cost;
    while (input.parent > -1) {
        input = searchTree[input.parent];
        ans += input.cost;
    }
    return ans;
}

void rewire(vector<Node> &searchTree, int count) {
    vector<int> withRad;
    vector<float> costs;
    vector<float> dists;
    vector<int> parents;
    vector<int> indexOfNodes;

    float x = searchTree[count].cord[0];
    float y = searchTree[count].cord[1];
    // Initial search- normal RRT*
    for (int i = 0; i < count; i++) {
        float dist = sqrt((searchTree[i].cord[0] - x)*(searchTree[i].cord[0] - x) + (searchTree[i].cord[1] - y)*(searchTree[i].cord[1] - y));
        if (dist <= 1.1*rad) {
            withRad.push_back(i);
            float cost = calcCost(searchTree[i], searchTree);
            costs.push_back(cost);
            dists.push_back(dist);
            parents.push_back(searchTree[i].parent);
            indexOfNodes.push_back(i);
        }
    }

    // This add parent penalty for close neighbor (alg 1 or RRT*-AR)
    vector<float> costRewire = costs;
    for (int i = 0; (unsigned) i < costs.size(); i++) {
        int parent = parents[i];
        int foundIndex = distance(indexOfNodes.begin(), find(indexOfNodes.begin(), indexOfNodes.end(), parent));
        if ((unsigned) foundIndex != costs.size()) {
            costRewire[foundIndex] += closePenalty;
        } 
    }

    int minNeighbor = -1;
    float minCost =  numeric_limits<float>::max();
    for (int i = 0; (unsigned) i < costs.size(); i++) {
        if (costRewire[i] + dists[i] < minCost) {
            minCost = costRewire[i] + dists[i];
            minNeighbor = withRad[i];
        } 
    }

    searchTree[count].parent = minNeighbor;
    vector<int> permutation(costs.size());
    iota(permutation.begin(), permutation.end(), 0);
    sort(permutation.begin(), permutation.end(),    [&](int A, int B) -> bool {
                return costs[A] < costs[B];});

    for (int i = 0; (unsigned) i < costs.size(); i++) {
        if (minCost + dists[permutation[i]] < costs[permutation[i]]) {
            searchTree[withRad[permutation[i]]].parent = count;
            minCost += closePenalty;
        }
    }
}

vector<vector<vector<float>>> runRRT(float initialX, float initialY, int maxNodesIn, float radIn, float closePenaltyIn, vector<vector<int>> og, vector<vector<float>> goalsIn) {
    goals = goalsIn;
    rad = radIn;
    closePenalty = closePenaltyIn;
    vector<Node> searchTree;
    Node origin = Node(initialX, initialY, -1, 0, false, 0);
    searchTree.push_back(origin);
    vector<vector<vector<float>>> sol;
    int count = 1;

    while (count < maxNodesIn) {
        //std::srand(std::time(nullptr));

        // Generate two random floats between 0 and 100
        float x = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 100.0f;
        float y = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 100.0f;
        if (og[static_cast<int>(x)][static_cast<int>(y)]) {
            continue;
        }
        Node close = closest(searchTree, x, y);

        Node Xnew = steer(x,y,close, count);
        if (og[static_cast<int>(Xnew.cord[0])][static_cast<int>(Xnew.cord[1])]) {
            continue;
        }
        searchTree.push_back(Xnew);
        rewire(searchTree, count);
        count++;
    }

    // Adds every branch in the vector
    vector<vector<float>> fulltree;
    for (Node nodes : searchTree) {
        vector<float> temp = nodes.cord;
        if (nodes.parent >= 0) {
            temp.push_back(static_cast<float>(nodes.parent));
        } else {
            temp.push_back(-1.0f);
        }
        fulltree.push_back(temp);
    }
    sol.push_back(fulltree);

    // Adds branches that connect to the goal
    for (Node nodes : searchTree) {
        if (nodes.inGoal > 0) {
            int parentIndex = nodes.parent;
            vector<vector<float>> solBranch;
            vector<float> temp = nodes.cord;
            solBranch.push_back(goals[nodes.inGoal-1]);
            solBranch.push_back(temp);
            while (parentIndex >= 0) {
                Node nodesInner = searchTree[parentIndex];
                temp = nodesInner.cord;
                solBranch.push_back(temp);
                parentIndex = nodesInner.parent;
            }
            reverse(solBranch.begin(), solBranch.end());
            sol.push_back(solBranch);
        }
    }
    return sol;
}

PYBIND11_MODULE(RRTAR, handle) {
    handle.doc() = "This is the module docs.";
    handle.def("add", &add, "A function that adds two numbers");
    handle.def("runRRT", &runRRT, "Sets the RRT origin");

}