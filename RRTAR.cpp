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

struct Node {
    vector<float> cord;
    Node* parent;
    int pos;
    Node() {
        cord = {0.0,0.0};
        parent = NULL;
        pos = -1;
    }
    Node(float xOrigin, float yOrigin, Node* parental, int posIn) {
        cord = {xOrigin,yOrigin};
        parent = parental;
        pos = posIn;
    }
};

int add(int i, int j) {
    return i + j;
}

Node closest(vector<Node> searchTree, float x, float y) {
    float minDist = numeric_limits<float>::max();
    Node sol;
    for (Node elem : searchTree) {
        float dist = (elem.cord[0] - x)*(elem.cord[0] - x) + (elem.cord[1] - y)*(elem.cord[1] - y);
        if (dist < minDist) {
            minDist = dist;
            sol = elem;
        }
    }
    return sol;
}

// Author:ChatGPT
Node steer(float x, float y, float radius, Node node, int count) {
    // Calculate the distance between the current node's coordinates and the given coordinates
    float distance = sqrt(pow(x - node.cord[0], 2) + pow(y - node.cord[1], 2));

    if (distance <= radius) {
        // If the distance is within the radius, create a new node at the given coordinates without projection
        Node newNode = Node(x, y, &node, count);
        return newNode;
    } else {
        // Calculate the angle between the current node's coordinates and the projected coordinates
        float angle = atan2(y - node.cord[1], x - node.cord[0]);

        // Calculate the projected x and y coordinates on the circle
        float projectedX = node.cord[0] + radius * cos(angle);
        float projectedY = node.cord[1] + radius * sin(angle);

        // Create a new node with the projected coordinates and set its parent to the input node
        Node projectedNode = Node(projectedX, projectedY, &node, count);
        return projectedNode;
    }
}

vector<vector<float>> runRRT(float initialX, float initialY, int maxNodesIn, float rad) {

    vector<Node> searchTree;
    Node origin = Node(initialX, initialY, NULL, 0);
    searchTree.push_back(origin);
    vector<vector<float>> sol;
    int count = 0;

    while (count < maxNodesIn) {
        //std::srand(std::time(nullptr));

        // Generate two random floats between 0 and 100
        float x = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 100.0f;
        float y = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 100.0f;
        Node close = closest(searchTree, x, y);
        Node Xnew = steer(x,y,rad,close, count);
        searchTree.push_back(Xnew);
        count++;
    }

    for (Node nodes : searchTree) {
        sol.push_back(nodes.cord);
    }
    return sol;
}



PYBIND11_MODULE(RRTAR, handle) {
    handle.doc() = "This is the module docs.";
    handle.def("add", &add, "A function that adds two numbers");
    handle.def("runRRT", &runRRT, "Sets the RRT origin");

}