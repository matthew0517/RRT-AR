// @author Matthew Wallace
// No special compolation instructions.

#include <random>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
using namespace std;

struct Node {
    vector<float> cord;
    Node* parent;
    Node() {
        cord = {0.0,0.0};
        parent = NULL;
    }
    Node(float xOrigin, float yOrigin, Node* parental) {
        cord = {xOrigin,yOrigin};
        parent = parental;
    }
};

int add(int i, int j) {
    return i + j;
}

vector<vector<float>> runRRT(float initialX, float initialY, int maxNodesIn) {
    vector<Node> searchTree;
    Node origin = Node(initialX, initialY, NULL);
    vector<vector<float>> sol;
    sol.push_back(origin.cord);
    int count = 0;

    while (count < maxNodesIn) {
        
        count++;
    }


    return sol;
}

Node closest(vector<Node> searchTree, float x, float y) {
    float minDist = numeric_limits<float>::max();
    Node sol;
    for (Node elem : searchTree) {
        float dist = (elem.cord[0] - x)*(elem.cord[0] - x) + (elem.cord[1] - y)*(elem.cord[1] - y);
        if (dist < minDist) {
            sol = elem;
        }
    }
    return sol;
}


PYBIND11_MODULE(RRTAR, handle) {
    handle.doc() = "This is the module docs.";
    handle.def("add", &add, "A function that adds two numbers");
    handle.def("runRRT", &runRRT, "Sets the RRT origin");

}