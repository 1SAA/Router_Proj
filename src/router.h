#ifndef __ROUTER_H__
#define __ROUTER_H__
#include <vector>
#include "flute.h"
#include "circuit.hpp"

struct Node;
class DPInfo;
class Edge;

struct STtree {
public:
    int root;
    std::vector<Node> nodes;  
};

struct DPInfo {
public:
    std::vector<float> cost; /// dp value
    std::vector<int> prev; /// dp flag array
};

class Edge {
public:
    Node &parent, &child;
    std::vector<Segment> segs; /// 2d routing segment from child to parent
    std::vector<DPInfo> dp_segs;
};

struct Node {
public:
    int x, y, layer, isPin, id;
    DPInfo dp;
    std::vector<Edge> neighbor;
};

class RouteInfo {
public:
    int netid;
    STtree tree;
    std::vector<Edge*> edges;
    std::vector<Segment> segs_3d;
};

#endif