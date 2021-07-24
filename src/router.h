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
    Edge(Node &p, Node &c, const std::vector<Segment> &s, const std::vector<DPInfo> &d) :
    parent(p), child(c), segs(s), dp_segs(d)
    {}
    
    Node &parent, &child;
    std::vector<Segment> segs; /// 2d routing segment from child to parent
    std::vector<DPInfo> dp_segs;
};

struct Node {
public:
    int x, y, hasPin, id;
    DPInfo dp;
    std::vector<Edge> neighbor;
    std::vector<int> layers;
    std::vector<std::vector<float> > intv_cost;
};

class RouteInfo {
public:
    int netid;
    STtree tree;
    std::vector<Edge*> edges;
    std::vector<Segment> segs_3d;
};

#endif