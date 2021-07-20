#ifndef ROUTER_H
#define ROUTER_H

#include "grids.h"
#include "circuit.hpp"
#include "parser.h"
#include "router.h"

class Solver {
private:
    std::vector<std::vector<Grid> > Circuit; /// map of the circuit
    std::vector<MasterCell> mCells; /// vector of master cells
    std::vector<CellInst> cInsts; /// vector of cell instances
    std::vector<Net> Nets; /// net
    std::vector<Layer> Layers; /// layers
    std::vector<float> powerPrefixSum; /// prefix sum of layers' power factors
    std::vector<VoltageArea> VAreas; /// voltage areas
    int maxCellMove, cntCellMove, numRow, numCol, numLayer;
    int numMaster, numNet, numInst;
    void canonicalize(std::vector<Segment> &route_seg);
    void increase_congestion(int netid);
    void ripup(int netid);
    RouteInfo init2d(int netid);
    bool route2d(RouteInfo &info);
    bool route2pin2d(Edge &twopin);
    bool route_nets(std::vector<int> &ids);
    bool route3d(RouteInfo &info);
    int get3DCon(Point st, Point en);
    int get3DCon(Point p);
    void edge_dp(Edge &edge, int min_layer);
    int assign_layer_for_edge(Edge &edge, int startLayer, std::vector<Segment> &segs_3d);
    void assign_layer_dp(Node &node, int min_layer);
    void assign_layer_compute_seg(Node &node, int curLayer, std::vector<Segment> &segs_3d);
    int get2DCon(Point st, Point en);
public:
    Solver(Parser::ProblemInfo &problem);
    bool route_after_move(const std::vector<int>& insts, const std::vector<Point>& new_locs);
    void print_solution(std::ostream &os);
    void print();
};

#endif