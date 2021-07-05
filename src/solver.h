#ifndef ROUTER_H
#define ROUTER_H

#include "grids.h"
#include "circuit.hpp"
#include "parser.h"

class Solver {
private:
    std::vector<std::vector<Grid> > Circuit; /// map of the circuit
    std::vector<MasterCell> mCells; /// vector of master cells
    std::vector<CellInst> cInsts; /// vector of cell instances
    std::vector<Net> Nets; /// net
    std::vector<Layer> Layers; /// layers
    std::vector<VoltageArea> VAreas; /// voltage areas
    int maxCellMove, cntCellMove, numRow, numCol, numLayer;
    int numMaster, numNet, numInst;
    void canonicalize(std::vector<Segment> &route_seg);
    void increase_congestion(int netid);
    void ripup(int netid);
    bool route_two_pin(Point st, Point en, int min_layer, std::vector<Segment> &seg);
    bool route_net(int netid);
public:
    Solver(Parser::ProblemInfo &problem);
    bool route_after_move(const std::vector<int>& insts, const std::vector<Point>& new_locs);
    void print_solution(std::ostream &os);
    void print();
};

#endif