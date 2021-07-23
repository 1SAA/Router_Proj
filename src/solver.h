#ifndef ROUTER_H
#define ROUTER_H

#include <string>
#include <queue>
#include <set>
#include <functional>

#include "grids.h"
#include "circuit.hpp"
#include "parser.h"
#include "router.h"

class Solver {

public:
    class OneMovement {
    public:
        std::string cellName;
        Point dest;
        OneMovement(std::string c, Point d): cellName(c), dest(d) {}
    };

private:
    GridDB DataBase;
    //std::vector<std::vector<Grid> > Circuit; /// map of the circuit
    std::vector<MasterCell> mCells; /// vector of master cells
    std::vector<CellInst> cInsts; /// vector of cell instances
    std::vector<Net> Nets; /// net
    std::vector<Layer> Layers; /// layers
    std::vector<float> powerPrefixSum; /// prefix sum of layers' power factors
    std::vector<VoltageArea> VAreas; /// voltage areas
    std::vector<int> Lock;

    std::function<bool(int, int)> comparator;

    std::set<int, decltype(comparator)> NetHeap; ///

    int maxCellMove, cntCellMove, numRow, numCol, numLayer;

    int numMaster, numNet, numInst;
    float totalCost;

    void canonicalize(std::vector<Segment> &route_seg);

    RouteInfo init2d(int netid);

    bool route2d(RouteInfo &info);

    bool route2pin2d(Edge &twopin);

    bool route_nets(std::vector<int> &ids);

    bool route3d(RouteInfo &info);

    int get3DCon(Point st, Point en);

    int get3DCon(Point g);

    void edge_dp(Edge &edge, int min_layer);

    void assign_layer_dp(Node &node, int min_layer);

    void assign_layer_compute_seg(Node &node, int curLayer, std::vector<Segment> &segs_3d);

    int assign_layer_for_edge(Edge &edge, int startLayer, std::vector<Segment> &segs_3d);

    int get2DCon(Point st, Point en);

    bool mazeRouting(Edge &twopin);

    int getLength(Net &net);

    void getBoundingBox(Net &net, Point &l, Point &r);

    void get2DBound(Net &net, Point &l, Point &r, std::string n);

    void addInst(CellInst &inst, Point pos, int c);

    bool checkInst(CellInst &inst, Point p);

    void moveInst(CellInst &inst, Point &dest);

    void incBlockage(CellInst &inst, Point pos, int sgn);

    Point findPosition(CellInst &inst, int posx, int posy);

    std::pair<std::vector<int>, std::vector<Point> > getMoveList();

    float calcCost(std::vector<Segment> &segs);
public:
    void run();

    Solver(Parser::ProblemInfo &problem);

    bool route_after_move(const std::vector<int>& insts, const std::vector<Point>& new_locs);

    void print_solution(std::ostream &os);

    void print();
};

#endif