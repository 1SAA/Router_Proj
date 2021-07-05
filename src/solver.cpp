#include <string>
#include <map>
#include <unordered_set>
#include <iostream>
#include "solver.h"
#include "flute.h"

/**
 * @brief Init solver from ProblemInfo
 */
Solver::Solver(Parser::ProblemInfo &problem) {
    maxCellMove = problem.maxCellMove;
    numRow = problem.gGridX;
    numCol = problem.gGridY;
    numLayer = problem.numLayer;
    numMaster = problem.masters.size();
    numInst = problem.insts.size();
    numNet = problem.nets.size();
    

    std::map<std::string, int> name2layer;
    std::map<std::string, int> name2master;
    std::map<std::string, int> name2inst;
    std::map<std::string, int> name2VA;
    std::map<std::string, int> name2net;

    /// Voltage Areas
    int i = 1;
    VAreas.resize(problem.areas.size() + 1);
    for (auto x : problem.areas) {
        auto &v = VAreas[i];
        name2VA[x.name] = i;
        v = VoltageArea();
        for (auto g : x.grids)
            v.area.push_back(Point(g.first, g.second, 0));
        i++;
    }

    /// Layers
    Layers.resize(numLayer + 1);
    for (auto x : problem.layers) {
        int id = x.idx;
        Layers[id] = Layer(x.name, x.idx, x.direction, x.defSupply, x.powerFactor);
        name2layer[x.name] = id;
    }

    /// Master Cells
    mCells.resize(numMaster);
    i = 0;
    for (auto &x : problem.masters) {
        auto &master = mCells[i];
        name2master[x.name] = i;
        master = MasterCell(x.name, x.cntPin, x.cntBlockage);
        int j = 0;
        for (auto p : x.pins) {
            master.pins[j] = Pin(p.name, name2layer[p.layer]);
            master.s2pin[p.name] = j;
            j++;
        }
        j = 0;
        for (auto b : x.blockages) {
            master.blocks[j] = Blockage(b.name, name2layer[b.layer], b.demand);
            master.s2block[b.name] = j;
        }
        i++;
    }

    /// Cell Instances

    cInsts.resize(numInst);
    i = 0;
    for (auto &x : problem.insts) {
        auto &inst = cInsts[i];
        inst = CellInst(x.name, Point(x.gX, x.gY, 0), name2master[x.masterName], 
                        x.moveConstr, 0);
        name2inst[x.name] = i;
        i++;
    }

    for (auto &x : problem.areas) {
        int idArea = name2VA[x.name];
        for (auto &inst : x.instNames) {
            int idInst = name2inst[inst];
            VAreas[idArea].instList.push_back(idInst);
            cInsts[idInst].voltageLabel = idArea;
        }
    }

    /// Init Grid
    Circuit.resize(numRow + 1, std::vector<Grid>(numCol + 1));
    for (int i = 1; i <= numRow; ++i) {
        for (int j = 1; j <= numCol; ++j) {
            Circuit[i][j] = Grid({i, j}, numLayer + 1, -1);
            for (int k = 1; k <= numLayer; ++k)
                Circuit[i][j][k].supply = Layers[k].supply;
        }
    }
    
    for (int i = 0, n = VAreas.size(); i < n; ++i) {
        for (auto p : VAreas[i].area) {
            Circuit[p.x][p.y].getVoltageLabel() = i;
        }
    }

    for (auto &p : problem.supplyGrids) 
        Circuit[p.gX][p.gY][p.layer].supply += p.diff;

    /// Nets and Segments
    i = 0;
    Nets.resize(problem.nets.size());
    for (auto &x : problem.nets) {
        name2net[x.name] = i;
        auto &net = Nets[i];
        net = Net(x.name, x.cntPin, x.minRoutingLayConstr, x.weight);
        for (int j = 0, n = x.pins.size(); j < n; ++j) {
            int inst = name2inst[x.pins[j].instName];
            int master = cInsts[inst].masterCell;
            int pin = mCells[master].s2pin[x.pins[j].pinName];
            net[j] = {inst, pin, mCells[master].pins[pin].layerNum};
            cInsts[inst].NetIds.push_back(i);
        }
        i++;
    }

    i = 0;
    for (auto &x : problem.routes) {
        int netid = name2net[x.netName];
        Nets[netid].segments.push_back(Segment(Point(x.sX, x.sY, x.sL), Point(x.eX, x.eY, x.eL)));
        // TODO Canonicalize routing segments
        // Routing Segments may have intersections
        /*
            canonicalize(Nets[netid].segments);
        */
    }

    /// Update Demand
    for (int i = 0, n = Nets.size(); i < n; ++i)
        increase_congestion(i);
}

/**
 * @brief call this function after reroute to increase congestion
 * @param netid id of the rerouted net
 */
void Solver::increase_congestion(int netid) {

}

/**
 * @brief ripup a net and decrease congestion
 * @param netid id of the ripuped net
 */
void Solver::ripup(int netid) {

}

/**
 * @brief remove routing segment intersections
 * @param route_seg reference to routing segments of a net
 *                  call this function may change this parameter
 * @return void
 */
void Solver::canonicalize(std::vector<Segment> &route_seg) {

}

/**
 * @brief route 2-pin net
 * @param st start point
 * @param en end point
 * @param seg reference to the vector used to store the routing result
 * @return bool 
 */
bool Solver::route_two_pin(Point st, Point en, int min_layer, std::vector<Segment> &seg) {
    return false;
}

static void explore(Tree &t) {
    dbg_print("print Steiner Tree\n");
    dbg_print("====================\n");
    int num_vertex = sizeof(t.branch);
    for (int i = 0; i < num_vertex; ++i) {
        if (i != t.branch[i].n)
            dbg_print("%d --> %d\n", i, t.branch[i].n);
        else
            dbg_print("root: %d\n", i);
    }
    dbg_print("=====================\n");
}

/**
 * @brief route a net at best effort
 * @param netid id of the net to be routed
 * @return bool route successful or not
 */
bool Solver::route_net(int netid) {
    /// call flute, using acc=3
    int deg = Nets[netid].pins.size();
    int *xs = new DTYPE [deg];
    int *ys = new DTYPE [deg];

    for (int i = 0; i < deg; ++i) {
        int inst = Nets[netid].pins[i].inst;
        xs[i] = cInsts[inst].position.x;
        ys[i] = cInsts[inst].position.y;
    }

    const int FLUTE_ACC = 3;
    Tree t = flute(deg, xs, ys, FLUTE_ACC);
    DEBUG(explore(t));

    /// layer assignment
    
    /// two-pin net routing

    delete xs;
    delete ys;

    return false;
}

/**
 * @brief reroute after cell movement
 * @param insts id of cell instances
 * @param new_locs new locations of these cell instances
 * @return reroute is successful or not
 */
bool Solver::route_after_move(const std::vector<int> &insts, const std::vector<Point> &new_locs) {
    /// find nets that are affected
    std::unordered_set<int> changed_net;
    for (int i = 0, n = insts.size(); i < n; ++i) {
        int id = insts[i];
        if (cInsts[id].position == new_locs[i])
            continue;
        if (cInsts[id].movedFlag == 0) {
            cInsts[id].movedFlag = 1;
            cntCellMove++;
        }
        for (auto &net_id : cInsts[id].NetIds) 
            changed_net.insert(net_id);
    }

    /// update blockage
    for (int i = 0, n = insts.size(); i < n; ++i) {
        int id = insts[i];
        int master_id = cInsts[id].masterCell;
        int x = cInsts[id].position.x, y = cInsts[id].position.y;
        for (auto &b : mCells[master_id].blocks) {
            Circuit[x][y][b.layerNum].demand -= b.demand;
            Circuit[new_locs[i].x][new_locs[i].y][b.layerNum].demand += b.demand;
        }
    }

    /// ripup these nets
    for (auto &x : changed_net)
        ripup(x);
    /// reroute these nets
    for (auto &x : changed_net) /// TODO: in which order to route these nets
        route_net(x);

    return false;
}

void Solver::print_solution(std::ostream &os) {
    os << "NumMovedCellInst " << cntCellMove << "\n";
    for (auto &inst : cInsts) 
        if (inst.movedFlag) {
            os << "CellInst " << inst.name << " ";
            os << inst.position.x << " " << inst.position.y << "\n";
        }
    int cntRoutes = 0;
    for (auto &net : Nets)
        cntRoutes += net.segments.size();
    os << "NumRoutes " << cntRoutes << "\n";
    for (auto &net : Nets)
        for (auto &seg : net.segments) {
            os << seg.spoint.x << " " << seg.spoint.y << " " << seg.spoint.z << " ";
            os << seg.epoint.x << " " << seg.epoint.y << " " << seg.epoint.z << " ";
            os << net.name << "\n";
        }
}

void Solver::print() {
    printf("Information about current stage\n");
    printf("=================================\n");
    printf("maxCellMove: %d, numRow: %d, numCol: %d, numLayer, %d\n", maxCellMove, numRow, numCol, numLayer);
    printf("\n");
    printf("MasterCells: %d\n", numMaster);
    for (int i = 0; i < numMaster; ++i) {
        std::cout << mCells[i].name << ": " << i << "\n";
        printf("Pins %d:", mCells[i].pinCount);
        for (auto &p : mCells[i].pins)
            printf(" %d", p.layerNum);
        printf("\n");
        printf("Blockage %d:", mCells[i].blockageCount);
        for (auto &b : mCells[i].blocks)
            printf(" (%d, %d)", b.layerNum, b.demand);
        printf("\n");
    }
    
    printf("\n");
    printf("CellInsts: %d\n", numInst);
    for (int i = 0; i < numInst; ++i) {
        std::cout << cInsts[i].name << ": " << i << "\n";
        printf("Coordinate: (%d, %d)\n", cInsts[i].position.x, cInsts[i].position.y);
        printf("Master: %d, Voltage %d, Movable %d\n", cInsts[i].masterCell, 
                cInsts[i].voltageLabel, cInsts[i].movable);
        printf("\n");
    }

    printf("\n");
    printf("Nets: %d\n", numNet);
    for (int i = 0; i < numNet; ++i) {
        std::cout << Nets[i].name << ": " << i << "\n";
        printf("MinLayer: %d, Weight: %f", Nets[i].minConstraint, Nets[i].weight);
        printf("pins: %lu\n", Nets[i].pins.size());
        for (auto &pin : Nets[i].pins)
            printf("(%d, %d, %d)\n", cInsts[pin.inst].position.x, cInsts[pin.inst].position.y, pin.layer);
        printf("Segments: %lu\n", Nets[i].segments.size());
        for (auto &seg : Nets[i].segments) 
            std::cout << seg.spoint << " -- " << seg.epoint << "\n";
        printf("\n");
    }
    printf("=================================\n");
}