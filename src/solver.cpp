#include <string>
#include <map>
#include <unordered_set>
#include <iostream>
#include <algorithm>
#include <list>
#include "solver.h"

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
    powerPrefixSum.resize(numLayer + 1, 0.0);
    for (int i = 1; i <= numLayer; ++i)
        powerPrefixSum[i] = powerPrefixSum[i - 1] + Layers[i].powerFactor;
        
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

    // Init Grid DataBase    
    DataBase = GridDB(numRow, numCol, numLayer);
    for (int i = 1; i <= numRow; ++i) {
        for (int j = 1; j <= numCol; ++j) {
            for (int k = 1; k <= numLayer; ++k)
                DataBase.getSupply({i, j, k}) = Layers[k].supply;
        }
    }
    for (int i = 0, n = VAreas.size(); i < n; ++i) {
        for (auto p : VAreas[i].area) {
            DataBase.getVoltageLable({p.x, p.y}) = i;
        }
    }

    for (auto &p : problem.supplyGrids) 
        DataBase.getSupply({p.gX, p.gY, p.layer}) += p.diff;

    // Blockages
    for (auto &c : cInsts) {
        auto m = mCells[c.masterCell];
        for (auto &b : m.blocks) 
            DataBase.incDemand({c.position.x, c.position.y, b.layerNum}, b.demand);
    }

    /// Nets and Segments
    i = 0;
    Nets.resize(problem.nets.size());
    for (auto &x : problem.nets) {
        name2net[x.name] = i;
        auto &net = Nets[i];
        int minLayer;
        if (x.minRoutingLayConstr == "NoCstr")
            minLayer = 1;
        else 
            minLayer = name2layer[x.minRoutingLayConstr];
        net = Net(x.name, x.cntPin, minLayer, x.weight);
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
        canonicalize(Nets[netid].segments);
        DataBase.inc3DCon(Nets[netid].segments, 1);
    }
    DataBase.get2DFrom3D();
}

/**
 * @brief remove routing segment intersections. Intersections between H/V-segments and Z-segments
 *        are not eliminated.
 * @param route_seg reference to routing segments of a net
 *                  call this function may change this parameter
 * @return void
 */
void Solver::canonicalize(std::vector<Segment> &route_seg) {
    std::list<Segment> segList;

    for (auto &seg : route_seg) {
        if (seg.spoint == seg.epoint)
            continue;
        segList.push_back(seg);
    }

    route_seg.clear();
    for (auto iter = segList.begin(); iter != segList.end(); ) {
        Segment curSeg = *iter;
        auto nxt = iter;
        nxt++;
        while (nxt != segList.end()) {
            if (Segment::isIntersect(curSeg, *nxt)) {
                *nxt = Segment::Merge(curSeg, *nxt);
                break;
            } else 
                nxt++;
        }
        segList.erase(++iter);
        route_seg.push_back(curSeg);
    }
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
 * @brief route nets, first do 2d routing, then do layer assignment
 * @param ids nets that needs to be routed
 * @return bool route successful or not
 */
bool Solver::route_nets(std::vector<int> &ids) {
    /// init 2d routing info
    /// TODO restore changes if failed

    std::vector<RouteInfo> r2d;
    for (auto &id : ids) 
        r2d.push_back(init2d(id));
    
    /// do 2d routing
    std::vector<std::vector<Segment> > backup;
    for (auto &info : r2d) {
        if (route2d(info) == false) {
            for (auto &x : backup)
                DataBase.inc2DCon(x, 1);
            return false;
        }
        std::vector<Segment> segs_2d;
        for (auto edge : info.edges)
            segs_2d.insert(segs_2d.begin(), edge->segs.begin(), edge->segs.end());
        canonicalize(segs_2d);
        DataBase.inc2DCon(segs_2d, -1);
        backup.push_back(segs_2d);
    }
    
    backup.clear();
    /// 3d routing, i.e. layer assignment
    for (auto &info : r2d) {
        if (route3d(info) == false) {
            for (auto &x : backup)
                DataBase.inc3DCon(x, 1);
            return false;
        }
        canonicalize(info.segs_3d);
        DataBase.inc3DCon(info.segs_3d, -1);
        backup.push_back(info.segs_3d);
    }
    for (auto &info : r2d)
        Nets[info.netid].segments = info.segs_3d;
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
    std::vector<int> changed_net;
    for (int i = 0, n = insts.size(); i < n; ++i) {
        int id = insts[i];
        if (cInsts[id].position == new_locs[i])
            continue;
        if (cInsts[id].movedFlag == 0) {
            cInsts[id].movedFlag = 1;
            cntCellMove++;
        }
        for (auto &net_id : cInsts[id].NetIds) 
            changed_net.push_back(net_id);
    }

    std::sort(changed_net.begin(), changed_net.end());
    changed_net.end() = std::unique(changed_net.begin(), changed_net.end());

    /// update blockage
    for (int i = 0, n = insts.size(); i < n; ++i) {
        int id = insts[i];
        int master_id = cInsts[id].masterCell;
        int x = cInsts[id].position.x, y = cInsts[id].position.y;
        for (auto &b : mCells[master_id].blocks) {
            DataBase.incDemand({x, y, b.layerNum}, -b.demand);
            DataBase.incDemand({new_locs[i].x, new_locs[i].y, b.layerNum}, b.demand);
        }
    }

    /// ripup these nets
    for (auto &x : changed_net) {
        DataBase.inc3DCon(Nets[x].segments, 1);
        DataBase.inc2DCon(Nets[x].segments, 1);
    }
    
    /// reroute these nets
    if (!route_nets(changed_net)) {
        for (auto &x : changed_net) {
            DataBase.inc3DCon(Nets[x].segments, -1);
            DataBase.inc2DCon(Nets[x].segments, -1);
        }
        return false;
    }
    return true;
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