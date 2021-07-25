#include <string>
#include <map>
#include <unordered_set>
#include <iostream>
#include <algorithm>
#include <list>
#include <cmath>
#include "solver.h"

/**
 * @brief Init solver from ProblemInfo
 */
Solver::Solver(Parser::ProblemInfo &problem) {
    Flute::readLUT();
    maxCellMove = problem.maxCellMove;
    numRow = problem.gGridX;
    numCol = problem.gGridY;
    numLayer = problem.numLayer;
    numMaster = problem.masters.size();
    numInst = problem.insts.size();
    numNet = problem.nets.size();
    totalCost = 0.0;
    cntCellMove = 0;

    comparator = [this](int a, int b) {
        auto na = Nets[a], nb = Nets[b];
        float wa = na.weight * (na.length - (na.rp - na.lp).norm1());
        float wb = nb.weight * (nb.length - (nb.rp - nb.lp).norm1());
        return wa == wb ? a < b : wa < wb;
    }; 
    
    Lock.resize(numInst, 0);
    NetHeap = std::set<int, decltype(comparator)>(comparator);

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
            j++;
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
        for (int j = 1; j <= numCol; ++j) 
            for (int k = 1; k <= numLayer; ++k) {
                DataBase.getSupply({i, j, k}) = Layers[k].supply;
            }
    }

    for (int i = 1, n = VAreas.size(); i < n; ++i) {
        for (auto p : VAreas[i].area) {
            DataBase.getVoltageLable({p.x, p.y}) = i;
        }
    }

    for (auto &p : problem.supplyGrids) 
        DataBase.getSupply({p.gX, p.gY, p.layer}) += p.diff;


    // Blockages
    for (auto &c : cInsts) 
        incBlockage(c, c.position, 1);

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
        DataBase.inc3DCon(Nets[netid].segments, 1);
    }

    for (auto &net : Nets) {
        canonicalize(net.segments);
        updateBox(net);
        net.updateLength();
        net.cost = calcCost(net.segments) * net.weight;
        totalCost += net.cost;
    }
    
    for (int i = 0; i < numNet; ++i)
        NetHeap.insert(i);

    DataBase.get2DFrom3D();
}

void Solver::incBlockage(CellInst &inst, Point p, int sgn) {
    auto m = mCells[inst.masterCell];
    for (auto b : m.blocks)
        DataBase.incDemand({p.x, p.y, b.layerNum}, sgn * b.demand);
}

float Solver::calcCost(std::vector<Segment> &segs) {
    float cost = 0.0;
    for (auto seg : segs) {
        if (seg.isVertical() || seg.isHorizontal()) 
            cost += ((seg.spoint - seg.epoint).norm1() + 1) * Layers[seg.spoint.z].powerFactor;
        else {
            int minz = seg.min().z, maxz = seg.max().z;
            for (int i = minz; i <= maxz; ++i) {
                bool intersect = false;
                for (unsigned j = 0, n = segs.size(); j < n; ++j)
                    if (!segs[j].isVertical()) {
                        if (segs[j].spoint.z == i && 
                            segs[j].min().x <= seg.spoint.x && segs[j].max().x >= seg.spoint.x &&
                            segs[j].min().y <= seg.spoint.y && segs[j].max().y >= seg.spoint.y) 
                        {
                            intersect = true;
                            break;
                        }
                    }
                if (intersect == true)
                    continue;
                cost += Layers[i].powerFactor;
            }
        }
    }
    return cost;
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
        bool flag = true;
        while (nxt != segList.end()) {
            if (Segment::isIntersect(curSeg, *nxt)) {
                *nxt = Segment::Merge(curSeg, *nxt);
                flag = false;
                break;
            } else 
                nxt++;
        }
        segList.erase(iter++);
        if (flag) route_seg.push_back(curSeg);
    }
}

/**
 * @brief route nets, first do 2d routing, then do layer assignment
 * @param ids nets that needs to be routed
 * @return bool route successful or not
 */
bool Solver::route_nets(std::vector<int> &ids) {
    /// init 2d routing info
    
    std::vector<RouteInfo> r2d(ids.size());
    for (unsigned i = 0, n = ids.size(); i < n; ++i)
        init2d(ids[i], r2d[i]);
    
    /// do 2d routing
    std::vector<std::vector<Segment> > backup;
    for (auto &info : r2d) {
        if (route2d(info) == false) {
            for (auto &x : backup)
                DataBase.inc2DCon(x, 1);
            return false;
        }
        std::vector<Segment> segs_2d;

        for (auto &node : info.tree.nodes)
            for (auto &twopin : node.neighbor)
                segs_2d.insert(segs_2d.end(), twopin.segs.begin(), twopin.segs.end());

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
        printNet(Nets[info.netid]);
        canonicalize(info.segs_3d);

        dbg_print("%d\n", info.netid);
        for (auto x : info.segs_3d) {
            dbg_print("(%d %d %d)", x.spoint.x, x.spoint.y, x.spoint.z);
            dbg_print(" -- (%d %d %d)\n", x.epoint.x, x.epoint.y, x.epoint.z);
        }
        dbg_print("----------------------\n");
        DataBase.inc3DCon(info.segs_3d, -1);
        backup.push_back(info.segs_3d);
    }
    for (auto &info : r2d)
        Nets[info.netid].segments = info.segs_3d;

    return true;
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
    std::vector<Point> old_location;

    for (int i = 0, n = insts.size(); i < n; ++i) {
        int id = insts[i];
        if (cInsts[id].position == new_locs[i])
            continue;
        for (auto &net_id : cInsts[id].NetIds) 
            changed_net.push_back(net_id);
    }

    std::sort(changed_net.begin(), changed_net.end());
    auto iter = std::unique(changed_net.begin(), changed_net.end());
    changed_net.resize(std::distance(changed_net.begin(), iter));

    /// backup original stats
    backup(insts, changed_net);

    for (int i = 0, n = insts.size(); i < n; ++i) {
        int id = insts[i];
        if (cInsts[id].position == new_locs[i])    
            continue;
        if (cInsts[id].movedFlag == 0) {
            cInsts[id].movedFlag = 1;
            cntCellMove++;
        }
    }

    /// erase from net heap
    for (auto idx : changed_net)
        NetHeap.erase(idx);

    /// update blockage
    for (int i = 0, n = insts.size(); i < n; ++i) {
        int id = insts[i];
        int x = cInsts[id].position.x, y = cInsts[id].position.y;

        incBlockage(cInsts[id], Point(x, y) , -1);
        incBlockage(cInsts[id], Point(new_locs[i].x, new_locs[i].y), 1);
        old_location.push_back(cInsts[id].position);
        cInsts[id].position = new_locs[i];
    }

    /// ripup these nets
    for (auto &x : changed_net) {
        DataBase.inc3DCon(Nets[x].segments, 1);
        DataBase.inc2DCon(Nets[x].segments, 1);
        totalCost -= Nets[x].cost;
    }
    
    bool flag = route_nets(changed_net);
    /// recover
    for (auto idx : changed_net)
        NetHeap.insert(idx);

    if (!flag) {
        recover();
        return false;

        for (unsigned i = 0, n = insts.size(); i < n; ++i) {
            int id = insts[i];
            int x = cInsts[id].position.x, y = cInsts[id].position.y;
            incBlockage(cInsts[id], Point(x, y) , -1);
            incBlockage(cInsts[id], Point(old_location[i].x, old_location[i].y), 1);
            cInsts[id].position = old_location[i];
        }
        for (auto &x : changed_net) {
            DataBase.inc3DCon(Nets[x].segments, -1);
            DataBase.inc2DCon(Nets[x].segments, -1);
            totalCost += Nets[x].cost;
        }
        return false;
    } 

    // update stats
    for (auto &x : changed_net) {
        updateBox(Nets[x]);
        Nets[x].updateLength();
        Nets[x].cost = calcCost(Nets[x].segments) * Nets[x].weight;
        totalCost += Nets[x].cost;
    }

    return true;
}

void Solver::backup(const std::vector<int> &insts, const std::vector<int> &nets) {
    backup_nets.clear();
    backup_insts.clear();
    backup_instids = insts;
    backup_netids = nets;

    for (auto idx : insts)
        backup_insts.push_back(cInsts[idx]);
    for (auto idx : nets)
        backup_nets.push_back(Nets[idx]);
}

void Solver::recover() {

    for (int i = 0, n = backup_netids.size(); i < n; ++i)
        NetHeap.erase(backup_netids[i]);

    for (int i = 0, n = backup_netids.size(); i < n; ++i) {
        int idx = backup_netids[i];
        auto &net = backup_nets[i];

        DataBase.inc3DCon(Nets[idx].segments, 1);
        DataBase.inc2DCon(Nets[idx].segments, 1);
        DataBase.inc3DCon(net.segments, -1);
        DataBase.inc2DCon(net.segments, -1);

        totalCost += net.cost - Nets[idx].cost;
        Nets[idx] = net;

        NetHeap.insert(idx);
    }
    
    for (int i = 0, n = backup_insts.size(); i < n; ++i) {
        int idx = backup_instids[i];
        auto &inst = backup_insts[i];

        incBlockage(inst, cInsts[idx].position, -1);
        incBlockage(inst, inst.position, 1);

        if (cInsts[idx].movedFlag == 1 && inst.movedFlag == 0) 
            cntCellMove--;

        cInsts[idx] = inst;
    }
}

void Solver::run() {
    dbg_print("Original Cost: %f\n", totalCost);

    /// simulated annealing
    int max_step = 1;
    float original_temp = sqrt(totalCost);

    for (int i = 0; i < max_step; ++i) {
        if (cntCellMove >= maxCellMove)
            break;
        if (NetHeap.empty())
            break;

        float T = original_temp * (1.0 - (float) i / max_step);

        float cost_before = totalCost;

        auto insts = getMoveList();
        bool flag = route_after_move(insts.first, insts.second);
        
        if (flag == false) {
            dbg_print("Failed");

            /// prevent routing the same net in the next time and get no solution
            backup_heap.push_back(*NetHeap.begin());
            NetHeap.erase(NetHeap.begin());
            continue;
        }

        /// push the index erased before into net heap
        for (auto idx : backup_heap)
            NetHeap.insert(idx);
        backup_heap.clear();

        float cost_after = totalCost;

        if (cost_after > cost_before) {
            float p = exp((cost_before - cost_after) / T);
            if (p < ((double) rand() / RAND_MAX))
                recover();
        }
    }

    dbg_print("Cost after movement and reroute: %f\n", totalCost);
}

void Solver::printNet(Net &net) {
    dbg_print("Net %s:\n", net.name.c_str());
    for (auto pin : net.pins) {
        auto inst = cInsts[pin.inst];
        dbg_print("%d, %d, %d\n", inst.position.x, inst.position.y, pin.layer);
    }
    dbg_print("---------------------------\n");
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