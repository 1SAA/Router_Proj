#include <string>
#include <map>
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
    
    mCells.resize(numMaster);
    cInsts.resize(numInst);
    Nets.resize(numNet);
    Layers.resize(numLayer);
    VAreas.resize(problem.areas.size());

    std::map<std::string, int> name2layer;
    std::map<std::string, int> name2master;
    std::map<std::string, int> name2inst;
    std::map<std::string, int> name2VA;
    std::map<std::string, int> name2net;

    /// Voltage Areas
    int i = 0;
    for (auto x : problem.areas) {
        auto &v = VAreas[i];
        name2VA[x.name] = i;
        v = VoltageArea();
        for (auto g : x.grids)
            v.area.push_back(Point(g.first, g.second, 0));
        i++;
    }

    /// Layers
    for (auto x : problem.layers) {
        int id = x.idx;
        Layers[id] = Layer(x.name, x.idx, x.direction, x.defSupply, x.powerFactor);
        name2layer[x.name] = id;
    }

    /// Master Cells
    i = 0;
    for (auto &x : problem.masters) {
        auto &master = mCells[i];
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
    Circuit.resize(numRow, std::vector<Grid>(numCol));
    for (int i = 0; i < numRow; ++i) {
        for (int j = 0; j < numCol; ++j) {
            Circuit[i][j] = Grid({i, j}, numLayer, -1);
            for (int k = 0; k < numLayer; ++k)
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
    for (auto &x : problem.nets) {
        name2net[x.name] = i;
        auto &net = Nets[i];
        net = Net(x.name, x.cntPin, x.minRoutingLayConstr, x.weight);
        for (int j = 0; j < x.pins.size(); ++j) {
            int inst = name2inst[x.pins[j].instName];
            int master = cInsts[inst].masterCell;
            int pin = mCells[master].s2pin[x.pins[j].pinName];
            net[j] = {inst, pin};
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
 * @brief reroute after cell movement
 * @param insts id of cell instances
 * @param new_locs new locations of these cell instances
 * @return reroute is successful or not
 */
bool Solver::route_after_move(const std::vector<int> &insts, const std::vector<Point> &new_locs) {
    
}