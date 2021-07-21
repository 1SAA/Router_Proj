#include "circuit.hpp"
#include "parser.h"
#include "movement.h"
#include "utils.h"
#include <cmath>
#include <algorithm>

using std::vector;
using std::string;
using std::map;
using std::min;
using std::max;

namespace Movement {

const int MAXBOUND = 300, MAXLAYER = 20, ALPHA = 2, BETA = 30;

int maxMove, numRow, numCol, numLay, numMas, numIns, numNet;
int cntMove;

vector<Layer> Layers;
vector<MasterCell> mCells;
vector<CellInst> cInsts;
vector<Net> Nets;
vector<VoltageArea> VAreas;
vector<NoneDefaultSupply> supPos;

map<string, int> name2nl, name2mc, name2ci, name2VA, name2net;

int leftSup[MAXLAYER][MAXBOUND][MAXBOUND];
int vlabel[MAXBOUND][MAXBOUND];

/**
 * @brief Initialize all data from input
 * 
 * @param problem 
 */
void init(Parser::ProblemInfo &problem) {
    
    maxMove = problem.maxCellMove;
    numRow = problem.gGridX;
    numCol = problem.gGridY;
    numLay = problem.numLayer;
    numMas = problem.masters.size();
    numIns = problem.insts.size();
    numNet = problem.nets.size();

    Layers.resize(numLay + 1);
    for (auto layer : problem.layers) {
        int idx = layer.idx;
        Layers[idx] = Layer(layer.name, idx, layer.direction,
                            layer.defSupply, layer.powerFactor);
        name2nl[layer.name] = idx;
    }

/*
    dbg_print("The number of layers: %d\n", numLay);
    for (int i =  1; i <= numLay; i++) {
        auto &layer = Layers[i];
        dbg_print("Layer %d: %s %c sup: %d pow: %f\n", layer.idx, layer.name.c_str(),
        layer.horv == 0 ? 'H' : 'V', layer.supply, layer.powerFactor);
    }
*/

    mCells.resize(numMas);
    int cur = 0;
    for (auto &mc : problem.masters) {
        auto &curMC = mCells[cur];
        name2mc[mc.name] = cur;
        curMC = MasterCell(mc.name, mc.cntPin, mc.cntBlockage);
        for (int i = 0; i < mc.pins.size(); i++) {
            auto &p = mc.pins[i];
            curMC.pins[i] = Pin(p.name, name2nl[p.layer]);
            curMC.s2pin[p.name] = i;
        }
        for (int i = 0; i < mc.blockages.size(); i++) {
            auto &b = mc.blockages[i];
            curMC.blocks[i] = Blockage(b.name, name2nl[b.layer],
                                       b.demand);
            curMC.s2block[b.name] = i;
        }
        cur++;
    }

/*
    dbg_print("The number of master cells: %d\n", numMas);
    for (auto &mc : mCells) {
        dbg_print("Master Cell %s pincount: %d blockage count %d\n", 
        mc.name.c_str(), mc.pinCount, mc.blockageCount);
        for (auto &pin : mc.pins) 
            dbg_print("Pin %s %d\n", pin.name.c_str(), pin.layerNum);
        for (auto &block : mc.blocks)
            dbg_print("Block %s %d %d\n", block.name.c_str(), block.layerNum
            , block.demand);
    }
*/

    cInsts.resize(numIns);
    cur = 0;
    for (auto &ci : problem.insts) {
        auto &inst = cInsts[cur];
        inst = CellInst(ci.name, Point(ci.gX, ci.gY, 0), name2mc[ci.masterName],
                        ci.moveConstr, 0);
        name2ci[ci.name] = cur;
        cur++;
    }

/*
    dbg_print("The number of cell instances: %d\n", numIns);
*/

    memset(vlabel, 0, sizeof vlabel);
    cur = 1;
    VAreas.resize(problem.areas.size() + 1);
    for (auto &x : problem.areas) {
        auto &v = VAreas[cur];
        name2VA[x.name] = cur;
        v = VoltageArea();
        for (auto &g : x.grids) {
            v.area.push_back(Point(g.first, g.second, 0));
            vlabel[g.first][g.second] = cur;
        }
        cur++;
    }
    for (auto &x : problem.areas) {
        int idArea = name2VA[x.name];
        for (auto &inst : x.instNames) {
            int idInst = name2ci[inst];
            VAreas[idArea].instList.push_back(idInst);
            cInsts[idInst].voltageLabel = idArea;
        }
    }

    cur = 0;
    Nets.resize(problem.nets.size());
    for (auto &n : problem.nets) {
        name2net[n.name] = cur;
        auto &net = Nets[cur];
        int minLayer;
        if (n.minRoutingLayConstr == "NoCstr")
            minLayer = 1;
        else 
            minLayer = name2nl[n.minRoutingLayConstr];
        net = Net(n.name, n.cntPin, minLayer, n.weight);
        for (int i = 0; i < n.pins.size(); i++) {
            int inst = name2ci[n.pins[i].instName];
            int master = cInsts[inst].masterCell;
            int pin = mCells[master].s2pin[n.pins[i].pinName];
            net[i] = {inst, pin, mCells[master].pins[pin].layerNum};
            cInsts[inst].NetIds.push_back(cur);
        }
        cur++;
    }

    for (auto &x : problem.routes) {
        int netid = name2net[x.netName];
        Nets[netid].segments.push_back(Segment(Point(x.sX, x.sY, x.sL), Point(x.eX, x.eY, x.eL)));
    }

    supPos.resize(problem.supplyGrids.size());
    cur = 0;
    for (auto &p : problem.supplyGrids) {
        auto &ele = supPos[cur];
        ele = NoneDefaultSupply(Point(p.gX, p.gY, p.layer), p.diff);
        cur++;
    }
    
}

int getLength(Net &net) {
    int sum = 0;
    map<Point, int> vis = {};
    for (auto &seg : net.segments) {

        if (!vis[seg.spoint])
            vis[seg.spoint] = 1,
            ++sum;
        
        if (!vis[seg.epoint])
            vis[seg.epoint] = 1,
            ++sum;

        int dist;

        if (seg.isVertical()) 
            dist = abs(seg.spoint.x - seg.epoint.x);
        else if(seg.isHorizontal()) 
            dist = abs(seg.spoint.y - seg.epoint.y);
        else if(seg.isVia()) 
            dist = abs(seg.spoint.z - seg.epoint.z);
        else 
            dist = 1;

        sum += dist - 1;
    }

    return sum;
}

void getBoundingBox(Net &net, Point &l, Point &r) {

    l.x = l.y = l.z = max(numRow, max(numCol, numLay)) + 1;
    r.x = r.y = r.z = 0;

    for (auto &seg : net.segments) {
        
        l.x = min(l.x, min(seg.spoint.x, seg.epoint.x));
        l.y = min(l.y, min(seg.spoint.y, seg.epoint.y));
        l.z = min(l.z, min(seg.spoint.z, seg.epoint.z));

        r.x = max(r.x, max(seg.spoint.x, seg.epoint.x));
        r.y = max(r.y, max(seg.spoint.y, seg.epoint.y));
        r.z = max(r.z, max(seg.spoint.z, seg.epoint.z));
    }
}

void get2DBound(Net &net, Point &l, Point &r, string n) {
    l.x = l.y = max(numRow, max(numCol, numLay)) + 1;
    r.x = r.y = 0;
    for (auto &np : net.pins) {

        if (cInsts[np.inst].name == n)
            continue;

        Point pos = cInsts[np.inst].position;

        l.x = min(l.x, pos.x);
        l.y = min(l.y, pos.y);

        r.x = max(r.x, pos.x);
        r.y = max(r.y, pos.y);
    }
}

void addInst(CellInst &inst, int c) {
    Point pos = inst.position;
    int master = inst.masterCell;
    for (auto &b : mCells[master].blocks) 
        leftSup[b.layerNum][pos.x][pos.y] += c * b.demand;
    for (auto &p : mCells[master].pins)
        leftSup[p.layerNum][pos.x][pos.y] += c * ALPHA;
}

bool checkInst(CellInst &inst) {
    Point pos = inst.position;
    if (inst.voltageLabel && inst.voltageLabel != vlabel[pos.x][pos.y])
        return false;
    int master = inst.masterCell;
    for (auto &b : mCells[master].blocks)
        if (leftSup[b.layerNum][pos.x][pos.y] < b.demand) 
            return false;
    for (auto &p : mCells[master].pins)
        if (leftSup[p.layerNum][pos.x][pos.y] < ALPHA)
            return false;
    return true;
}

void moveInst(CellInst &inst, Point &dest) {
    int n = inst.NetIds.size(), m = n << 1, i = 0;
    int *buffX, *buffY, mx1, mx2, my1, my2;
    buffX = new int[m];
    buffY = new int[m];

    Point l, r;
    for (int nid : inst.NetIds) {
        get2DBound(Nets[nid], l, r, inst.name);
//        dbg_print("point l %d %d\n", l.x, l.y);
//        dbg_print("point r %d %d\n", r.x, r.y);
        buffX[i << 1] = l.x;
        buffX[i << 1 | 1] = r.x;
        buffY[i << 1] = l.y;
        buffY[i << 1 | 1] = r.y;
        i++;
    }

    std::nth_element(buffX, buffX + n - 1, buffX + m);
    std::nth_element(buffY, buffY + n - 1, buffY + m);
    mx1 = buffX[n - 1];
    my1 = buffY[n - 1];

    std::nth_element(buffX, buffX + n, buffX + m);
    std::nth_element(buffY, buffY + n, buffY + m);
    mx2 = buffX[n];
    my2 = buffY[n];

    dest.x = (mx1 + mx2 + 1) >> 1;
    dest.y = (my1 + my2 + 1) >> 1;

    delete [] buffX;
    delete [] buffY;
}

void findPosition(CellInst &inst, int posx, int posy) {
    Point o = inst.position;

    inst.position = Point(posx, posy, 0);
    if (checkInst(inst)) 
        return;
    for (int d = 1; d <= BETA; d++) {
        int i, j;

        i = posx - d; j = posy;
        for (int c = 0; c < d; c++, i++, j--)
            if (i >= 1 && i <= numRow && j >= 1 && j <= numCol) {
                inst.position = Point(i, j, 0);
                if (checkInst(inst))
                    return;
            }
        
        i = posx; j = posy - d;
        for (int c = 0; c < d; c++, i++, j++)
            if (i >= 1 && i <= numRow && j >= 1 && j <= numCol) {
                inst.position = Point(i, j, 0);
                if (checkInst(inst))
                    return;
            }

        i = posx + d; j = posy;
        for (int c = 0; c < d; c++, i--, j++)
            if (i >= 1 && i <= numRow && j >= 1 && j <= numCol) {
                inst.position = Point(i, j, 0);
                if (checkInst(inst))
                    return;
            }
        
        i = posx; j = posy + d;
        for (int c = 0; c < d; c++, i--, j--)
            if (i >= 1 && i <= numRow && j >= 1 && j <= numCol) {
                inst.position = Point(i, j, 0);
                if (checkInst(inst))
                    return;
            }
    }

    inst.position = o;
}

struct cmp {
    vector<float> *w;
    cmp(vector<float> *pw): w(pw) {}
    bool operator () (const int &a, const int &b) {
        float wa = w->at(a), wb = w->at(b);
        return wa == wb ? a < b : wa > wb;
    }
};

vector<OneMovement> getMoveList() {
    
    vector<int> id(numNet);
    for (int i = 0; i < numNet; i++)
        id[i] = i;
    
    vector<Point> lp(numNet), rp(numNet);
    for (int i = 0; i < numNet; i++)
        getBoundingBox(Nets[i], lp[i], rp[i]);

    vector<float> w(numNet);
    for (int i = 0; i < numNet; i++) {
        auto &net = Nets[i];
        int length = getLength(net), elen = 0;
        elen += rp[i].x - lp[i].x;
        elen += rp[i].y - lp[i].y;
        elen += rp[i].z - lp[i].z;
        w[i] = net.weight * (length - elen);
/*
        dbg_print("%d %d %d\n", lp[i].x, lp[i].y, lp[i].z);
        dbg_print("%d %d %d\n", rp[i].x, rp[i].y, rp[i].z);
        dbg_print("Expected length %d\n", elen);\
        dbg_print("Weight %f\n", w[i]);
*/
    }

    sort(id.begin(), id.end(), cmp(&w));

/*
    for (int i = 0; i < numNet; i++)
        dbg_print("rank %d: %d\n", i, id[i]);
*/


    /*
    TO DO: Initialize congestion weight
    */

   for (int k = 1; k <= numLay; k++) {
       int defSup = Layers[k].supply;
       for (int i = 1; i <= numRow; i++)
        for (int j = 1; j <= numCol; j++)
            leftSup[k][i][j] = defSup;
//        dbg_print("Default Supply %d\n", defSup);
   }

    for (auto &d : supPos) {
        leftSup[d.position.z][d.position.x][d.position.y] += d.addition;
//        dbg_print("Addition %d\n", d.addition);
    }

    for (auto &inst : cInsts)
        addInst(inst, -1);

    vector<int> lock(numIns);
    vector<OneMovement> ret;
    for (int i = 0; i < numIns; i++) {
        auto &inst = cInsts[i];
        if (!inst.movable)
            lock[i] = 1;   
    }
    for (int i = 0; i < numNet; i++) {

        //dbg_print("Net number %d\n", i);

        auto &net = Nets[id[i]];
        for (auto &p : net.pins) {
            if (lock[p.inst])
                continue;
            //dbg_print("The number of instance %d\n", p.inst);
            CellInst &inst = cInsts[p.inst];
            addInst(inst, 1);
            Point o = inst.position, t;
            moveInst(inst, t);
            //dbg_print("Destination found\n");
            //dbg_print("%d %d\n", t.x, t.y);
            findPosition(inst, t.x, t.y);
            //dbg_print("Placement location found\n");
            addInst(inst, -1);
            if (inst.position == o)
                continue;
            lock[p.inst] = 1;
            ret.push_back(OneMovement(inst.name, inst.position));
            if (!inst.movedFlag)
                cntMove++;
            inst.movedFlag = 1;
        }
    }

    for (auto &o : ret) 
        dbg_print("Inst %s: %d %d\n", o.cellName.c_str(), o.dest.x, o.dest.y);

    return ret;
}

}