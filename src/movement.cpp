#include "circuit.hpp"
#include "parser.h"
#include "utils.h"
#include "solver.h"
#include <cmath>
#include <algorithm>

using std::vector;
using std::string;
using std::map;
using std::min;
using std::max;

const int MAXBOUND = 300, MAXLAYER = 20, ALPHA = 1, BETA = 30;

int Solver::getLength(Net &net) {
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

void Solver::updateBox(Net &net) {
    Point &l = net.lp, &r = net.rp;

    l.x = l.y = l.z = max(numRow, max(numCol, numLayer)) + 1;
    r.x = r.y = r.z = 0;

    for (auto &pin : net.pins) {
        Point p = cInsts[pin.inst].position;
        p.z = pin.layer;

        l.x = min(l.x, p.x);
        l.y = min(l.y, p.y);
        l.z = min(l.z, p.z);

        r.x = max(r.x, p.x);
        r.y = max(r.y, p.y);
        r.z = max(r.z, p.z);
    }
}

void Solver::get2DBound(Net &net, Point &l, Point &r, string n) {
    l.x = l.y = max(numRow, max(numCol, numLayer)) + 1;
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

void Solver::addInst(CellInst &inst, Point pos, int c) {
    int master = inst.masterCell;
    for (auto &b : mCells[master].blocks) 
        //leftSup[b.layerNum][pos.x][pos.y] += c * b.demand;
        DataBase.incDemand({pos.x, pos.y, b.layerNum}, c * b.demand);
    for (auto &p : mCells[master].pins)
        //leftSup[p.layerNum][pos.x][pos.y] += c * ALPHA; 
        DataBase.incDemand({pos.x, pos.y, p.layerNum}, c * ALPHA);
}

bool Solver::checkInst(CellInst &inst, Point pos) {
    if (inst.voltageLabel && inst.voltageLabel != DataBase.getVoltageLable(Point(pos.x, pos.y)))
        return false;
    int master = inst.masterCell;
    for (auto &b : mCells[master].blocks)
        if (DataBase.get3DCon({pos.x, pos.y, b.layerNum}) < 0)
            return false;
    for (auto &p : mCells[master].pins)
        if (DataBase.get3DCon({pos.x, pos.y, p.layerNum}) < 0)
            return false;
    return true;
}

void Solver::moveInst(CellInst &inst, Point &dest) {
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

Point Solver::findPosition(CellInst &inst, int posx, int posy) {
//    inst.position = Point(posx, posy, 0);
    if (checkInst(inst, Point(posx, posy))) 
        return Point(posx, posy);

    for (int d = 1; d <= BETA; d++) {
        int i, j;

        i = posx - d; j = posy;
        for (int c = 0; c < d; c++, i++, j--)
            if (i >= 1 && i <= numRow && j >= 1 && j <= numCol) {
                if (checkInst(inst, Point(i, j)))
                    return Point(i, j);
            }
        
        i = posx; j = posy - d;
        for (int c = 0; c < d; c++, i++, j++)
            if (i >= 1 && i <= numRow && j >= 1 && j <= numCol) {
                if (checkInst(inst, Point(i, j)))
                    return Point(i, j);
            }

        i = posx + d; j = posy;
        for (int c = 0; c < d; c++, i--, j++)
            if (i >= 1 && i <= numRow && j >= 1 && j <= numCol) {
                if (checkInst(inst, Point(i, j)))
                    return Point(i, j);
            }
        
        i = posx; j = posy + d;
        for (int c = 0; c < d; c++, i--, j--)
            if (i >= 1 && i <= numRow && j >= 1 && j <= numCol) {
                if (checkInst(inst, Point(i, j)))
                    return Point(i, j);
            }
    }
    return Point(inst.position.x, inst.position.y);
}


std::pair<std::vector<int>, std::vector<Point> >
Solver::getMoveList() {
    std::vector<int> movedInst;
    std::vector<Point> locations;

    int maxChange = 10, RemainedMove = maxCellMove - cntCellMove;
    int moveCnt = 0, moveCost = 0;;
    net_to_optimize.clear();

    for (int netid : NetHeap) {
        if (moveCnt >= maxChange || moveCost >= RemainedMove)
            break;

        net_to_optimize.push_back(netid);

        for (auto &p : Nets[netid].pins) {
            if (Lock[p.inst])
                continue;

            auto inst = cInsts[p.inst];

            if (inst.movable == 0)        
                continue;

            moveCnt++;
            if (!inst.movedFlag)
                moveCost++;
            
            movedInst.push_back(p.inst);
            Lock[p.inst] = 1;
        }
    }


    for (auto idx : movedInst)
        incBlockage(cInsts[idx], cInsts[idx].position, -1);

    for (auto idx : movedInst) {
        Point t;
        auto &inst = cInsts[idx];
        moveInst(inst, t);
        t = findPosition(inst, t.x, t.y);
        addInst(inst, t, 1);
        locations.push_back(t);
    }
    
    for (unsigned i = 0; i < movedInst.size(); ++i) 
        addInst(cInsts[movedInst[i]], locations[i], -1);

    for (auto idx : movedInst)
        incBlockage(cInsts[idx], cInsts[idx].position, 1);

    dbg_print("----------------------------\n");
    dbg_print("Net To Optimize:");
    for (auto idx : net_to_optimize) {
        NetHeap.erase(idx);
        dbg_print("%d ", idx);
    }
    dbg_print("\n");

    for (unsigned i = 0; i < movedInst.size(); ++i) {
        int idx = movedInst[i];
        dbg_print("Moved inst %s: ", cInsts[idx].name.c_str());
        dbg_print("(%d, %d) --> ", cInsts[idx].position.x, cInsts[idx].position.y);
        dbg_print("(%d, %d)\n", locations[i].x, locations[i].y);
    }
    dbg_print("-----------------------------\n");

    return std::make_pair(movedInst, locations);
}