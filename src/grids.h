/**
 * @file grids.h
 * @brief This file contains the definitions of grid properties
 */
#ifndef GRIDS_H
#define GRIDS_H

#include <vector>
#include "circuit.hpp"
#include "utils.h"

/**
 * @brief Informations of one grid
 */
class Grid {
public:
    struct Position {
        int x, y;
    };
    struct Congestion {
        int demand, supply;
    };
private:
    Position coord;
    int numLayers;
    int voltageLabel, Xcon, Ycon;
    std::vector<Congestion> layerCon;
public:
    Grid() {}
    Grid(const Position &_coord, const int &_numLayers, const int &_voltageLabel) :
    coord(_coord), numLayers(_numLayers), voltageLabel(_voltageLabel)
    {
        layerCon.resize(numLayers);
    }
    Position &getCoord() {return coord;}
    int &getVoltageLabel() {return voltageLabel;}
    int &getNumLayers() {return numLayers;}
    /**
     * @brief Congestion of this grid at layer i
     * @param i in [1, numLayers)
     */
    Congestion & operator [] (int i) {
        DEBUG(if (i <= 0 || i >= numLayers) dbg_print_line("invalid argument i: %d\n", i));
        return layerCon[i];
    }
};

/**
 * @brief this data structure holds the information about congestion
 *        (maybe include some information about the about the pass-by segments of a grid)
 *        2D congestions may not be consistent with 3D congestion
 */
class GridDB {
public:
    using Con2D = std::pair<int, int>; // first V direction, second H direction. 0 means unavailable
    struct Con3D {
        int NumLayer;
        std::vector<int> Supply;
        std::vector<int> Demand;

        Con3D(int NumLayer) : 
        NumLayer(NumLayer) 
        {
            Supply.resize(NumLayer, 0);
            Demand.resize(NumLayer, 0);
        }
        int &demand(int i) { 
            DEBUG(if (i <= 0 || i >= NumLayer) dbg_print_line("invalid argument i: %d\n", i));
            return Demand[i]; 
        }
        int &supply(int i) {
            DEBUG(if (i <= 0 || i >= NumLayer) dbg_print_line("invalid argument i: %d\n", i));
            return Supply[i]; 
        }
        int operator [] (const int &i) const {
            DEBUG(if (i <= 0 || i >= NumLayer) dbg_print_line("invalid argument i: %d\n", i));
            return Supply[i] - Demand[i];
        }
    };
private:
    std::vector<std::vector<Con2D> > Congestion2D; // first for H(0), second for V(1)
    std::vector<std::vector<Con3D> > Congestion3D;
    std::vector<std::vector<int> > VoltageLable;
    
    int NumRow, NumColumn, NumLayer;
public:
    GridDB() {}
    GridDB(int NumRow, int NumColumn, int NumLayer) :
    NumRow(NumRow), NumColumn(NumColumn), NumLayer(NumLayer)
    {
        Congestion2D.resize(NumRow + 1, std::vector<Con2D>(NumColumn + 1, std::make_pair(0, 0)));
        Congestion3D.resize(NumRow + 1, std::vector<Con3D>(NumColumn + 1, NumLayer + 1));
        VoltageLable.resize(NumRow + 1, std::vector<int>(NumColumn, 0));
    }

    void print() {
        dbg_print("2D Horizontal Congestion:\n");
        for (int i = 1; i <= NumRow; ++i) {
            for (int j = 1; j <= NumColumn; ++j)
                dbg_print("%d ", Congestion2D[i][j].first);
            dbg_print("\n");
        }
        dbg_print("2D Vertical Congestion:\n");
        for (int i = 1; i <= NumRow; ++i) {
            for (int j = 1; j <= NumColumn; ++j)
                dbg_print("%d ", Congestion2D[i][j].second);
            dbg_print("\n");
        }
        dbg_print("---------------------------\n");
    }

    void get2DFrom3D() {
        for (int i = 1; i <= NumRow; ++i)
            for (int j = 1; j <= NumColumn; ++j) {
                Congestion2D[i][j] = std::make_pair(0, 0);
                for (int k = 1; k <= NumLayer; ++k)
                    if (k & 1)
                        Congestion2D[i][j].first += Congestion3D[i][j][k];
                    else
                        Congestion2D[i][j].second += Congestion3D[i][j][k];
            }
    }

    int get2DCon(const Point &p, int flag) const {
        if (flag == 0)
            return Congestion2D[p.x][p.y].first;
        return Congestion2D[p.x][p.y].second;
    }

    int query2DCon(const Point &p, bool HorV) const {
        return HorV ? Congestion2D[p.x][p.y].first : Congestion2D[p.x][p.y].second;
    }

    int &getVoltageLable(const Point &p) {
        return VoltageLable[p.x][p.y];
    }

    int &getSupply(const Point &p) {
        return Congestion3D[p.x][p.y].supply(p.z);
    }

    void incDemand(const Point &p, const int &d) {
        Congestion3D[p.x][p.y].demand(p.z) += d;
        if (p.z & 1)
            Congestion2D[p.x][p.y].first -= d;
        else
            Congestion2D[p.x][p.y].second -= d;
    }
    
    int query2DCon(const Segment &seg) const {
        if (seg.isVertical()) {
            int minCon = INF;
            int minx = std::min(seg.spoint.x, seg.epoint.x);
            int maxx = std::max(seg.spoint.x, seg.epoint.x);
            for (int i = minx; i <= maxx; ++i)
                minCon = std::min(minCon, Congestion2D[i][seg.spoint.y].second);
            return minCon;
        } 
        if (seg.isHorizontal()) {
            int minCon = INF;
            int miny = std::min(seg.spoint.y, seg.epoint.y);
            int maxy = std::max(seg.spoint.y, seg.epoint.y);
            for (int i = miny; i <= maxy; ++i)
                minCon = std::min(minCon, Congestion2D[seg.spoint.x][i].first);
            return minCon;
        }
        return std::min(Congestion2D[seg.spoint.x][seg.spoint.y].first, Congestion2D[seg.spoint.x][seg.spoint.y].second);
    }

    int get3DCon(const Point &p) const {
        return Congestion3D[p.x][p.y][p.z];
    }

    int query3DCon(const Segment &seg) const {
        const Point &st = seg.spoint, &en = seg.epoint;
        int flagH = st.y != en.y;
        int flagV = st.x != en.x;
        int flagZ = st.z != en.z;
        DEBUG(if (flagH + flagV + flagZ > 1) {
                dbg_print_line("error: ");
                dbg_print_line("(%d %d %d) <-> (%d %d %d)\n", st.x, st.y, st.z, en.x, en.y, en.z);
            }
        );

        if (flagH) {
            int minCon = INF;
            for (int y = std::min(st.y, en.y); y <= std::max(st.y, en.y); ++y)
                minCon = std::min(minCon, get3DCon(Point(st.x, y, st.z)));
            return minCon;
        } 
        if (flagV) {
            int minCon = INF;
            for (int x = std::min(st.x, en.x); x <= std::max(st.x, en.x); ++x)
                minCon = std::min(minCon, get3DCon(Point(x, st.y, st.z)));
            return minCon;
        }
        if (flagZ) {
            int minCon = INF;
            for (int z = std::min(st.z, en.z); z <= std::max(st.z, en.z); ++z)
                minCon = std::min(minCon, get3DCon(Point(st.x, st.y, z)));
            return minCon;
        }
        return get3DCon(st);
    }

    /**
     * @brief increase 2D congestion from segs. seg can be both 2D segments and 3D segments
     *        this function doesn't guarantee the consistency between 2D congestion and 3D congestion
     * @param segs routing segments. 2D or 3D
     * @param d -1: increase congestion. 1: decrease congestion
     */
    void inc2DCon(const std::vector<Segment> &segs, int d) {
       for (auto &seg : segs)  {
           if (seg.isVertical()) {
                int minx = seg.min().x, maxx = seg.max().x;
                for (int i = minx; i <= maxx; ++i) 
                    Congestion2D[i][seg.spoint.y].second += d;

            } else if (seg.isHorizontal()) {
                int miny = seg.min().y, maxy = seg.max().y;
                for (int i = miny; i <= maxy; ++i) 
                    Congestion2D[seg.spoint.x][i].first += d;

            } else if (seg.isVia()) {
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
                    if (i & 1)
                        Congestion2D[seg.spoint.x][seg.spoint.y].first += d;
                    else
                        Congestion2D[seg.spoint.x][seg.spoint.y].second += d;
                }
            }
       }
    }

    /**
     * @brief update 3D congestion from routing segments.
     * @param segs routing segments. must be 3D segments
     * @param flag update 2D congestion or not
     */
    void inc3DCon(const std::vector<Segment> &segs, int d) {
        for (auto &seg : segs) {
            if (seg.isVertical()) {
                int minx = seg.min().x, maxx = seg.max().x;
                for (int i = minx; i <= maxx; ++i) 
                    Congestion3D[i][seg.spoint.y].demand(seg.spoint.z) -= d;

            } else if (seg.isHorizontal()) {
                int miny = seg.min().y, maxy = seg.max().y;
                for (int i = miny; i <= maxy; ++i) 
                    Congestion3D[seg.spoint.x][i].demand(seg.spoint.z) -= d;

            } else if (seg.isVia()) {
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
                    Congestion3D[seg.spoint.x][seg.spoint.y].demand(i) -= d;
                }
            }
        }
    }
};

#endif 