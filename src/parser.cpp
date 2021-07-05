/**
 * @file parser.cpp
 * @brief implementation of parse function
 */
#include "parser.h"
#include <iostream>
#include <sstream>

typedef std::stringstream sstream;

/**
 * @brief read one line and convert into stringstream
 */
static bool getLine(std::istream &is, sstream &ss) {
    std::string str;
    if (!std::getline(is, str))
        return false;
    ss = sstream(str);
    return true;
}

/**
 * @brief parse a input stream
 * @param input a reference to input stream
 * @param problem a reference to ProblemInfo data structure
 * @return void
 */
void Parser::parse(std::istream &input, ProblemInfo &problem) {
    string str;
    int foobar; /// place holder
    sstream line;
    while (getLine(input, line)) {
        string type;
        line >> type;
        if (type == "MaxCellMove") {
            line >> problem.maxCellMove;
        } else if (type == "GGridBoundaryIdx") {
            line >> foobar >> foobar >> problem.gGridX >> problem.gGridY;
        } else if (type == "NumLayer") {
            line >> problem.numLayer;
            problem.layers.resize(problem.numLayer);
            for (int i = 0; i < problem.numLayer; ++i) {
                getLine(input, line);
                LayerInfo &l = problem.layers[i];
                string dir;
                line >> str >> l.name >> l.idx
                    >> dir >> l.defSupply >> l.powerFactor;
                l.direction = dir == "H" ? 0 : 1;
            }
        } else if (type == "NumNonDefaultSupplyGGrid") {
            int size;
            line >> size;
            problem.supplyGrids.resize(size);
            for (int i = 0; i < size; ++i) {
                getLine(input, line);
                NoneDefSupply &s = problem.supplyGrids[i];
                line >> s.gX >> s.gY >> s.layer >> s.diff;
            }
        } else if (type == "NumMasterCell") {
            int size;
            line >> size;
            problem.masters.resize(size);
            for (int i = 0; i < size; ++i) {
                getLine(input, line);
                MasterCell &m = problem.masters[i];;
                line >> str >> m.name >> m.cntPin >> m.cntBlockage;
                m.pins.resize(m.cntPin);
                m.blockages.resize(m.cntBlockage);

                for (int j = 0; j < m.cntPin; ++j) {
                    getLine(input, line);
                    line >> str >> m.pins[j].name >> m.pins[j].layer;
                }
                
                for (int j = 0; j < m.cntBlockage; ++j) {
                    getLine(input, line);
                    line >> str >> m.blockages[j].name >> m.blockages[j].layer >> m.blockages[j].demand;
                }
            }
        } else if (type == "NumCellInst") {
            int size;
            line >> size;
            problem.insts.resize(size);
            for (int i = 0; i < size; ++i) {
                getLine(input, line);
                CellInst &v = problem.insts[i];
                string constr;
                line >> str >> v.name >> v.masterName >> v.gX >> v.gY >> constr;
                v.moveConstr = constr != "Fixed";
            }
        } else if (type == "NumNets") {
            int size;
            line >> size;
            problem.nets.resize(size);
            for (int i = 0; i < size; ++i) {
                getLine(input, line);
                NetInfo &n = problem.nets[i];
                line >> str >> n.name >> n.cntPin >> n.minRoutingLayConstr >> n.weight;
                n.pins.resize(n.cntPin);
                for (int j = 0; j < n.cntPin; ++j) {
                    getLine(input, line);
                    string cellpin;
                    line >> str >> cellpin;
                    int pos = cellpin.find('/');
                    n.pins[j].instName = cellpin.substr(0, pos);
                    n.pins[j].pinName = cellpin.substr(pos + 1);
                }
            }
        } else if (type == "NumVoltageAreas") {
            /// Each Voltage Area forms a connect component
            int size;
            line >> size;
            problem.areas.resize(size);

            for (int i = 0; i < size; ++i) {
                VoltageArea &area = problem.areas[i];

                getLine(input, line);
                line >> str >> area.name;

                getLine(input, line);
                int numGrid, numCell;
                line >> str >> numGrid;
                
                area.grids.resize(numGrid);
                for (int i = 0; i < numGrid; ++i) {
                    getLine(input, line);
                    line >> area.grids[i].first  >> area.grids[i].second;
                }

                getLine(input, line);
                line >> str >> numCell;
                area.instNames.resize(numCell);
                for (int i = 0; i < numCell; ++i) {
                    getLine(input, line);
                    line >> area.instNames[i];
                }
            }
        } else if (type == "NumRoutes") {
            int size;
            line >> size;
            problem.routes.resize(size);
            for (int i = 0; i < size; ++i) {
                getLine(input, line);
                RoutingSegment &route = problem.routes[i];
                line >> route.sX >> route.sY >> route.sL >> route.eX >> route.eY
                     >> route.eL >> route.netName;
            }
        }
    }
}