/** @file parser.h
 *  @brief Definition of data structures and functions used for 
 *         storing input informations
 */

#ifndef _PARSER_H_
#define _PARSER_H_

#include <iostream>
#include <vector>
#include <string>

namespace Parser {

using std::vector;
using std::string;

struct ProblemInfo;
struct GridInfo;
struct NoneDefSupply;
struct RoutingSegment;
struct MasterCell;
struct CellInst;
struct NetInfo;
struct LayerInfo;
struct VoltageArea;
struct MasterPin;
struct Blockage;
struct PinInfo;

/** @brief Data structure storing the information retrieved from input
 */
struct ProblemInfo {
public:
    int maxCellMove;
    int gGridX, gGridY; /// idx: [1, gGridX] x [1, gGridY]
    int numLayer;
    vector<LayerInfo> layers;
    vector<NoneDefSupply> supplyGrids;
    vector<MasterCell> masters;
    vector<CellInst> insts;
    vector<NetInfo> nets;
    vector<RoutingSegment> routes;
    vector<VoltageArea> areas;
};

/** @brief: Data structure storing the information of a None Default Supply gGrid
 */
struct NoneDefSupply {
public:
    int gX, gY, layer, diff;
};

/** @brief Information of a Master Cell
 */
struct MasterCell {
public:
    string name;
    int cntPin, cntBlockage;
    vector<MasterPin> pins;
    vector<Blockage> blockages;
};

/** @brief Information of a Master Pin
 */
struct MasterPin {
public:
    string name, layer;
};

/** @brief Information of a blockage
 */
struct Blockage {
public:
    string name, layer;
    int demand;
};

/** @brief Information of a Cell Instance
 */
struct CellInst {
public:
    string name, masterName;
    int gX, gY, moveConstr;
};

/** @brief Information of a Net
 */
struct NetInfo {
public:
    string name;
    int cntPin, minRoutingLayConstr;
    float weight;
    vector<PinInfo> pins;
};

/** @brief Information of a Pin in a Net
 */
struct PinInfo {
public:
    string instName, pinName;    
};

/** @brief Information of a Layer
 */
struct LayerInfo {
public:
    string name;
    int idx, direction, defSupply;
    float powerFactor;
};

/** @brief Information of a Routing Segments
 */
struct RoutingSegment {
public:
    string netName;
    int sX, sY, sL, eX, eY, eL;
};

/** @brief Information of a Voltage Area
 * Each Voltage Area is a connected component
 * Each gGrid and CellInst belongs to at most one Voltage Area
 */
struct VoltageArea {
public:
    string name;
    vector<std::pair<int, int> > grids;
    vector<std::string> instNames;
};

void parse(std::istream &input, ProblemInfo &problem);
};

#endif