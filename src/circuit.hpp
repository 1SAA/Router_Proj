#ifndef CIRCUIT_HPP
#define CIRCUIT_HPP

#include <vector>
#include <cstring>
#include <string>
#include <map>

class Point {
public:
    int row, col, lay;
    Point(int x, int y, int z): row(x), col(y), lay(z) {}
    
};

class Segment {
public:
    Point spoint, epoint;
    Segment(Point s, Point e): spoint(s), epoint(e) {}
};

class Layer {
public:
    std::string name;
    int idx, horv, supply;
    float powerFactor;
    Layer(std::string n, int id, int hv, int s, float p):
        name(n), idx(id), horv(hv), supply(s), powerFactor(p) {}
};

class NoneDefaultSupply {
public:
    Point position;
    int addition;
    NoneDefaultSupply(Point p, int a): position(p), addition(a) {}
};

class Pin {
public:
    std::string name;
    int layerNum;
    Pin(std::string n, int l): name(n), layerNum(l) {}
};

class Blockage {
public:
    std::string name;
    int layerNum, demand;
    Blockage(std::string n, int l, int d): name(n), layerNum(l), demand(d) {}
};

class MasterCell {
public:
    std::string name;
    int pinCount, blockageCount;
    std::vector<Pin> pins;
    std::map<std::string, int> s2pin;
    std::vector<Blockage> blocks;
    std::map<std::string, int> s2block;
    MasterCell(std::string n, int pc, int bc): name(n), pinCount(pc), blockageCount(bc) {
        pins.clear(); s2pin.clear();
        blocks.clear(); s2block.clear();
    }
};

class CellInst {
public:
    std::string name;
    Point position;
    int masterCell, movable;
    CellInst(std::string n, Point p, int ma, int mo): name(n), position(p), masterCell(ma), movable(mo) {}
};

class Net {
public:
    std::string name;
    int pinCount, minConstraint;
    float weight;
    std::vector<int> instList, pinsList;
    std::vector<Segment> segments;
    Net(std::string n, int p, int m, float w): name(n), pinCount(p), minConstraint(m), weight(w) {
        instList.clear();
        pinsList.clear();
        segments.clear();
    } 
};

class VoltageArea {
public:
    std::vector<Point> area;
    std::vector<int> instList;
    VoltageArea() {
        area.clear();
        area.clear();
    }
};

#endif