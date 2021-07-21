#ifndef CIRCUIT_HPP
#define CIRCUIT_HPP

#include <iostream>
#include <vector>
#include <cstring>
#include <string>
#include <map>

class Point {
public:
    int x, y, z;
    Point() = default;
    Point(int _x, int _y, int _z): x(_x), y(_y), z(_z) {}
    Point(int _x, int _y) : Point(_x, _y, 0) {}
    bool operator == (const Point &a) {
        return x == a.x && y == a.y && z == a.z;
    }
    Point operator + (const Point &a) {
        return Point(x + a.x, y + a.y, z + a.z);
    }
    Point operator - (const Point &a) {
        return Point(x - a.x, y - a.y, z - a.z);
    }
    Point &operator = (const Point &a) = default;
    int norm1() {
        return std::abs(x) + std::abs(y) + std::abs(z);
    }
    friend std::ostream &operator << (std::ostream &os, const Point &p) {
        os << "(" << p.x << ", " << p.y << ", " << p.z << ", " << ")";
        return os;
    }
    friend bool operator < (const Point a, const Point b) {
        if (a.x != b.x)
            return a.x < b.x;
        else if (a.y != b.y)
            return a.y < b.y;
        else if (a.z != b.z)
            return a.z < b.z;
        return false;
    }
};

class Segment {
public:
    Segment() = default;
    Point spoint, epoint;
    Segment(Point s, Point e): spoint(s), epoint(e) {}
    bool isVertical() {return spoint.x != epoint.x;}
    bool isHorizontal() {return spoint.y != epoint.y;}
    bool isVia() {return spoint.z != epoint.z;}
};

class Layer {
public:
    std::string name;
    int idx, horv, supply; /// 0 horizontal, 1 vertical
    float powerFactor;
    Layer() = default;
    Layer(const std::string &n, int id, int hv, int s, float p):
        name(n), idx(id), horv(hv), supply(s), powerFactor(p) {}
};

class NoneDefaultSupply {
public:
    Point position;
    int addition;
    NoneDefaultSupply() = default;
    NoneDefaultSupply(Point p, int a): position(p), addition(a) {}
};

class Pin {
public:
    std::string name;
    int layerNum;
    Pin() = default;
    Pin(const std::string &n, int l): name(n), layerNum(l) {}
};

class Blockage {
public:
    std::string name;
    int layerNum, demand;
    Blockage() = default;
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
    MasterCell() = default;
    MasterCell(const std::string &n, int pc, int bc): name(n), pinCount(pc), blockageCount(bc) {
        pins.resize(pc); s2pin.clear();
        blocks.resize(bc); s2block.clear();
    }
};

class CellInst {
public:
    std::string name;
    Point position;
    int masterCell, movable, voltageLabel;
    bool movedFlag;
    std::vector<int> NetIds; /// nets associate with this cell inst
    CellInst() = default;
    CellInst(std::string n, Point p, int ma, int mo, int _voltageLabel): 
    name(n), position(p), masterCell(ma), movable(mo),
    voltageLabel(_voltageLabel)
    {
        movedFlag = 0;
    }
};

class Net {
public:
    std::string name;
    int pinCount, minConstraint;
    float weight;
    struct NetPin {
        int inst, pin, layer;
    };
    std::vector<NetPin> pins;
    std::vector<Segment> segments;
    Net() = default;
    Net(std::string n, int p, int m, float w): name(n), pinCount(p), minConstraint(m), weight(w) {
        pins.resize(p);
        segments.clear();
    }
    NetPin & operator [] (const int &i) { return pins[i]; }
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