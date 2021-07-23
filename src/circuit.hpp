#ifndef CIRCUIT_HPP
#define CIRCUIT_HPP

#include <iostream>
#include <vector>
#include <cstring>
#include <string>
#include <map>
#include "utils.h"

class Point {
public:
    int x, y, z;
    Point() = default;
    Point(int _x, int _y, int _z): x(_x), y(_y), z(_z) {}
    Point(int _x, int _y) : Point(_x, _y, 0) {}
    bool operator == (const Point &a) const {
        return x == a.x && y == a.y && z == a.z;
    }
    Point operator + (const Point &a) const {
        return Point(x + a.x, y + a.y, z + a.z);
    }
    Point operator - (const Point &a) const {
        return Point(x - a.x, y - a.y, z - a.z);
    }
    Point &operator = (const Point &a) = default;
    int norm1() const {
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
    bool isVertical() const {return spoint.x != epoint.x;}
    bool isHorizontal() const {return spoint.y != epoint.y;}
    bool isVia() const {return spoint.z != epoint.z;}

    Point min() const {
        return Point(std::min(spoint.x, epoint.x), std::min(spoint.y, epoint.y), 
                     std::min(spoint.z, epoint.z));
    }

    Point max() const {
        return Point(std::max(spoint.x, epoint.x), std::max(spoint.y, epoint.y),
                     std::max(spoint.z, epoint.z));
    }

    /**
     * @brief only consider intersections between segments of same direction
     */
    static bool isIntersect(const Segment &a, const Segment &b) {
        if (a.isVertical() && b.isVertical()) {
            if (a.spoint.y != b.spoint.y || a.spoint.z != b.spoint.z)
                return false;
            int al = std::min(a.spoint.x, a.epoint.x), ar = std::max(a.spoint.x, a.epoint.x);
            int bl = std::min(b.spoint.x, b.epoint.x), br = std::max(b.spoint.x, b.epoint.x);
            if ((al <= bl && bl <= ar) || (al <= br && br <= ar))
                return true;
            return false;
        } else if (a.isHorizontal() && b.isHorizontal()) {
            if (a.spoint.x != b.spoint.x || a.spoint.z != b.spoint.z)
                return false;
            int al = std::min(a.spoint.y, a.epoint.y), ar = std::max(a.spoint.y, a.epoint.y);
            int bl = std::min(b.spoint.y, b.epoint.y), br = std::max(b.spoint.y, b.epoint.y);
            if ((al <= bl && bl <= ar) || (al <= br && br <= ar))
                return true;
            return false;
        } else if (a.isVia() && b.isVia()) {
            if (a.spoint.x != b.spoint.x || a.spoint.y != b.spoint.y)
                return false;
            int al = std::min(a.spoint.z, a.epoint.z), ar = std::max(a.spoint.z, a.epoint.z);
            int bl = std::min(b.spoint.z, b.epoint.z), br = std::max(b.spoint.z, b.epoint.z);
            if ((al <= bl && bl <= ar) || (al <= br && br <= ar))
                return true;
            return false;
        }
        return false;
    }

    static Segment Merge(const Segment &a, const Segment &b) {
        assert(isIntersect(a, b));
        Point mina = a.min(), minb = b.min(), maxa = a.max(), maxb = b.max();
        if (a.isVertical())
            return Segment({std::min(mina.x, minb.x), mina.y, mina.z}, {std::max(maxa.x, maxb.x), mina.y, mina.z});
        if (a.isHorizontal())
            return Segment({mina.x, std::min(mina.y, minb.y), mina.z}, {mina.x, std::max(maxa.x, maxb.x), mina.z});
        if (a.isVia())
            return Segment({mina.x, mina.y, std::min(mina.z, minb.z)}, {mina.x, mina.y, std::max(maxa.z, maxb.z)});
    }
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
    int pinCount, minConstraint, length;
    float weight, cost;
    struct NetPin {
        int inst, pin, layer;
    };
    std::vector<NetPin> pins;
    std::vector<Segment> segments;
    Point lp, rp; // bounding box boundary

    void updateBox() {
        lp.x = lp.y = lp.z = INF;
        rp.x = rp.y = rp.z = 0;

        for (auto &seg : segments) {
            
            lp.x = std::min(lp.x, std::min(seg.spoint.x, seg.epoint.x));
            lp.y = std::min(lp.y, std::min(seg.spoint.y, seg.epoint.y));
            lp.z = std::min(lp.z, std::min(seg.spoint.z, seg.epoint.z));

            rp.x = std::max(rp.x, std::max(seg.spoint.x, seg.epoint.x));
            rp.y = std::max(rp.y, std::max(seg.spoint.y, seg.epoint.y));
            rp.z = std::max(rp.z, std::max(seg.spoint.z, seg.epoint.z));
        }
    }

    void updateLength() {
        length = 0;
        for (auto &seg : segments) 
            length += (seg.spoint - seg.epoint).norm1();
    }

    Net() = default;
    Net(std::string n, int p, int m, float w): name(n), pinCount(p), minConstraint(m), weight(w) {
        pins.resize(p);
        segments.clear();
        cost = 0.0;
        length = 0;
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