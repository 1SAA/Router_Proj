/**
 * @file grids.h
 * @brief This file contains the definitions of grid properties
 */
#ifndef GRIDS_H
#define GRIDS_H

#include <vector>
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
     * @param i in [0, numLayers)
     */
    Congestion & operator [] (int i) {
        DEBUG(if (i < 0 || i >= numLayers) dbg_print("invalid argument i: %d\n", i));
        return layerCon[i];
    }
};

#endif 