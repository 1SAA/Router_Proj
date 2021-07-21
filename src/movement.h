#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "circuit.hpp"
#include "parser.h"

using std::vector;
using std::string;
using std::map;

namespace Movement {

class OneMovement {
public:
    string cellName;
    Point dest;
    OneMovement(string c, Point d): cellName(c), dest(d) {}
};

void init(Parser::ProblemInfo &problem);
vector<OneMovement> getMoveList();

}

#endif