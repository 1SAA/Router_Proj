/** @file main.cpp
 *  @brief Entry of the program
 */
#include <iostream>
#include <fstream>
#include "parser.h"
#include "movement.h"
#include "solver.h"
#include "utils.h"

int main(int argv, char *argc[]) {
    if (argv < 3) {
        std::cerr << "./cell_move_router <input_file> <output_file>\n";
        exit(0);
    }
    std::ifstream input(argc[1]);
    std::ofstream output(argc[2]);

    Parser::ProblemInfo problem;
    Parser::parse(input, problem);
    Movement::init(problem);
    dbg_print("Initialization is done.\n");
    vector<Movement::OneMovement> ret = Movement::getMoveList();
    dbg_print("The number of Movement %d\n", ret.size());
/*    Solver solver(problem);
    solver.print();
    
    solver.print_solution(output);*/
}