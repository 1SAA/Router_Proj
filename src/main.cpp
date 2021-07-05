/** @file main.cpp
 *  @brief Entry of the program
 */
#include <iostream>
#include <fstream>
#include "parser.h"
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
    Solver solver(problem);
    solver.print();
    
    solver.print_solution(output);
}