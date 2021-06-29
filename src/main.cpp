/** @file main.cpp
 *  @brief Entry of the program
 */
#include <iostream>
#include <fstream>
#include "parser.h"
#include "utils.h"

int main(int argv, char *argc[]) {
    if (argv < 3) {
        std::cerr << "./cell_move_router <input_file> <output_file>\n";
        exit(0);
    }
    std::ifstream input(argc[0]);
    std::ofstream output(argc[1]);

    Parser::ProblemInfo problem;
    Parser::parse(input, problem);

}