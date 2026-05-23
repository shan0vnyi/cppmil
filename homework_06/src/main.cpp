#include "ballistics.hpp"

#include <cstdlib>
#include <iostream>

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "usage: ballistics_cli <input_path>\n";
    return EXIT_FAILURE;
  }

  const auto input = readBallisticsInput(argv[1]);
  if (!input.has_value()) {
    std::cerr << "error: failed to read ballistics input\n";
    return EXIT_FAILURE;
  }

  const auto solution = computeDropSolution(*input);
  if (!solution.has_value()) {
    std::cerr << "error: failed to compute drop solution\n";
    return EXIT_FAILURE;
  }

  printSolution(*solution);

  return EXIT_SUCCESS;
}