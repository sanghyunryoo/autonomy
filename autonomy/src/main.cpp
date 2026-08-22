#include <cstdlib>
#include <exception>
#include <iostream>

#include "autonomy/core/runtime_runner.hpp"

int main()
{
  autonomy::RuntimeRunner runner;
  try {
    if (!runner.init()) {
      std::cerr << "[main] failed to initialize RuntimeRunner\n";
      runner.finish();
      return EXIT_FAILURE;
    }
    runner.run();

    std::cerr << "[main] RuntimeRunner returned unexpectedly\n";
    runner.finish();
    return EXIT_FAILURE;
  } catch (const std::exception & error) {
    std::cerr << "[main] fatal error: " << error.what() << '\n';
  } catch (...) {
    std::cerr << "[main] fatal unknown error\n";
  }
  runner.finish();
  return EXIT_FAILURE;
}
