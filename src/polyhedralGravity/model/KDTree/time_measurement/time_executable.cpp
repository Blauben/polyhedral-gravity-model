#include "time_executable.h"

#include <cmath>

int main() {
    //TODO: fill with Functions and Param from KDTree and Polyhedron
    const std::vector<std::shared_ptr<IFunction>> functions{
        std::make_shared<Function<int, int>>("TestFunc", [](const int a) {return std::pow(a, 5200);}, 200),
        std::make_shared<Function<std::string, std::string>>("TestFunc", [](const std::string& a) {return a;}, "Hallo")
    };

    std::for_each(functions.begin(), functions.end(), [](const auto funcStructPtr) {
        const auto duration_ms = funcStructPtr->measureTimeMs();
        std::cout << funcStructPtr->name << " execution time: " << duration_ms << " ms" << std::endl;
    });

    return EXIT_SUCCESS;
}