#include "polyhedralGravity/input/YAMLConfigReader.h"
#include "polyhedralGravity/model/KDTree/time_measurement/TimeExecutable.h"

int main(int argc, char *argv[]) {
    using namespace polyhedralGravity;
    if (argc < 2) {
        std::cout << "Error: Please supply a config yaml argument!" << std::endl;
        return EXIT_FAILURE;
    }
    const std::shared_ptr<ConfigSource> config = std::make_shared<YAMLConfigReader>(argv[1]);
    const auto polyhedralSource = config->getDataSource()->getPolyhedralSource();
    measureTreePerformance(std::get<0>(polyhedralSource), std::get<1>(polyhedralSource));
    return EXIT_SUCCESS;
}