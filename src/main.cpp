#include "polyhedralGravity/Info.h"
#include "polyhedralGravity/input/ConfigSource.h"
#include "polyhedralGravity/input/YAMLConfigReader.h"
#include "polyhedralGravity/model/GravityModel.h"
#include "polyhedralGravity/output/CSVWriter.h"
#include "polyhedralGravity/output/Logging.h"
#include <chrono>

int main(int argc, char *argv[]) {
    std::vector<std::string> files{"../example-config/data/Eros_scaled-140296.node", "../example-config/data/Eros_scaled-140296.face"};
    const auto polyhedron = polyhedralGravity::Polyhedron(files, 1.0, polyhedralGravity::NormalOrientation::OUTWARDS, polyhedralGravity::PolyhedronIntegrity::HEAL, polyhedralGravity::PlaneSelectionAlgorithm::Algorithm::LOG);
}