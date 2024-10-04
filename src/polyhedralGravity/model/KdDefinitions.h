#pragma once

#include <array>
#include "Polyhedron.h"

using namespace polyhedralGravity;

enum Direction {
    HORIZONTAL, VERTICAL
};
using Plane = std::pair<std::array<double,3>, Direction>;
using Box = std::pair<std::array<double, 3>, std::array<double, 3>>;

struct SplitParam { //TODO refactor into KDTree methods
    const std::vector<Array3>& vertices;
    const std::vector<IndexArray3>& faces;
    const std::vector<size_t> indexBoundFaces;
    const Box boundingBox;
};