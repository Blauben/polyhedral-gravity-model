#pragma once

#include <array>
#include "Polyhedron.h"

using namespace polyhedralGravity;

enum Direction {
    X=0, Y=1, Z=2
};
using Plane = std::pair<std::array<double,3>, Direction>;
using Box = std::pair<std::array<double, 3>, std::array<double, 3>>;

struct SplitParam {
    const std::vector<Array3>& vertices;
    const std::vector<IndexArray3>& faces;
    const std::vector<size_t> indexBoundFaces;
    const Box boundingBox;
    const Direction splitDirection;
};
typedef struct SplitParam SplitParam;