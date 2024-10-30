#include "polyhedralGravity/model/KDTree/plane_selection/PlaneSelectionAlgorithm.h"

#include "LogNSquaredPlane.h"

namespace polyhedralGravity {

    std::unique_ptr<PlaneSelectionAlgorithm> PlaneSelectionAlgorithm::create(Algorithm algorithm) {
        switch (algorithm) {
            case Algorithm::NOTREE:
                return std::make_unique<LogNSquaredPlane>();
            default:
                return std::make_unique<LogNSquaredPlane>();//TODO: adjust
        }
    }

    std::pair<const double, bool> PlaneSelectionAlgorithm::costForPlane(const Box &boundingBox, const Plane &plane, const size_t trianglesMin, const size_t trianglesMax, const size_t trianglesPlanar) {
        //Checks if the split plane is one of the faces of the bounding box, if so the split is useless
        if (plane.axisCoordinate == boundingBox.minPoint[static_cast<int>(plane.orientation)] || plane.axisCoordinate == boundingBox.maxPoint[static_cast<int>(plane.orientation)]) {
            //will be discarded later because not splitting is cheaper (finitely many nodes!) than using this plane (infinite cost)
            return {std::numeric_limits<double>::infinity(), false};
        }
        //calculate parameters for Surface Area Heuristic (SAH): childBoxSurfaceAreas; number of contained triangles in each box
        auto [box1, box2] = boundingBox.splitBox(plane);
        //equalT are triangles lying in the plane (not in the boxes)
        const double surfaceAreaBounding = boundingBox.surfaceArea();
        const double surfaceArea1 = box1.surfaceArea();
        const double surfaceArea2 = box2.surfaceArea();
        //evaluate SAH: Include equalT once in each box and record option with minimum cost
        const double costLesser = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * (static_cast<double>(trianglesMin + trianglesPlanar)) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>(trianglesMax));
        const double costUpper = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * static_cast<double>(trianglesMin) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>((trianglesMax + trianglesPlanar)));
        //if empty space is cut off, reduce cost by 20%
        const double factor = trianglesMin == 0 || trianglesMax == 0 ? 0.8 : 1;
        if (costLesser <= costUpper) {
            return {factor * costLesser, true};
        }
        //if empty space is cut off, reduce cost by 20%
        return {factor * costUpper, false};
    }

    template<typename... Points>
    std::array<double, sizeof...(Points)> PlaneSelectionAlgorithm::clipToVoxel(const Box &box, const Direction direction, Points... points) {
        auto clip = [&box, &direction](const Array3 &point) -> double {
            const auto &coordinate = point[static_cast<int>(direction)];
            if (coordinate < box.minPoint[static_cast<int>(direction)]) {
                return box.minPoint[static_cast<int>(direction)];
            }
            if (coordinate > box.maxPoint[static_cast<int>(direction)]) {
                return box.maxPoint[static_cast<int>(direction)];
            }
            return coordinate;
        };
        std::array<double, sizeof...(Points)> result{clip(points)...};
        return result;
    }
}// namespace polyhedralGravity