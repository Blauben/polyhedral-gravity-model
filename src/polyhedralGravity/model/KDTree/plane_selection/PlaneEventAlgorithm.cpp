#include "polyhedralGravity/model/KDTree/plane_selection/PlaneEventAlgorithm.h"

namespace polyhedralGravity {
    PlaneEventList PlaneEventAlgorithm::generatePlaneEventsFromFaces(const SplitParam &splitParam, std::vector<Direction> directions) {
        PlaneEventList events{};
        events.reserve(countFaces(splitParam.boundFaces) * 2);
        if (std::holds_alternative<PlaneEventList>(splitParam.boundFaces)) {
            return std::get<PlaneEventList>(splitParam.boundFaces);
        }
        const auto &boundTriangles{std::get<TriangleIndexList>(splitParam.boundFaces)};
        //transform the faces into vertices
        auto [vertex3_begin, vertex3_end] = transformIterator(boundTriangles.cbegin(), boundTriangles.cend(), splitParam.vertices, splitParam.faces);
        std::for_each(vertex3_begin, vertex3_end, [&splitParam, &events, &directions](const auto &indexAndTriplet) {
            const auto [index, triplet] = indexAndTriplet;
            //first clip the triangles vertices to the current bounding box and then get the bounding box of the clipped triangle -> use the box edges as split plane candidates
            const auto [minPoint, maxPoint] = Box::getBoundingBox<std::vector<Array3>>(splitParam.boundingBox.clipToVoxel(triplet));
            for (const auto &direction: directions) {
                // if the triangle is perpendicular to the split direction, generate a planar event with the candidate plane in which the triangle lies
                if (minPoint[static_cast<int>(direction)] == maxPoint[static_cast<int>(direction)]) {
                    events.emplace_back(
                            PlaneEventType::planar,
                            Plane(minPoint, direction),
                            index);
                    return;
                }
                //else create a starting and ending event consisting of the planes defined by the min and max points of the face's bounding box.
                events.emplace_back(
                        PlaneEventType::starting,
                        Plane(minPoint, direction),
                        index);
                events.emplace_back(
                        PlaneEventType::ending,
                        Plane(maxPoint, direction),
                        index);
            }
        });
        //sort the events by plane position and then by PlaneEventType. Refer to {@link PlaneEventType} for the specific order
        std::sort(events.begin(), events.end());
        return events;
    }
}// namespace polyhedralGravity
