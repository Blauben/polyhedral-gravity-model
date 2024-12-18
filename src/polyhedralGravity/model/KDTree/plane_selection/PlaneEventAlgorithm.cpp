#include "polyhedralGravity/model/KDTree/plane_selection/PlaneEventAlgorithm.h"

namespace polyhedralGravity {

    TriangleCounter::TriangleCounter(const size_t dimensionCount, const std::array<size_t, 3> &initialValues)
        : dimensionTriangleValues(dimensionCount, initialValues) {
    }

    void TriangleCounter::updateMax(Direction direction, const size_t p_planar, const size_t p_end) {
        dimensionTriangleValues[static_cast<size_t>(direction)][1] -= p_planar + p_end;
    }

    void TriangleCounter::updateMin(Direction direction, const size_t p_planar, const size_t p_start) {
        dimensionTriangleValues[static_cast<size_t>(direction)][0] += p_planar + p_start;
    }
    void TriangleCounter::setPlanar(Direction direction, const size_t p_planar) {
        dimensionTriangleValues[static_cast<size_t>(direction)][2] = p_planar;
    }
    size_t TriangleCounter::getMin(Direction direction) const {
        return dimensionTriangleValues[static_cast<size_t>(direction)][0];
    }
    size_t TriangleCounter::getMax(Direction direction) const {
        return dimensionTriangleValues[static_cast<size_t>(direction)][1];
    }
    size_t TriangleCounter::getPlanar(Direction direction) const {
        return dimensionTriangleValues[static_cast<size_t>(direction)][2];
    }

    PlaneEventVector PlaneEventAlgorithm::generatePlaneEventsFromFaces(const SplitParam &splitParam, std::vector<Direction> directions) {
        PlaneEventVector events{};
        events.reserve(countFaces(splitParam.boundFaces) * 2);
        if (std::holds_alternative<PlaneEventVector>(splitParam.boundFaces)) {
            return std::get<PlaneEventVector>(splitParam.boundFaces);
        }
        const auto &boundTriangles{std::get<TriangleIndexVector>(splitParam.boundFaces)};
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

    std::tuple<Plane, double, bool> PlaneEventAlgorithm::traversePlaneEvents(const PlaneEventVector &events, TriangleCounter &triangleCounter, const Box &boundingBox) {
        //initialize the default plane and make it costly
        double cost{std::numeric_limits<double>::infinity()};
        Plane optPlane{};
        bool minSide{true};
        //traverse all the events
        int i{0};
        while (i < events.size()) {
            //poll a plane to test
            const Plane &candidatePlane = events[i].plane;
            //for each plane calculate the faces whose vertices lie in the plane. Differentiate between the face starting in the plane, ending in the plane or all vertices lying in the plane
            size_t p_start{0}, p_end{0}, p_planar{0};
            //count all faces that end in the plane, this works because the PlaneEvents are sorted by position and then by PlaneEventType
            while (i < events.size() && events[i].plane.orientation != candidatePlane.orientation && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::ending) {
                p_end++;
                i++;
            }
            //count all the faces that lie in the plane
            while (i < events.size() && events[i].plane.orientation != candidatePlane.orientation && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::planar) {
                p_planar++;
                i++;
            }
            //count all the faces that start in the plane
            while (i < events.size() && events[i].plane.orientation != candidatePlane.orientation && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::starting) {
                p_start++;
                i++;
            }
            //reference to the triangle counter of the current dimension -> better readability
            triangleCounter.setPlanar(candidatePlane.orientation, p_planar);
            triangleCounter.updateMax(candidatePlane.orientation, p_planar, p_end);
            //evaluate plane and update should the new plane be more efficient
            auto [candidateCost, minSideChosen] = costForPlane(boundingBox, candidatePlane, triangleCounter.getMin(candidatePlane.orientation), triangleCounter.getMax(candidatePlane.orientation), triangleCounter.getPlanar(candidatePlane.orientation));
            if (candidateCost < cost) {
                cost = candidateCost;
                optPlane = candidatePlane;
                minSide = minSideChosen;
            }
            //shift the plane to the next candidate and prepare next iteration
            triangleCounter.updateMin(candidatePlane.orientation, p_planar, p_start);
            triangleCounter.setPlanar(candidatePlane.orientation, 0);
        }
        //generate the triangle index lists for the child bounding boxes and return them along with the optimal plane and the plane's cost.
        return {optPlane, cost, minSide};
    }
}// namespace polyhedralGravity
