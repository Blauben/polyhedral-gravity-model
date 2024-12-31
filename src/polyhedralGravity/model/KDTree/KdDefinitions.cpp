#include "polyhedralGravity/model/KDTree/KdDefinitions.h"

namespace polyhedralGravity {

    Array3 normal(Direction direction) {
        switch (direction) {
            case Direction::X:
                return Array3{1, 0, 0};
            case Direction::Y:
                return Array3{0, 1, 0};
            case Direction::Z:
                return Array3{0, 0, 1};
            default:
                throw std::invalid_argument{"Unknown Direction enum value used during normal fetching."};
        }
    }

    Plane::Plane(const Array3 &point, Direction direction)
        : axisCoordinate(point[static_cast<int>(direction)]), orientation(direction) {
    }
    Plane::Plane(const double point, const Direction direction)
        : axisCoordinate(point), orientation(direction) {
    }

    Array3 Plane::normal(const bool returnFlipped) const {
        using namespace util;
        return polyhedralGravity::normal(orientation) * (returnFlipped ? -1 : 1);
    }

    Array3 Plane::originPoint() const {
        Array3 point{0.0, 0.0, 0.0};
        point[static_cast<int>(orientation)] = axisCoordinate;
        return point;
    }

    bool Plane::operator==(const Plane &other) const {
        return axisCoordinate == other.axisCoordinate && orientation == other.orientation;
    }

    Box::Box(const std::pair<Array3, Array3> &pair)
        : minPoint{pair.first}, maxPoint{pair.second} {
    }

    Box::Box()
        : minPoint{0.0, 0.0, 0.0}, maxPoint{0.0, 0.0, 0.0} {
    }

    std::pair<double, double> Box::rayBoxIntersection(const Array3 &origin, const Array3 &inverseRay) const {
        //calculate the parameter t in $ origin + t * ray = point $
        auto const lambdaIntersectSlabPoint = [&origin, &inverseRay](const Array3 &point) {
            using namespace util;
            //if original ray had 0 as coordinate then inverse ray coordinate is inf. TODO: check
            return (point - origin) * inverseRay;
        };
        //intersections with slabs defined through minPoint
        auto [tx_1, ty_1, tz_1] = lambdaIntersectSlabPoint(minPoint);
        //intersections with slabs defined through maxPoint
        auto [tx_2, ty_2, tz_2] = lambdaIntersectSlabPoint(maxPoint);

        //return the parameters in ordered by '<'
        auto [tx_enter, tx_exit] = std::minmax(tx_1, tx_2);
        auto [ty_enter, ty_exit] = std::minmax(ty_1, ty_2);
        auto [tz_enter, tz_exit] = std::minmax(tz_1, tz_2);

        //calculate the point where all slabs have been entered: t_enter
        const double t_enter{std::max(tx_enter, std::max(ty_enter, tz_enter))};
        //calculates the point where the first slab has been exited: t_exit
        const double t_exit{std::min(tx_exit, std::min(ty_exit, tz_exit))};

        return {t_enter, t_exit};
    }

    double Box::surfaceArea() const {
        const double width = std::abs(maxPoint[0] - minPoint[0]);
        const double length = std::abs(maxPoint[1] - minPoint[1]);
        const double height = std::abs(maxPoint[2] - minPoint[2]);
        return 2 * (width * length + width * height + length * height);
    }

    std::pair<Box, Box> Box::splitBox(const Plane &plane) const {
        //clone the original box two times -> modify clones to become child boxes defined by the splitting plane
        Box box1{*this};
        Box box2{*this};
        const Direction &axis{plane.orientation};
        //Shift edges of the boxes to match the plane
        box1.maxPoint[static_cast<int>(axis)] = plane.axisCoordinate;
        box2.minPoint[static_cast<int>(axis)] = plane.axisCoordinate;
        return std::make_pair(box1, box2);
    }

    std::vector<Array3> Box::clipToVoxel(const std::array<Array3, 3> &points) const {
        using namespace util;
        //use clipped as the input vector because the inner for loop swaps input and clipped each iteration,
        //since each iteration needs the output of the previous iteration as input.
        std::vector<Array3> clipped(points.cbegin(), points.cend());
        std::vector<Array3> input{};
        input.reserve(points.size());
        //every plane defined by the maxPoint has to flip its normal because the normals have to point inside the bounding box.
        bool flipPlane = false;
        for (const Direction direction: ALL_DIRECTIONS) {
            const auto directionPlanes = {Plane(minPoint, direction), Plane(maxPoint, direction)};
            for (const auto &plane: directionPlanes) {
                std::swap(input, clipped);
                clipToVoxelPlane(plane, flipPlane, input, clipped);
                input.clear();
                flipPlane = !flipPlane;
            }
        }
        return clipped;
    }

    void Box::clipToVoxelPlane(const Plane &plane, const bool flipPlaneNormal, const std::vector<Array3> &source, std::vector<Array3> &dest) {
        using namespace util;
        //the distance is interpreted in the normal direction, negative values are in opposite direction of the normal.
        static constexpr auto isInside = [](const double distance) { return distance >= 0.0; };
        static constexpr auto intersectionPoint = [](const Array3 &from, const Array3 &to, const double distanceFrom, const double distanceTo) {
            // solve for t in $ [(t * from + (1-t) * to ) - origin] * normal = 0 $
            // equation explained: search for a point on the plane defined by $ (point - origin) * normal $, where point is linearly interpolated using vectors from and to.
            const double t{distanceTo / (distanceTo - distanceFrom)};
            return from * t + to * (1.0 - t);
        };
        for (size_t i{0}; i < source.size(); i++) {
            const Array3 &from{source[i]};
            const Array3 &to{source[(i + 1) % source.size()]};
            // $ (from - origin) * normal = cos alpha * |from - origin| * |normal| = cos alpha * |from - origin| * 1 $ ^= distance of from to the plane in the direction of the normal.
            const double distanceFrom{dot(from - plane.originPoint(), plane.normal(flipPlaneNormal))};
            const double distanceTo{dot(to - plane.originPoint(), plane.normal(flipPlaneNormal))};
            if (isInside(distanceFrom) && isInside(distanceTo)) {
                dest.push_back(to);
            } else if (isInside(distanceFrom) && !isInside(distanceTo)) {
                dest.emplace_back(intersectionPoint(from, to, distanceFrom, distanceTo));
            } else if (!isInside(distanceFrom) && isInside(distanceTo)) {
                dest.emplace_back(intersectionPoint(from, to, distanceFrom, distanceTo));
                dest.push_back(to);
            } else if (!isInside(distanceFrom) && !isInside(distanceTo)) {
                //do nothing
            }
        }
    }


    PlaneEvent::PlaneEvent(const PlaneEventType type, const Plane plane, const unsigned faceIndex)
        : type{type}, plane{plane}, faceIndex{faceIndex} {
    }

    bool PlaneEvent::operator<(const PlaneEvent &other) const {
        if (this->plane.axisCoordinate != other.plane.axisCoordinate) {
            return this->plane.axisCoordinate < other.plane.axisCoordinate;
        }
        if (this->plane.orientation == other.plane.orientation) {
            return this->type < other.type;
        }
        return static_cast<unsigned>(this->plane.orientation) < static_cast<unsigned>(other.plane.orientation);
    }

    bool PlaneEvent::operator==(const PlaneEvent &other) const {
        return type == other.type && plane == other.plane && faceIndex == other.faceIndex;
    }

    TriangleIndexVector convertEventsToFaces(const std::variant<TriangleIndexVector, PlaneEventVector> &events) {
        if (std::holds_alternative<TriangleIndexVector>(events)) {
            return std::get<TriangleIndexVector>(events);
        }
        const auto &eventList{std::get<PlaneEventVector>(events)};
        TriangleIndexVector triangles{};
        triangles.reserve(eventList.size());
        //used to avoid duplication
        std::unordered_set<size_t> processedFaces{};
        auto insertIfAbsent = [&triangles, &processedFaces](const auto &planeEvent) {
            const auto faceIndex{planeEvent.faceIndex};
            if (processedFaces.find(faceIndex) == processedFaces.end()) {
                processedFaces.insert(faceIndex);
                triangles.push_back(faceIndex);
            }
        };
        thrust::for_each(thrust::host, eventList.cbegin(), eventList.cend(), insertIfAbsent);
        triangles.shrink_to_fit();
        return triangles;
    }

    size_t countFaces(const std::variant<TriangleIndexVector, PlaneEventVector> &triangles) {
        return std::visit(util::overloaded{[](const TriangleIndexVector &indexList) {
                                               return indexList.size();
                                           },
                                           [](const PlaneEventVector &eventList) {
                                               size_t count{0};
                                               std::unordered_set<size_t> processedFaces{};
                                               thrust::for_each(thrust::host, eventList.cbegin(), eventList.cend(), [&processedFaces, &count](const auto &planeEvent) {
                                                   if (processedFaces.find(planeEvent.faceIndex) == processedFaces.end()) {
                                                       processedFaces.insert(planeEvent.faceIndex);
                                                       count++;
                                                   }
                                               });
                                               return count;
                                           }},
                          triangles);
    }

    size_t recursionDepth(const size_t nodeId) {
        //t: depth of the current node, N: amount of nodes in the tree if tree is complete
        // $ N = 2^(t+1)-1 $ (geometric series formula for partial sums), assume $ N >= idx + 1 $
        return static_cast<size_t>(std::ceil(std::log2(nodeId + 2))) - 1;
    }

}// namespace polyhedralGravity
