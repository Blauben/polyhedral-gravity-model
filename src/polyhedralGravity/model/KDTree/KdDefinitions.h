#pragma once

#include "polyhedralGravity/model/GravityModelData.h"

#include <array>
#include <cstdint>
#include <memory>
#include <thrust/iterator/transform_iterator.h>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace polyhedralGravity {

    /**
     * Index for array access.
     */
    constexpr uint8_t MIN{0};
    /**
    * Index for array access.
    */
    constexpr uint8_t MAX{1};

    /**
     * Assigns an integer index to the coordinate axes
     *
     * Used to specify coordinates. E.g. CoordinateArray[Direction]
     */
    enum class Direction {
        X = 0,
        Y = 1,
        Z = 2
    };

    /**
     * Returns the normal vector for a direction.
     * @param direction The direction to return the normal vector for.
     * @return The normal vector.
     */
    static Array3 normal(const Direction direction) {
        switch (direction) {
            case Direction::X:
                return Array3{1, 0, 0};
            case Direction::Y:
                return Array3{0, 1, 0};
            case Direction::Z:
                return Array3{0, 0, 1};
            default:
                return Array3{0, 0, 0};
        }
    }

    /**
     * Number of dimensions for the polyhedron. Also corresponds to the number of elements of the {@link Direction} enum.
     */
    constexpr int DIMENSIONS = 3;

    /**
     * Defines a plane that is parallel to one of the coordinate planes, by taking the fixed axis coordinate value for the plane and the coordinate index ({@link Direction}) that is fixed for every \
     * point on the plane.
     *
     * E.g. Specifying 0.0 and Direction::X would describe the YZ plane that goes through the origin. The direction is equivalent to the coordinate that is 1 in the normal vector of the plane, the others are 0.
     */
    struct Plane {
        /**
         * Each point lying on the plane has to have this value in the dimension specified in the orientation parameter.
         */
        double axisCoordinate;
        /**
         * Specifies which coordinate dimension is fixed for every point on the plane.
         */
        Direction orientation;

        /**
        * Returns the normal vector for this plane.
        * @param returnFlipped Whether to return the normal pointing in the opposite direction.
        * @return The normal vector.
        */
        [[nodiscard]] Array3 normal(const bool returnFlipped = false) const {
            using namespace util;
            return polyhedralGravity::normal(orientation) * (returnFlipped ? -1 : 1);
        }

        /**
         * Returns the origin point of a plane, meaning a point that lies on the plane.
         * @return The origin point.
         */
        [[nodiscard]] Array3 originPoint() const {
            Array3 point{};
            point.fill(0);
            point[static_cast<int>(orientation)] = axisCoordinate;
            return point;
        }

        /**
        * Equality operator used for testing purposes
        */
        bool operator==(const Plane &other) const {
            return axisCoordinate == other.axisCoordinate && orientation == other.orientation;
        }

        Plane() = default;
        Plane(const Array3 &point, Direction direction)
            : axisCoordinate(point[static_cast<int>(direction)]), orientation(direction) {
        }
        Plane(const double point, const Direction direction)
            : axisCoordinate(point), orientation(direction) {
        }
    };

    /**
     * Defines a rectangular box by taking two opposite corner points. First is the point closest to the origin and second is the point farthest away.
     */
    struct Box {
        /**
         * The point closer to the origin, thus minimal
         */
        Array3 minPoint;
        /**
         * The point further away from the origin, thus maximal.
         */
        Array3 maxPoint;

        /**
         * Calculates the intersection points of a ray and a box.
         * @param origin The origin of the ray.
         * @param ray The ray direction vector.
         * @return Parameters t of the equation $ intersection_point = origin + t * ray $ for the entry and exit intersection points.
         */
        [[nodiscard]] std::pair<double, double> rayBoxIntersection(const Array3 &origin, const Array3 &ray) const {
            //calculate the parameter t in $ origin + t * ray = point $
            auto const lambdaIntersectSlabPoint = [&origin, &ray](const Array3 &point) {
                return std::make_tuple(
                        (point[0] - origin[0]) / ray[0],
                        (point[1] - origin[1]) / ray[1],
                        (point[2] - origin[2]) / ray[2]);
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

        /**
        * Calculates the surface area of a box.
        * @return the surface area
        */
        [[nodiscard]] double surfaceArea() const {
            const double width = std::abs(maxPoint[0] - minPoint[0]);
            const double length = std::abs(maxPoint[1] - minPoint[1]);
            const double height = std::abs(maxPoint[2] - minPoint[2]);
            return 2 * (width * length + width * height + length * height);
        }

        /**
       * Splits this box into two new boxes.
       * @param plane the plane by which to split the original box.
       * @return a pair of boxes that result by splitting this box.
       */
        [[nodiscard]] std::pair<Box, Box> splitBox(const Plane &plane) const {
            //clone the original box two times -> modify clones to become child boxes defined by the splitting plane
            Box box1{*this};
            Box box2{*this};
            const Direction &axis{plane.orientation};
            //Shift edges of the boxes to match the plane
            box1.maxPoint[static_cast<int>(axis)] = plane.axisCoordinate;
            box2.minPoint[static_cast<int>(axis)] = plane.axisCoordinate;
            return std::make_pair(box1, box2);
        }

        /**
        * Finds the minimal bounding box for a set of vertices.
        * @param vertices the set of vertex coordinates for which to find the box
        * @return the bounding box {@link Box}
        */
        template<typename Container>
        static Box getBoundingBox(const Container &vertices) {
            using namespace util;
            return Box(findMinMaxCoordinates<Container, Array3>(vertices));
        }

        /**
        * Takes points of a face of a polyhedron and clips them to this box. If all the points lie in the box no changes are made but if points lie outside of the box they are linearly interpolated onto the box.
        * Uses the Sutherland-Hodgman-Algorithm.
        * @param points The corner points of the face to be clipped.
        * @return The new corner points of the clipped face.
        */
        [[nodiscard]] std::vector<Array3> clipToVoxel(const std::array<Array3, 3> &points) const {
            using namespace util;
            //use clipped as the input vector because the inner for loop swaps input and clipped each iteration,
            //since each iteration needs the output of the previous iteration as input.
            std::vector<Array3> clipped(points.cbegin(), points.cend());
            std::vector<Array3> input{};
            input.reserve(points.size());
            //every plane defined by the maxPoint has to flip its normal because the normals have to point inside the bounding box.
            bool flipPlane = false;
            for (const Direction direction: {Direction::X, Direction::Y, Direction::Z}) {
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

        explicit Box(const std::pair<Array3, Array3> &pair)
            : minPoint{pair.first}, maxPoint{pair.second} {
        }
        Box()
            : minPoint{0.0, 0.0, 0.0}, maxPoint{0.0, 0.0, 0.0} {
        }

    private:
        /**
         * Takes a plane and a set of vertices and clips them accordingly.
         * Used as a sub procedure by the Sutherland-Hodgman-Algorithm.
         * @param plane The plane to split the vertices by
         * @param flipPlaneNormal Specifies which side of the plane is inside (In the direction or opposite of the plane normal).
         * @param source The vertices to be transformed to lie on the inside of the plane.
         * @param dest The transformed vertices.
         */
        static void clipToVoxelPlane(const Plane &plane, const bool flipPlaneNormal, const std::vector<Array3> &source, std::vector<Array3> &dest) {
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
    };

    /**
     * A set that stores indices of the faces vector in the KDTree. This effectively corresponds to a set of triangles. For performance purposes a std::vector is used instead of a std::set.
     */
    using TriangleIndexList = std::vector<size_t>;

    /**
    * Triangle sets contained in an array. Used by the KDTree to divide a bounding boxes included triangles into smaller subsets. For the semantic purpose of the contained sets please refer to the comments in the usage context.
     */
    template<size_t Number>
    using TriangleIndexLists = std::array<std::unique_ptr<TriangleIndexList>, Number>;

    /**
    * Used by {@link PlaneEvent} to position the face that generated the event relative to the generated plane.
    */
    enum class PlaneEventType {
        ending = 0,
        planar = 1,
        starting = 2,
    };

    /**
     * Generated when traversing the vector of faces and building their candidate planes.
     */
    struct PlaneEvent {
        PlaneEventType type;
        /**
         * The candidate plane suggested by the face included in this struct.
         */
        Plane plane;
        /**
         * The index of the face that generated this candidate plane.
         */
        unsigned int faceIndex;

        PlaneEvent(const PlaneEventType type, const Plane plane, const unsigned faceIndex)
            : type{type}, plane{plane}, faceIndex{faceIndex} {
        }

        /**
         * Less operator used for sorting an PlaneEvent vector.
         * @param other the PlaneEvent to compare this to.
         * @return true if this should precede the other argument.
         */
        bool operator<(const PlaneEvent &other) const {
            if (this->plane.axisCoordinate != other.plane.axisCoordinate) {
                return this->plane.axisCoordinate < other.plane.axisCoordinate;
            }
            if (this->plane.orientation == other.plane.orientation) {
                return this->type < other.type;
            }
            return static_cast<unsigned>(this->plane.orientation) < static_cast<unsigned>(other.plane.orientation);
        }

        /**
         *Equality operator used for testing purposes
         */
        bool operator==(const PlaneEvent &other) const {
            return type == other.type && plane == other.plane && faceIndex == other.faceIndex;
        }
    };

    /**
     * A list of PlaneEvents.
    */
    using PlaneEventList = std::vector<PlaneEvent>;

    /**
    * An array of PlaneEventLists.
    */
    template<size_t Number>
    using PlaneEventLists = std::array<std::unique_ptr<PlaneEventList>, Number>;

    /**
     * Extracts the indices of the triangle faces that are referenced by the given PlaneEvents.
     * @param events The PlaneEvents containing information about the faces.
     * @return A list of face indices.
     */
    static TriangleIndexList convertEventsToFaces(const std::variant<TriangleIndexList, PlaneEventList> &events) {
        if (std::holds_alternative<TriangleIndexList>(events)) {
            return std::get<TriangleIndexList>(events);
        }
        const auto &eventList{std::get<PlaneEventList>(events)};
        TriangleIndexList triangles{};
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
        std::for_each(eventList.cbegin(), eventList.cend(), insertIfAbsent);
        triangles.shrink_to_fit();
        return triangles;
    }

    static size_t countFaces(const std::variant<TriangleIndexList, PlaneEventList> &triangles) {
        return std::visit(util::overloaded{
                                  [](const TriangleIndexList &indexList) {
                                      return indexList.size();
                                  },
                                  [](const PlaneEventList &eventList) {
                                      size_t count{0};
                                      std::unordered_set<size_t> processedFaces{};
                                      std::for_each(eventList.cbegin(), eventList.cend(), [&processedFaces, &count](const auto &planeEvent) {
                                          if (processedFaces.find(planeEvent.faceIndex) == processedFaces.end()) {
                                              processedFaces.insert(planeEvent.faceIndex);
                                              count++;
                                          }
                                      });
                                      return count;
                                  }},
                          triangles);
    }


    /**
        * An iterator transforming face indices to vertices and returning both.
        * This function returns a pair of transform iterators (first = begin(), second = end()).
        * @param begin begin iterator of the face indice vector to transform.
        * @param end end iterator of the face indice vector to transform.
        * @param vertices the vector of vertices to look up the indices obtained from the faces vector.
        * @param faces the faces vector to lookup face indices.
        * @return pair of transform iterators.
        */
    [[nodiscard]] static auto transformIterator(const std::vector<size_t>::const_iterator begin, const std::vector<size_t>::const_iterator end, const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces) {
        //The offset must be captured by value to ensure its lifetime!
        const auto lambdaApplication = [&vertices, &faces](size_t faceIndex) {
            const auto &face = faces[faceIndex];
            Array3Triplet vertexTriplet = {
                    vertices[face[0]],
                    vertices[face[1]],
                    vertices[face[2]]};
            return std::make_pair(faceIndex, vertexTriplet);
        };


        auto first = thrust::make_transform_iterator(begin, lambdaApplication);
        auto last = thrust::make_transform_iterator(end, lambdaApplication);
        return std::make_pair(first, last);
    }

    /**
    * The distance to the tree's root from this node. Used to limit the depth and the size of the tree.
    * @param nodeId The id of the node to determine the depth (distance to root node) of.
    */
    inline size_t recursionDepth(const size_t nodeId) {
        //t: depth of the current node, N: amount of nodes in the tree if tree is complete
        // $ N = 2^(t+1)-1 $ (geometric series formula for partial sums), assume $ N >= idx + 1 $
        return static_cast<size_t>(std::ceil(std::log2(nodeId + 2))) - 1;
    }

}// namespace polyhedralGravity