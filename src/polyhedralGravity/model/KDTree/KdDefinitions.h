#pragma once

#include "polyhedralGravity/model/GravityModelData.h"

#include <array>
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
     * Number of dimensions for the polyhedron. Also corresponds to the number of elements of the {@link Direction} enum.
     */
    constexpr int DIMENSIONS = 3;

    /**
     * Defines a plane that is parallel to one of the coordinate planes, by taking the fixed axis coordinate value for the plane and the coordinate index ({@link Direction}) that is fixed for every \
     * point on the plane.
     *
     * E.g. Specifying 0.0 and Direction::X would describe the YZ plane that goes through the origin.
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

        explicit Box(const std::pair<Array3, Array3> &pair)
            : minPoint{pair.first}, maxPoint{pair.second} {
        }
        Box()
            : minPoint{0.0, 0.0, 0.0}, maxPoint{0.0, 0.0, 0.0} {
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
        unsigned faceIndex;


        PlaneEvent(const PlaneEventType type, const Plane plane, const unsigned faceIndex)
            : type{type}, plane{plane}, faceIndex{faceIndex} {
        }

        /**
         * Less operator used for sorting an PlaneEvent vector.
         * @param other the PlaneEvent to compare this to.
         * @return true if this should precede the other argument.
         */
        bool operator<(const PlaneEvent &other) const {
            if (this->plane.axisCoordinate == other.plane.axisCoordinate) {
                return this->type < other.type;
            }
            return this->plane.axisCoordinate < other.plane.axisCoordinate;
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
        std::unordered_set<unsigned long> processedFaces{};
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
                                      std::unordered_set<unsigned long> processedFaces{};
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
     * Helper struct to bundle important parameters required for splitting a Polyhedron for better readability.
     */
    struct SplitParam {
        /**
         * The vertices that compose the Polyhedron.
         */
        const std::vector<Array3> &vertices;
        /**
         * The faces that connect the vertices to render the Polyhedron.
         */
        const std::vector<IndexArray3> &faces;
        /**
         * Either an index list of faces that are included in the current bounding box of the KDTree or a list of PlaneEvents containing the information about thr bound faces. Important when building deeper levels of a KDTree.
         */
        std::variant<TriangleIndexList, PlaneEventList> boundFaces;
        /**
         * The current bounding box that should be divided further by the KDTree.
         */
        Box boundingBox;
        /**
         * The direction in which the current bounding box should be divided by further.
         * Refer to {@link Plane} on how to interpret the Direction.
         */
        mutable Direction splitDirection;

        /**
         * Constructor that initializes all fields. Intended for the use with std::make_unique. See {@link SplitParam} fields for further information.
         *
         */
        SplitParam(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces, const Box &boundingBox, const Direction splitDirection)
            : vertices{vertices}, faces{faces}, boundFaces{TriangleIndexList(faces.size())}, boundingBox{boundingBox}, splitDirection{splitDirection} {
            auto &indexList = std::get<TriangleIndexList>(boundFaces);
            std::iota(indexList.begin(), indexList.end(), 0);
        }
    };

    /**
        * An iterator transforming face indices to vertices and returning both.
        * This function returns a pair of transform iterators (first = begin(), second = end()).
        * @param begin begin iterator of the face indice vector to transform.
        * @param end end iterator of the face indice vector to transform.
        * @param vertices the vector of vertices to look up the indices obtained from the faces vector.
        * @param faces the faces vector to lookup face indices.
        * @return pair of transform iterators.
        */
    [[nodiscard]] static auto transformIterator(const std::vector<unsigned long>::const_iterator begin, const std::vector<unsigned long>::const_iterator end, const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces) {
        //The offset must be captured by value to ensure its lifetime!
        const auto lambdaApplication = [&vertices, &faces](unsigned long faceIndex) {
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

}// namespace polyhedralGravity