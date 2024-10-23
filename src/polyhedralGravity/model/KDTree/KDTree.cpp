#include "polyhedralGravity/model/KDTree/KDTree.h"

namespace polyhedralGravity {

    //on initialization of the tree a single bounding box which includes all the faces of the polyhedron is generated. Both the list of included faces and the parameters of the box are written to the split parameters
    KDTree::KDTree(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces)
        : _vertices{vertices}, _faces{faces}, _splitParam{std::make_unique<SplitParam>(_vertices, _faces, getBoundingBox(vertices), Direction::X)} {
    }

    TreeNode &KDTree::getRootNode() {
        //if the node has already been generated, don't do it again. Instead let the factory determine the TreeNode subclass based on the optimal split.
        if (!this->_rootNode) {
            this->_rootNode = TreeNodeFactory::treeNodeFactory(*std::move(this->_splitParam));
        }
        return *this->_rootNode;
    }

    size_t KDTree::countIntersections(const Array3 &origin, const Array3 &ray) {
        //it's possible that a single intersection point is on the edge between two triangles. The point would be counted twice if the intersection points were not documented -> use of std::set
        std::set<Array3> set{};
        this->getFaceIntersections(origin, ray, set);
        return set.size();
    }

    void KDTree::getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) {
        this->getRootNode().getFaceIntersections(origin, ray, intersections);
    }

    // O(N*log^2(N)) implementation
    std::tuple<Plane, double, TriangleIndexLists<2>> KDTree::findPlane(const SplitParam &splitParam) {//TODO iterate over all
        //initialize the default plane and make it costly
        double cost = std::numeric_limits<double>::infinity();
        Plane optPlane{0, splitParam.splitDirection};
        //store the triangleSets that are implicitly generated during plane testing for later use.
        TriangleIndexLists<2> optTriangleIndexLists{};
        //each vertex proposes a split plane candidate: create an event and queue it in the buffer
        std::vector<PlaneEvent> events{};
        auto [vertex3_begin, vertex3_end] = transformIterator(splitParam.indexBoundFaces.cbegin(), splitParam.indexBoundFaces.cend(), splitParam.vertices, splitParam.faces);
        std::for_each(vertex3_begin, vertex3_end, [&splitParam, &optPlane, &cost, &optTriangleIndexLists, &events](const auto &indexAndTriplet) {
            const auto [index, triplet] = indexAndTriplet;
            const auto [minPoint, maxPoint] = getBoundingBox<std::array<Array3, 3>>(triplet);
            // if the the triangle is perpendicular to the split direction, generate a planar event with the candidate plane in which the triangle lies
            if (minPoint[splitParam.splitDirection] == maxPoint[splitParam.splitDirection]) {
                events.emplace_back(
                                .type = PlaneEventType::planar,
                                .plane = Plane{
                                        .axisCoordinate = minPoint[splitParam.splitDirection],
                                        .orientation = splitParam.splitDirection},
                                .faceIndex = index);
                return;
            }
            //else create a starting and ending event consisting of the planes defined by the min and max points of the face's bounding box.
            events.emplace_back(
                            .type = PlaneEventType::starting,
                            .plane = Plane{
                                    .axisCoordinate = minPoint[splitParam.splitDirection],
                                    .orientation = splitParam.splitDirection},
                            .faceIndex = index);
            events.emplace_back(
                            .type = PlaneEventType::ending,
                            .plane = Plane{
                                    .axisCoordinate = maxPoint[splitParam.splitDirection],
                                    .orientation = splitParam.splitDirection},
                            .faceIndex = index);
        });
        std::sort(events.begin(), events.end());
        size_t trianglesLeft{0}, trianglesRight{splitParam.indexBoundFaces.size()}, trianglesPlanar{0};
        for (int i{0}; i < events.size(); i++) {
            Plane &candidatePlane = events[i].plane;
            size_t p_start{0}, p_end{0}, p_planar{0};
            while (i < events.size() && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::ending) {
                p_end++;
                i++;
            }
            while (i < events.size() && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::planar) {
                p_planar++;
                i++;
            }
            while (i < events.size() && events[i].plane.axisCoordinate == candidatePlane.axisCoordinate && events[i].type == PlaneEventType::starting) {
                p_start++;
                i++;
            }
            trianglesPlanar = p_planar;
            trianglesRight -= p_planar + p_end;
            auto [cost, minSideChosen] = costForPlane(splitParam.boundingBox, candidatePlane, p_start, p_end, p_planar);
            trianglesLeft += p_planar + p_start;
            trianglesPlanar = 0;
            //TODO: continue here -> update best plane
        }
        return std::make_tuple(optPlane, cost, std::move(optTriangleIndexLists));
    }

    template<typename Container>
    Box KDTree::getBoundingBox(const Container &vertices) {
        using namespace util;
        return Box(findMinMaxCoordinates<Container, Array3>(vertices));
    }

    std::pair<Box, Box> KDTree::splitBox(const Box &box, const Plane &plane) {
        //clone the original box two times -> modify clones to become child boxes defined by the splitting plane
        Box box1{box};
        Box box2{box};
        const Direction &axis{plane.orientation};
        //Shift edges of the boxes to match the plane
        box1.maxPoint[static_cast<int>(axis)] = plane.axisCoordinate;
        box2.minPoint[static_cast<int>(axis)] = plane.axisCoordinate;
        return std::make_pair(box1, box2);
    }

    std::pair<const double, bool> KDTree::costForPlane(const Box &boundingBox, const Plane &plane, const size_t trianglesMin, const size_t trianglesMax, const size_t trianglesPlanar) {
        //Checks if the split plane is one of the faces of the bounding box, if so the split is useless
        if (plane.axisCoordinate == boundingBox.minPoint[static_cast<int>(plane.orientation)] || plane.axisCoordinate == boundingBox.maxPoint[static_cast<int>(plane.orientation)]) {
            //will be discarded later because not splitting is cheaper (finitely many nodes!) than using this plane (infinite cost)
            return {std::numeric_limits<double>::infinity(), false};
        }
        //calculate parameters for Surface Area Heuristic (SAH): childBoxSurfaceAreas; number of contained triangles in each box
        auto [box1, box2] = splitBox(boundingBox, plane);
        //equalT are triangles lying in the plane (not in the boxes)
        const double surfaceAreaBounding = surfaceAreaOfBox(boundingBox);
        const double surfaceArea1 = surfaceAreaOfBox(box1);
        const double surfaceArea2 = surfaceAreaOfBox(box2);
        //evaluate SAH: Include equalT once in each box and record option with minimum cost
        double costLesser = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * (static_cast<double>(trianglesMin + trianglesPlanar)) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>(trianglesMax));
        //if empty space is cut off, reduce cost by 20%
        costLesser *= trianglesMin + trianglesPlanar == 0 || trianglesMax == 0 ? 0.8 : 1.0;
        double costUpper = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * static_cast<double>(trianglesMin) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>((trianglesMax + trianglesPlanar)));
        //if empty space is cut off, reduce cost by 20%
        costUpper *= trianglesMax + trianglesPlanar == 0 || trianglesMin == 0 ? 0.8 : 1.0;
        if (costLesser <= costUpper) {
            return {costLesser, true};
        }
        return {costUpper, false};
    }

    double KDTree::surfaceAreaOfBox(const Box &box) {
        const double width = std::abs(box.maxPoint[0] - box.minPoint[0]);
        const double length = std::abs(box.maxPoint[1] - box.minPoint[1]);
        const double height = std::abs(box.maxPoint[2] - box.minPoint[2]);
        return 2 * (width * length + width * height + length * height);
    }

    TriangleIndexLists<3> KDTree::containedTriangles(const SplitParam &param, const Plane &split) {
        using namespace polyhedralGravity;
        //define three sets of triangles: closer to the origin, further away, in the plane
        auto index_less = std::make_unique<TriangleIndexList>(param.indexBoundFaces.size() / 2);
        auto index_greater = std::make_unique<TriangleIndexList>(param.indexBoundFaces.size() / 2);
        auto index_equal = std::make_unique<TriangleIndexList>();


        //perform check for every triangle contained in this node's bounding box.
        //transform faceIndices into the vertices
        auto [begin, end] = transformIterator(param.indexBoundFaces.cbegin(), param.indexBoundFaces.cend(), param.vertices, param.faces);
        std::for_each(begin, end, [&split, &index_greater, &index_less, &index_equal](std::pair<unsigned long, Array3Triplet> pair) {
            auto [faceIndex, vertices] = pair;
            bool less{false}, greater{false}, equal{false};
            for (const Array3 vertex: vertices) {
                //vertex is closer to the origin than the plane
                if (vertex[static_cast<int>(split.orientation)] < split.axisCoordinate) {
                    less = true;
                    //vertex is farther away of the origin than the plane
                } else if (vertex[static_cast<int>(split.orientation)] > split.axisCoordinate) {
                    greater = true;
                    //vertex lies on the plane
                }
            }

            //all vertices of the triangle lie in the plane -> triangle lies in the plane
            if (!less && !greater) {
                index_equal->push_back(faceIndex);
                return;
            }
            //triangle has area in the greater bounding box and needs to be checked there for intersections
            if (greater) {
                index_greater->push_back(faceIndex);
            }
            //triangle has area in the closer bounding box and needs to be checked there for intersections
            if (less) {
                index_less->push_back(faceIndex);
            }
        });
        return std::array{(std::move(index_less)), (std::move(index_greater)), (std::move(index_equal))};
    }

}// namespace polyhedralGravity