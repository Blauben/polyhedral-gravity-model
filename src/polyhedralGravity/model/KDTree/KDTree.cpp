#include "polyhedralGravity/model/KDTree/KDTree.h"

namespace polyhedralGravity {

    KDTree::KDTree(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces)
        : _vertices{vertices}, _faces{faces} {
        //on initialization of the tree a single bounding box which includes all the faces of the polyhedron is generated. Both the list of included faces and the parameters of the box are written to the split paramters
        TriangleIndexRange boundFaces = {0, _faces.size(), _faces};
        const Box boundingBox{getBoundingBox(vertices)};
        this->_splitParam = std::make_unique<SplitParam>(_vertices, _faces, boundFaces, boundingBox, Direction::X);
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
        auto set{std::set<Array3>()};
        this->getFaceIntersections(origin, ray, set);
        return set.size();
    }

    void KDTree::getFaceIntersections(const Array3 &origin, const Array3 &ray, std::set<Array3> &intersections) {
        this->getRootNode().getFaceIntersections(origin, ray, intersections);
    }


    std::tuple<Plane, double, TriangleIndexRanges<2>> KDTree::findPlane(const SplitParam &param) {// O(N^2) implementation
        //initialize the default plane and make it costly
        double cost = std::numeric_limits<double>::infinity();
        Plane optPlane{0, param.splitDirection};
        //store the triangleSets that are implicitly generated during plane testing for later use.
        TriangleIndexRanges<2> optTriangleIndexRanges{TriangleIndexRange{param.faces.begin(), param.faces.begin()}, TriangleIndexRange{param.faces.end(), param.faces.end()}};
        //each vertex proposes a split plane candidate: test for each of them, store them in buffer set to avoid duplicate testing
        std::unordered_set<double> testedPlaneCoordinates{};
        auto [bound_start, bound_end] = param.indexBoundFaces;
        std::for_each(bound_start, bound_end, [&param, &optPlane, &cost, &optTriangleIndexRanges, &testedPlaneCoordinates](const IndexArray3 &face) {
            const auto [minPoint, maxPoint] = getBoundingBox(faceToVertices(face, param.vertices));
            for (const auto planeSurfacePoint: {minPoint, maxPoint}) {
                //constructs the plane that goes through a vertex lying on the bounding box of the face to be checked and spans in a specified direction.
                Plane candidatePlane{planeSurfacePoint[static_cast<int>(param.splitDirection)], param.splitDirection};
                //continue if plane has already been tested
                if (testedPlaneCoordinates.find(candidatePlane.first) != testedPlaneCoordinates.cend()) {
                    continue;
                }
                testedPlaneCoordinates.emplace(candidatePlane.first);
                //evaluate the candidate plane and store if it is better than the currently stored result
                if (auto [candidateCost, TriangleIndexRanges] = costForPlane(param, candidatePlane); candidateCost < cost) {
                    cost = candidateCost;
                    optPlane = candidatePlane;
                    optTriangleIndexRanges = TriangleIndexRanges;
                }
            }
        });
        return std::make_tuple(std::move(optPlane), cost, std::move(optTriangleIndexRanges));
    }

    Box KDTree::getBoundingBox(const std::vector<Array3> &vertices) {
        using namespace polyhedralGravity;
        //return empty box centered at the origin if no vertices provided
        if (vertices.empty()) {
            return {{0, 0, 0}, {0, 0, 0}};
        }
        //initialize values from the array -> even if only one vertex is provided the box is still correct without executing the loop.
        Array3 min = vertices[0];
        Array3 max = vertices[0];
        //test each vertex for proximity to the origin and find minima and maxima
        std::for_each(vertices.cbegin() + 1, vertices.cend(), [&min, &max](const Array3 &vertex) {
            //test each dimension separately -> Calculates the corner points of the smallest bounding box gradually
            for (int i = 0; i < vertex.size(); i++) {
                min[i] = std::min(min[i], vertex[i]);
                max[i] = std::max(max[i], vertex[i]);
            }
        });
        return {min, max};
    }

    std::vector<Array3> KDTree::faceToVertices(const IndexArray3 &face, const std::vector<Array3> &vertices) {
        return {vertices[face[0]], vertices[face[1]], vertices[face[2]]};
    }

    std::pair<Box, Box> KDTree::splitBox(const Box &box, const Plane &plane) {
        //clone the original box two times -> modify clones to become child boxes defined by the splitting plane
        Box box1{box};
        Box box2{box};
        const Direction &axis{plane.second};
        //box.first == min ; box.second == max -> Shift edges of the boxes to match the plane
        box1.second[static_cast<int>(axis)] = plane.first;
        box2.first[static_cast<int>(axis)] = plane.first;
        return std::make_pair(box1, box2);
    }

    std::pair<const double, TriangleIndexRanges<2>> KDTree::costForPlane(const SplitParam &param, const Plane &plane) {
        //Checks if the split plane is one of the faces of the bounding box, if so the split is useless
        if (plane.first == param.boundingBox.first[static_cast<int>(plane.second)] || plane.first == param.boundingBox.second[static_cast<int>(plane.second)]) {
            return {std::numeric_limits<double>::infinity(), {param.indexBoundFaces, param.indexBoundFaces}};//will be discarded later because not splitting is cheaper (finitely many nodes!) than using this plane (infinite cost)
        }
        //calculate parameters for Surface Area Heuristic (SAH): childBoxSurfaceAreas; number of contained triangles in each box
        auto [box1, box2] = splitBox(param.boundingBox, plane);
        //equalT are triangles lying in the plane (not in the boxes)
        auto [lessT, greaterT, equalT] = containedTriangles(param, plane);
        const double surfaceAreaBounding = surfaceAreaOfBox(param.boundingBox);
        const double surfaceArea1 = surfaceAreaOfBox(box1);
        const double surfaceArea2 = surfaceAreaOfBox(box2);
        //evaluate SAH: Include equalT once in each box and record option with minimum cost
        double costLesser = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * (static_cast<double>(lessT.size() + equalT.size())) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>(greaterT.size()));
        //if empty space is cut off, reduce cost by 20%
        costLesser *= lessT.size() + equalT.size() == 0 || greaterT.empty() ? 0.8 : 1;
        double costUpper = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * static_cast<double>(lessT.size()) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>((greaterT.size() + equalT.size())));
        //if empty space is cut off, reduce cost by 20%
        costUpper *= greaterT.size() + equalT.size() == 0 || lessT.empty() ? 0.8 : 1;
        if (costLesser <= costUpper) {
            //include equalT in the boxes triangles for further subdivision
            lessT.end = equalT.end;
            return {costLesser, {lessT, greaterT}};
        }
        //include equalT in the boxes triangles for further subdivision
        greaterT.begin = equalT.begin;
        return {costUpper, {lessT, greaterT}};
    }

    double KDTree::surfaceAreaOfBox(const Box &box) {
        const double width = std::abs(box.second[0] - box.first[0]);
        const double length = std::abs(box.second[1] - box.first[1]);
        const double height = std::abs(box.second[2] - box.first[2]);
        return 2 * (width * length + width * height + length * height);
    }

    TriangleIndexRanges<3> KDTree::containedTriangles(const SplitParam &param, const Plane &split) {
        using namespace polyhedralGravity;
        //define three sets of triangles: closer to the origin, further away, in the plane
        std::vector<IndexArray3> face_less{param.indexBoundFaces.size() / 2};
        std::vector<IndexArray3> face_greater{param.indexBoundFaces.size() / 2};
        std::vector<IndexArray3> face_equal{};

        //perform check for every triangle contained in this node's bounding box.
        std::for_each(param.indexBoundFaces.begin, param.indexBoundFaces.end, [&param, &split, &face_greater, &face_less, &face_equal](const IndexArray3 &face) {
            bool less{false}, greater{false}, equal{false};
            //transform a triangle into the three vertices it comprises
            auto vertices{faceToVertices(face, param.vertices)};
            for (const Array3 vertex: vertices) {
                //vertex is closer to the origin than the plane
                if (vertex[static_cast<int>(split.second)] < split.first) {
                    less = true;
                } else if (vertex[static_cast<int>(split.second)] > split.first) {//vertex is farther away of the origin than the plane
                    greater = true;
                } else if (vertex[static_cast<int>(split.second)] == split.first) {//vertex lies on the plane
                    equal = true;
                }
            }

            //all vertices of the triangle lie in the plane -> triangle lies in the plane
            if (!less && !greater && equal) {
                face_equal.push_back(face);
                return;
            }
            //triangle has area in the greater bounding box and needs to be checked there for intersections
            if (greater) {
                face_greater.push_back(face);
            }
            //triangle has area in the closer bounding box and needs to be checked there for intersections
            if (less) {
                face_less.push_back(face);
            }
        });

        auto endLess = std::copy(face_less.cbegin(), face_less.cend(), param.faces.begin());
        auto endEqual = std::copy(face_equal.cbegin(), face_equal.cend(), endLess);
        auto endGreater = std::copy(face_greater.cbegin(), face_greater.cend(), endEqual);
        return {TriangleIndexRange(param.indexBoundFaces.begin, endLess), TriangleIndexRange(endEqual, endGreater), TriangleIndexRange(endLess, endEqual)};
    }

}// namespace polyhedralGravity