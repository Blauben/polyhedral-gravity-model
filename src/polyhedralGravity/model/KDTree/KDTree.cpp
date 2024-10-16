#include "polyhedralGravity/model/KDTree/KDTree.h"

namespace polyhedralGravity {

    KDTree::KDTree(const std::vector<Array3> &vertices, const std::vector<IndexArray3> &faces)
        : _vertices{vertices}, _faces{faces} {
        //on initialization of the tree a single bounding box which includes all the faces of the polyhedron is generated. Both the list of included faces and the parameters of the box are written to the split paramters
        TriangleIndexList boundFaces(faces.size());
        std::iota(boundFaces.begin(), boundFaces.end(), 0);
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


    std::tuple<Plane, double, TriangleIndexLists<2>> KDTree::findPlane(const SplitParam &param) {// O(N^2) implementation
        //initialize the default plane and make it costly
        double cost = std::numeric_limits<double>::infinity();
        Plane optPlane{0, param.splitDirection};
        //store the triangleSets that are implicitly generated during plane testing for later use.
        TriangleIndexLists<2> optTriangleIndexLists{};
        //each vertex proposes a split plane candidate: test for each of them, store them in buffer set to avoid duplicate testing
        std::unordered_set<double> testedPlaneCoordinates{};
        std::for_each(param.indexBoundFaces.cbegin(), param.indexBoundFaces.cend(), [&param, &optPlane, &cost, &optTriangleIndexLists, &testedPlaneCoordinates](const size_t faceIndex) {
            const auto &face = param.faces[faceIndex];
            for (const size_t &vertexIndex: face) {
                Plane candidatePlane{param.vertices[vertexIndex][static_cast<int>(param.splitDirection)], param.splitDirection};//constructs the plane that goes through the vertex at index vertexIndex and spans in a specified direction.
                if (testedPlaneCoordinates.find(candidatePlane.first) != testedPlaneCoordinates.cend()) {                       //continue if plane has already been tested
                    continue;
                }
                testedPlaneCoordinates.emplace(candidatePlane.first);
                if (auto [candidateCost, triangleIndexLists] = costForPlane(param, candidatePlane); candidateCost < cost) {//evaluate the candidate plane and store if it is better than the currently stored result
                    cost = candidateCost;
                    optPlane = candidatePlane;
                    optTriangleIndexLists = std::move(triangleIndexLists);
                }
            }
        });
        return std::make_tuple(std::move(optPlane), cost, std::move(optTriangleIndexLists));
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
        std::for_each(vertices.cbegin() + 1, vertices.cend(), [&min, &max](const Array3 &vertex) {//test each vertex for proximity to the origin and find minima and maxima
            for (int i = 0; i < vertex.size(); i++) {                                             //test each dimension separately -> Calculates the corner points of the smallest bounding box gradually
                min[i] = std::min(min[i], vertex[i]);
                max[i] = std::max(max[i], vertex[i]);
            }
        });
        return {min, max};
    }

    std::pair<Box, Box> KDTree::splitBox(const Box &box, const Plane &plane) {
        //clone the original box two times -> modify clones to become child boxes defined by the splitting plane
        Box box1{box};
        Box box2{box};
        const Direction &axis{plane.second};
        box1.second[static_cast<int>(axis)] = plane.first;//box.first == min ; box.second == max -> Shift edges of the boxes to match the plane
        box2.first[static_cast<int>(axis)] = plane.first;
        return std::make_pair(box1, box2);
    }

    std::pair<const double, TriangleIndexLists<2>> KDTree::costForPlane(const SplitParam &param, const Plane &plane) {
        //Checks if the split plane is one of the faces of the bounding box, if so the split is useless
        if (plane.first == param.boundingBox.first[static_cast<int>(plane.second)] || plane.first == param.boundingBox.second[static_cast<int>(plane.second)]) {
            return std::make_pair(std::numeric_limits<double>::infinity(), TriangleIndexLists<2>{});//will be discarded later because not splitting is cheaper (finitely many nodes!) than using this plane (infinite cost)
        }
        //calculate parameters for Surface Area Heuristic (SAH): childBoxSurfaceAreas; number of contained triangles in each box
        auto [box1, box2] = splitBox(param.boundingBox, plane);
        auto [lessT, greaterT, equalT] = containedTriangles(param, plane);//equalT are triangles lying in the plane (not in the boxes)
        const double surfaceAreaBounding = surfaceAreaOfBox(param.boundingBox);
        const double surfaceArea1 = surfaceAreaOfBox(box1);
        const double surfaceArea2 = surfaceAreaOfBox(box2);
        //evaluate SAH: Include equalT once in each box and record option with minimum cost
        double costLesser = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * (static_cast<double>(lessT->size() + equalT->size())) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>(greaterT->size()));
        costLesser *= lessT->size() + equalT->size() == 0 || greaterT->empty() ? 0.8 : 1;//if empty space is cut off, reduce cost by 20%
        double costUpper = traverseStepCost + triangleIntersectionCost * ((surfaceArea1 / surfaceAreaBounding) * static_cast<double>(lessT->size()) + (surfaceArea2 / surfaceAreaBounding) * static_cast<double>((greaterT->size() + equalT->size())));
        costUpper *= greaterT->size() + equalT->size() == 0 || lessT->empty() ? 0.8 : 1;//if empty space is cut off, reduce cost by 20%
        if (costLesser <= costUpper) {
            lessT->insert(lessT->cend(), equalT->cbegin(), equalT->cend());//include equalT in the boxes triangles for further subdivision
            return std::make_pair(costLesser, std::array{std::move(lessT), std::move(greaterT)});
        }
        greaterT->insert(greaterT->cend(), equalT->cbegin(), equalT->cend());//include equalT in the boxes triangles for further subdivision
        return {costUpper, std::array{(std::move(lessT)), (std::move(greaterT))}};
    }

    double KDTree::surfaceAreaOfBox(const Box &box) {
        const double width = std::abs(box.second[0] - box.first[0]);
        const double length = std::abs(box.second[1] - box.first[1]);
        const double height = std::abs(box.second[2] - box.first[2]);
        return 2 * (width * length + width * height + length * height);
    }

    TriangleIndexLists<3> KDTree::containedTriangles(const SplitParam &param, const Plane &split) {
        using namespace polyhedralGravity;
        //define three sets of triangles: closer to the origin, further away, in the plane
        auto index_less = std::make_unique<TriangleIndexList>(param.indexBoundFaces.size() / 2);
        auto index_greater = std::make_unique<TriangleIndexList>(param.indexBoundFaces.size() / 2);
        auto index_equal = std::make_unique<TriangleIndexList>();

        //perform check for every triangle contained in this node's bounding box.
        std::for_each(param.indexBoundFaces.cbegin(), param.indexBoundFaces.cend(), [&param, &split, &index_greater, &index_less, &index_equal](const size_t faceIndex) {
            const IndexArray3 &face{param.faces[faceIndex]};
            bool less{false}, greater{false}, equal{false};
            //transform a triangle into the three vertices it comprises
            std::array<Array3, 3> vertices{};
            std::transform(face.cbegin(), face.cend(), vertices.begin(), [&param](const size_t vertexIndex) { return param.vertices[vertexIndex]; });
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