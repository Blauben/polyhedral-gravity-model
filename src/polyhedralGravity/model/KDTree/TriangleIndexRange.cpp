#include "polyhedralGravity/model/KDTree/TriangleIndexRange.h"

namespace polyhedralGravity {
    size_t TriangleIndexRange::size() const {
        return end_idx - begin_idx + duplicate_faces->size();
    }

    bool TriangleIndexRange::empty() const {
        return begin_idx == end_idx && duplicate_faces->empty();
    }

    TriangleIndexRange::iterator TriangleIndexRange::begin() const {
        return {faces, duplicate_faces, begin_idx, end_idx};
    }

    TriangleIndexRange::iterator TriangleIndexRange::end() const {
        return iterator(faces, duplicate_faces, begin_idx, end_idx).end();
    }

    TriangleIndexRange::const_iterator TriangleIndexRange::cbegin() const {
        return {faces, duplicate_faces, begin_idx, end_idx};
    }

    TriangleIndexRange::const_iterator TriangleIndexRange::cend() const {
        return const_iterator(faces, duplicate_faces, begin_idx, end_idx).end();
    }

    TriangleIndexRange::TriangleIndexRange(std::shared_ptr<std::vector<IndexArray3>> faces)
        : begin_idx{0}, end_idx{faces->size()}, faces{std::move(faces)} {
    }


    TriangleIndexRange::TriangleIndexRange(const TriangleIndexRange &other, size_t begin, size_t end) {
        this->faces = other.faces;
        this->begin_idx = begin;
        this->end_idx = end;
    }
}