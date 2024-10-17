#pragma once

#include <utility>
#include <memory>

#include "polyhedralGravity/model/GravityModelData.h"
namespace polyhedralGravity {
    /**
         * A range of triangles on the faces vector in the KDTree. This effectively corresponds to a set of triangles. Internally iterators are constructed on demand.
         */
    class TriangleIndexRange {
        struct iterator {
            friend class TriangleIndexRange;
            using iterator_category = std::forward_iterator_tag;
            using difference_type = size_t;
            using value_type = IndexArray3;
            using pointer = IndexArray3*;
            using reference = IndexArray3&;
        private:
            size_t index;
            const size_t face_begin;
            const size_t face_end;
            std::shared_ptr<std::vector<IndexArray3>> faces;
            std::shared_ptr<std::vector<IndexArray3>> duplicate_faces;
            iterator(std::shared_ptr<std::vector<IndexArray3>> faces, std::shared_ptr<std::vector<IndexArray3>> duplicate_faces, const size_t face_begin, const size_t face_end) : index{face_begin}, face_begin{face_begin}, face_end{face_end}, faces{std::move(faces)}, duplicate_faces{std::move(duplicate_faces)} {}
            iterator& end() {index = face_end + duplicate_faces->size(); return *this;}

        public:
            reference operator*() const {
                if (index < face_end) {
                    return faces->at(index);
                }
                if (index - face_end < duplicate_faces->size()) {
                    return duplicate_faces->at(index - face_end);
                }
                throw std::out_of_range("Iterator out of bounds in TriangleIndexRange");
            }
            pointer operator->() const {
                if (index < face_end) {
                    return &faces->at(index);
                }
                if (index - face_end < duplicate_faces->size()) {
                    return &duplicate_faces->at(index - face_end);
                }
                throw std::out_of_range("Iterator out of bounds in TriangleIndexRange");
            }
            iterator operator++(int) {auto ret = *this; index++; return ret;}
            iterator& operator++() { index++; return *this;}
            bool operator==(const iterator& other) const {return index == other.index;}
            bool operator!=(const iterator& other) const {return index != other.index;}
        };

        struct const_iterator {
            friend class TriangleIndexRange;
            using iterator_category = std::forward_iterator_tag;
            using difference_type = size_t;
            using value_type = IndexArray3;
            using pointer = const IndexArray3*;
            using reference = const IndexArray3&;
        private:
            size_t index;
            const size_t face_begin;
            const size_t face_end;
            std::shared_ptr<const std::vector<IndexArray3>> faces;
            std::shared_ptr<const std::vector<IndexArray3>> duplicate_faces;
            const_iterator(std::shared_ptr<std::vector<IndexArray3>> faces, std::shared_ptr<std::vector<IndexArray3>> duplicate_faces, const size_t face_begin, const size_t face_end) : index{face_begin}, face_begin{face_begin}, face_end{face_end}, faces{std::move(faces)}, duplicate_faces{std::move(duplicate_faces)} {}
            const_iterator& end() {index = face_end + duplicate_faces->size(); return *this;}

        public:
            reference operator*() const {
                if (index < face_end) {
                    return faces->at(index);
                }
                if (index - face_end < duplicate_faces->size()) {
                    return duplicate_faces->at(index - face_end);
                }
                throw std::out_of_range("Iterator out of bounds in TriangleIndexRange");
            }
            pointer operator->() const {
                if (index < face_end) {
                    return &faces->at(index);
                }
                if (index - face_end < duplicate_faces->size()) {
                    return &duplicate_faces->at(index - face_end);
                }
                throw std::out_of_range("Iterator out of bounds in TriangleIndexRange");
            }
            const_iterator operator++(int) {auto ret = *this; index++; return ret;}
            const_iterator& operator++() { index++; return *this;}
            bool operator==(const const_iterator& other) const {return index == other.index;}
            bool operator!=(const const_iterator& other) const {return index != other.index;}
        };


    public:
        /**
         * Start index of triangle range (inclusive).
         */
        size_t begin_idx;
        /**
         * End of triangle range (exclusive). Treated as vector_end_index = vector.size() - end_idx
         * This ensures that additional elements can be inserted
         */
        size_t end_idx;
        /**
         * The faces that connect the vertices to render the Polyhedron.
         */
        std::shared_ptr<std::vector<IndexArray3>> faces;

        /**
         * To avoid having to enlarge the source vector. Additional faces found to be in two triangle sets are stored here.
         */
        std::shared_ptr<std::vector<IndexArray3>> duplicate_faces{std::make_shared<std::vector<IndexArray3>>()};

        /**
         * Calculates the size of the range. Corresponds to the number of bound faces.
         * @return the number of faces.
         */
        [[nodiscard]] size_t size() const;

        /**
         * Checks for emptiness of the range.
         * @return true if the range has no bound faces.
         */
        [[nodiscard]] bool empty() const ;

        /**
         * @return The begin iterator for the bound faces.
         */
        [[nodiscard]] iterator begin() const ;

        /**
        * @return The end iterator for the bound faces.
        */
        [[nodiscard]] iterator end() const ;

        /**
         * @return The constant begin iterator for the bound faces.
         */
        [[nodiscard]] const_iterator cbegin() const ;

        /**
         * @return The constant end iterator for the bound faces.
         */
        [[nodiscard]] const_iterator cend() const ;

        /**
         * Constructs a range that default initializes to include all passed faces.
         * @param faces The faces vector to base the range on.
         */
        explicit TriangleIndexRange(std::shared_ptr<std::vector<IndexArray3>> faces);

        /**
         * Copies a range and assigns new boundary indices to it.
         * @param other The range to be copied.
         * @param begin New start index.
         * @param end New end index.
         */
        TriangleIndexRange(const TriangleIndexRange &other, size_t begin, size_t end);
    };

}