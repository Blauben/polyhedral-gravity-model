#include "TreeUtils.h"

#include <stdexcept>

template <typename T>
size_t replaceAndShift(std::shared_ptr<std::vector<T>> to, std::vector<T>& from, typename std::vector<T>::iterator position, size_t replaceCharacters) {
    if(replaceCharacters < from.size()) {
        throw std::invalid_argument("replaceCharacters must be greater or equal to the size of the from vector!");
    }
    //calculates how many characters to copy before inserting
    //Depending on the available elements in the source vector, the available space to be overwritten in the destination vector and the amount of characters that are allowed to be replaced
    size_t copyAmount = std::min(from.size(), std::min(std::distance(position, to->end()), replaceCharacters));
    //overrides values in destination vector and saves iterator to next available element
    auto insert_position = std::copy(from->begin(), from->begin() + copyAmount, position);
    //inserts elements, shifting following elements to the side
    to->insert(insert_position, from->begin() + copyAmount, from->end());
    return std::max(static_cast<size_t>(0), replaceCharacters - copyAmount);
}