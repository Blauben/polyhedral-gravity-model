#pragma once

#include <vector>
#include <memory>

/**
* Copies a number of vector elements into a another effectively replacing them. The remaining characters are then inserted at the end of the previously copied elements shifting the original vector's element to the right.
* @param to Destination vector to copy to and insert values into.
* @param from Source vector to copy from.
* @param position Iterator position on where to copy to.
* @param replaceCharacters Number of characters to be replaced, before insertion takes place
* @return max(0, replaceCharacters - <characters copied/inserted from source vector>) : characters left to be replaced
*/
template<typename T>
size_t replaceAndShift(std::shared_ptr<std::vector<T>> to, std::vector<T>& from, typename std::vector<T>::iterator position, size_t replaceCharacters);