#ifndef MULTIVIEW_CARTESIAN_PRODUCT_H
#define MULTIVIEW_CARTESIAN_PRODUCT_H

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Eigen>

// Function to recursively generate all combinations
template <typename T>
void generateCombinationsHelper(
    const std::vector<std::vector<T>>& sets,
    std::vector<T>& currentCombination,
    std::vector<std::vector<T>>& allCombinations,
    size_t setIndex
) {
    // Base case: if we've processed all sets, add the current combination to our results
    if (setIndex == sets.size()) {
        if (!currentCombination.empty()) {
            allCombinations.push_back(currentCombination);
        }
        return;
    }

    // Recursive case: try each element from the current set
    for (const T& element : sets[setIndex]) {
        // Add the current element to our combination
        currentCombination.push_back(element);

        // Recursively generate combinations for the next set
        generateCombinationsHelper(sets, currentCombination, allCombinations, setIndex + 1);

        // Backtrack: remove the current element for the next iteration
        currentCombination.pop_back();
    }
}

// Wrapper function that returns all combinations
template <typename T>
std::vector<std::vector<T>> generateCombinations(const std::vector<std::vector<T>>& sets) {
    std::vector<std::vector<T>> allCombinations;
    std::vector<T> currentCombination;

    generateCombinationsHelper(sets, currentCombination, allCombinations, 0);
    return allCombinations;
}

// Function to print a single combination
template <typename T>
void printCombination(const std::vector<T>& combination) {
    std::cout << "{ ";
    for (size_t i = 0; i < combination.size(); ++i) {
        std::cout << combination[i];
        if (i < combination.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << " }";
}

#endif  // MULTIVIEW_CARTESIAN_PRODUCT_H