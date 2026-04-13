#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <stdexcept>

// Function to recursively generate all combinations
template <typename T>
void generateCombinationsHelper(
    const std::vector<std::vector<T>>& nonEmptySets,
    std::vector<T>& currentCombination,
    std::vector<std::vector<T>>& allCombinations,
    size_t setIndex
) {
    // Base case: if we've processed all sets, add the current combination to our results
    if (setIndex == nonEmptySets.size()) {
        allCombinations.push_back(currentCombination);
        return;
    }
    
    // Recursive case: try each element from the current set
    for (const T& element : nonEmptySets[setIndex]) {
        // Add the current element to our combination
        currentCombination.push_back(element);
        
        // Recursively generate combinations for the next set
        generateCombinationsHelper(nonEmptySets, currentCombination, allCombinations, setIndex + 1);
        
        // Backtrack: remove the current element for the next iteration
        currentCombination.pop_back();
    }
}

// Wrapper function that returns all combinations, handling empty sets
template <typename T>
std::vector<std::vector<T>> generateCombinations(const std::vector<std::vector<T>>& sets) {
    std::vector<std::vector<T>> allCombinations;
    std::vector<T> currentCombination;
    
    // Filter out empty sets
    std::vector<std::vector<T>> nonEmptySets;
    for (const auto& set : sets) {
        if (!set.empty()) {
            nonEmptySets.push_back(set);
        }
    }
    
    // Check if we have at least two non-empty sets
    if (nonEmptySets.size() < 2) {
        throw std::invalid_argument("At least two non-empty sets are required to generate combinations");
    }
    
    generateCombinationsHelper(nonEmptySets, currentCombination, allCombinations, 0);
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

int main() {
    try {
        // Example 1: All non-empty sets
        std::cout << "Example 1: All non-empty sets" << std::endl;
        std::vector<std::vector<std::string>> sets1 = {
            {"a_1", "a_2", "a_3"},           // Set A with l=3
            {"b_1", "b_2"},                  // Set B with m=2
            {"c_1", "c_2", "c_3", "c_4"}     // Set C with n=4
        };
        
        std::vector<std::vector<std::string>> allCombinations1 = generateCombinations(sets1);
        
        std::cout << "All combinations:" << std::endl;
        for (const auto& combination : allCombinations1) {
            printCombination(combination);
            std::cout << std::endl;
        }
        
        std::cout << "\nTotal number of combinations: " << allCombinations1.size() << std::endl;
        
        // Example 2: With an empty set that gets skipped
        std::cout << "\n\nExample 2: With an empty set" << std::endl;
        std::vector<std::vector<std::string>> sets2 = {
            {"a_1", "a_2"},           // Set A with l=2
            {"b_1"},                  // Set B with m=1
            {}                        // Empty Set C
        };
        
        std::vector<std::vector<std::string>> allCombinations2 = generateCombinations(sets2);
        
        std::cout << "All combinations (empty set C is ignored):" << std::endl;
        for (const auto& combination : allCombinations2) {
            printCombination(combination);
            std::cout << std::endl;
        }
        
        std::cout << "\nTotal number of combinations: " << allCombinations2.size() << std::endl;
        
        // Example 3: Error case - not enough non-empty sets
        std::cout << "\n\nExample 3: Not enough non-empty sets" << std::endl;
        std::vector<std::vector<std::string>> sets3 = {
            {"a_1", "a_2"},           // Set A with l=2
            {},                       // Empty Set B
            {}                        // Empty Set C
        };
        
        try {
            std::vector<std::vector<std::string>> allCombinations3 = generateCombinations(sets3);
        } catch (const std::exception& e) {
            std::cout << "Error: " << e.what() << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}