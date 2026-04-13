#pragma once

#include <iostream> // For std::cerr
#include <vector>   // For std::vector
#include <cfloat>   // For DBL_EPSILON, or use <limits> std::numeric_limits<double>::epsilon()
#include <cmath>    // For std::fabs

// Avoid 'using namespace std;' in header files for better practice.

class HungarianAlgorithm
{
public:
	HungarianAlgorithm();
	~HungarianAlgorithm();

    /**
     * @brief Solves the assignment problem using the Hungarian algorithm.
     * @param DistMatrix The cost matrix (nRows x nCols).
     * @param Assignment Output vector where Assignment[row] = assigned_col, or -1 if unassigned.
     * @return The total cost of the optimal assignment.
     */
	double Solve(std::vector<std::vector<double>>& DistMatrix, std::vector<int>& Assignment);

private:
    // Core implementation of the Hungarian algorithm (Munkres algorithm)
	void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
    // Helper to build the assignment vector from the star matrix
	void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
    // Helper to compute the total cost of the assignment
	void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);

    // Steps of the Munkres algorithm
	void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
	void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
	void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
};
