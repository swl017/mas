#include <mas_tracker/hungarian.h> // Header for this implementation
#include <cstdlib>   // For malloc, free, calloc
#include <cmath>     // For std::fabs
#include <vector>    // For std::vector (used in Solve's signature)
#include <iostream>  // For std::cerr, std::endl
#include <limits>    // For std::numeric_limits (alternative to DBL_MAX)

// Note: DBL_EPSILON is from <cfloat> which is included via hungarian.h

HungarianAlgorithm::HungarianAlgorithm(){}
HungarianAlgorithm::~HungarianAlgorithm(){}

double HungarianAlgorithm::Solve(std::vector<std::vector<double>>& DistMatrix, std::vector<int>& Assignment)
{
	unsigned int nRows = DistMatrix.size();
	if (nRows == 0) {
        Assignment.clear();
        return 0.0; // Handle empty matrix case
    }
	unsigned int nCols = DistMatrix[0].size();
    if (nCols == 0) {
        Assignment.assign(nRows, -1); // All rows unassigned
        return 0.0; // Handle case with rows but no columns
    }

	// Allocate memory for internal representation of the distance matrix
    // and the assignment array.
	double *distMatrixIn = new double[nRows * nCols];
	int *assignment_internal = new int[nRows]; // Use a different name to avoid confusion
	double cost = 0.0;

	// Fill in the distMatrixIn. Mind the index is "i + nRows * j" (column-major access style).
	for (unsigned int i = 0; i < nRows; i++) {
		for (unsigned int j = 0; j < nCols; j++) {
			distMatrixIn[i + nRows * j] = DistMatrix[i][j];
        }
    }

	// Call the core solving function
	assignmentoptimal(assignment_internal, &cost, distMatrixIn, nRows, nCols);

	// Prepare the output Assignment vector
	Assignment.assign(nRows, -1); // Initialize with -1
	for (unsigned int r = 0; r < nRows; r++) {
		Assignment[r] = assignment_internal[r];
    }

	delete[] distMatrixIn;
	delete[] assignment_internal;
	return cost;
}

void HungarianAlgorithm::assignmentoptimal(int *assignment, double *cost, double *distMatrixIn, int nOfRows, int nOfColumns)
{
	double *distMatrix, *distMatrixTemp, *distMatrixEnd, *columnEnd, value, minValue;
	bool *coveredColumns, *coveredRows, *starMatrix, *newStarMatrix, *primeMatrix;
	int nOfElements, minDim, row, col;

	/* initialization */
	*cost = 0;
	for (row = 0; row < nOfRows; row++)
		assignment[row] = -1;

	/* generate working copy of distance Matrix */
	/* check if all matrix elements are positive */
	nOfElements = nOfRows * nOfColumns;
	distMatrix = (double *)malloc(nOfElements * sizeof(double));
    if (!distMatrix) { std::cerr << "HungarianAlgorithm: Memory allocation failed for distMatrix." << std::endl; return; }

	distMatrixEnd = distMatrix + nOfElements;

	for (int k = 0; k < nOfElements; k++) // Use int for loop counter matching nOfElements type
	{
		value = distMatrixIn[k];
		if (value < 0)
			std::cerr << "HungarianAlgorithm: Warning - All matrix elements should be non-negative for standard assignment problem." << std::endl;
		distMatrix[k] = value;
	}

	/* memory allocation for helper matrices */
	coveredColumns = (bool *)calloc(nOfColumns, sizeof(bool));
	coveredRows = (bool *)calloc(nOfRows, sizeof(bool));
	starMatrix = (bool *)calloc(nOfElements, sizeof(bool));
	primeMatrix = (bool *)calloc(nOfElements, sizeof(bool));
	newStarMatrix = (bool *)calloc(nOfElements, sizeof(bool));

    // Check for allocation failures
    if (!coveredColumns || !coveredRows || !starMatrix || !primeMatrix || !newStarMatrix) {
        std::cerr << "HungarianAlgorithm: Memory allocation failed for helper matrices." << std::endl;
        free(distMatrix); // Free already allocated memory
        if(coveredColumns) free(coveredColumns);
        if(coveredRows) free(coveredRows);
        if(starMatrix) free(starMatrix);
        if(primeMatrix) free(primeMatrix);
        if(newStarMatrix) free(newStarMatrix);
        return;
    }


	/* preliminary steps */
	if (nOfRows <= nOfColumns)
	{
		minDim = nOfRows;
		for (row = 0; row < nOfRows; row++)
		{
			/* find the smallest element in the row */
			distMatrixTemp = distMatrix + row;
			minValue = *distMatrixTemp;
			distMatrixTemp += nOfRows; // Move to next element in the same row (column-major storage)
			while (distMatrixTemp < distMatrixEnd && (distMatrixTemp - (distMatrix + row)) % nOfRows == 0 ) // Check bounds and stay in row
            { // This loop logic for row minimum with column-major storage is tricky.
              // A simpler way for column-major: iterate by column for this row.
                minValue = distMatrix[row + 0 * nOfRows]; // first element in row
                for(int c_idx = 1; c_idx < nOfColumns; ++c_idx) {
                    if (distMatrix[row + c_idx * nOfRows] < minValue) {
                        minValue = distMatrix[row + c_idx * nOfRows];
                    }
                }
                break; // Found min for the row
            }
            // Corrected row minimum finding for column-major storage:
            if (nOfColumns > 0) {
                minValue = distMatrix[row + 0 * nOfRows];
                for (int c_idx = 1; c_idx < nOfColumns; c_idx++) {
                    if (distMatrix[row + c_idx * nOfRows] < minValue) {
                        minValue = distMatrix[row + c_idx * nOfRows];
                    }
                }
            } else { // No columns
                minValue = 0; // Or handle error
            }


			/* subtract the smallest element from each element of the row */
            for (int c_idx = 0; c_idx < nOfColumns; ++c_idx) {
                distMatrix[row + c_idx * nOfRows] -= minValue;
            }
		}

		/* Steps 1 and 2a */
		for (row = 0; row < nOfRows; row++) {
			for (col = 0; col < nOfColumns; col++) {
				if (std::fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON) {
					if (!coveredColumns[col])
					{
						starMatrix[row + nOfRows*col] = true;
						coveredColumns[col] = true;
						break; // Move to next row
					}
                }
            }
        }
	}
	else /* if(nOfRows > nOfColumns) */
	{
		minDim = nOfColumns;
		for (col = 0; col < nOfColumns; col++)
		{
			/* find the smallest element in the column */
			distMatrixTemp = distMatrix + nOfRows*col; // Start of column
			columnEnd = distMatrixTemp + nOfRows;   // End of column

			minValue = *distMatrixTemp++;
			while (distMatrixTemp < columnEnd)
			{
				value = *distMatrixTemp++;
				if (value < minValue)
					minValue = value;
			}

			/* subtract the smallest element from each element of the column */
			distMatrixTemp = distMatrix + nOfRows*col;
			while (distMatrixTemp < columnEnd)
				*distMatrixTemp++ -= minValue;
		}

		/* Steps 1 and 2a */
		for (col = 0; col < nOfColumns; col++) {
			for (row = 0; row < nOfRows; row++) {
				if (std::fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON) {
					if (!coveredRows[row])
					{
						starMatrix[row + nOfRows*col] = true;
						coveredColumns[col] = true; // Also cover column
						coveredRows[row] = true;
						break; // Move to next column's scan or next row if logic implies
					}
                }
            }
        }
		for (row = 0; row < nOfRows; row++) // Uncover rows for the next steps
			coveredRows[row] = false;
	}

	/* move to step 2b */
	step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);

	/* compute cost and remove invalid assignments */
	computeassignmentcost(assignment, cost, distMatrixIn, nOfRows); // Use original distMatrixIn for cost

	/* free allocated memory */
	free(distMatrix);
	free(coveredColumns);
	free(coveredRows);
	free(starMatrix);
	free(primeMatrix);
	free(newStarMatrix);
}

void HungarianAlgorithm::buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns)
{
	for (int row = 0; row < nOfRows; row++) {
        assignment[row] = -1; // Initialize to unassigned
		for (int col = 0; col < nOfColumns; col++) {
			if (starMatrix[row + nOfRows*col])
			{
				assignment[row] = col;
				break; // Found assignment for this row
			}
        }
    }
}

void HungarianAlgorithm::computeassignmentcost(int *assignment, double *cost, double *distMatrixOriginal, int nOfRows)
{
    *cost = 0; // Reset cost
	for (int row = 0; row < nOfRows; row++)
	{
		int col = assignment[row];
		if (col >= 0) // If row is assigned
			*cost += distMatrixOriginal[row + nOfRows*col];
	}
}

void HungarianAlgorithm::step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	bool *starMatrixTemp; // No need for columnEnd here
	int col;

	/* cover every column containing a starred zero */
	for (col = 0; col < nOfColumns; col++)
	{
        // coveredColumns[col] = false; // Should already be initialized or managed by previous steps
		starMatrixTemp = starMatrix + nOfRows*col; // Points to the start of column 'col'
		for (int row = 0; row < nOfRows; ++row) { // Iterate through rows in this column
            if (starMatrixTemp[row]) // If starMatrix[row + nOfRows*col] is true
			{
				coveredColumns[col] = true;
				break; // Column is covered, move to next column
			}
        }
	}

	/* move to step 2b */
	step2b(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

void HungarianAlgorithm::step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	int col, nOfCoveredColumns;

	/* count covered columns */
	nOfCoveredColumns = 0;
	for (col = 0; col < nOfColumns; col++)
		if (coveredColumns[col])
			nOfCoveredColumns++;

	if (nOfCoveredColumns >= minDim) // If number of covered columns is at least minDim (number of assignments needed)
	{
		/* algorithm finished */
		buildassignmentvector(assignment, starMatrix, nOfRows, nOfColumns);
	}
	else
	{
		/* move to step 3 */
		step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
	}
}

void HungarianAlgorithm::step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	bool zerosFound;
	int row, col, starCol;

	zerosFound = true;
	while (zerosFound)
	{
		zerosFound = false;
		for (col = 0; col < nOfColumns; col++) {
			if (!coveredColumns[col]) {
				for (row = 0; row < nOfRows; row++) {
					if ((!coveredRows[row]) && (std::fabs(distMatrix[row + nOfRows*col]) < DBL_EPSILON))
					{
						/* prime zero */
						primeMatrix[row + nOfRows*col] = true;

						/* find starred zero in current row */
						for (starCol = 0; starCol < nOfColumns; starCol++) {
							if (starMatrix[row + nOfRows*starCol]) {
								break; // Found starred zero in this row at starCol
                            }
                        }

						if (starCol == nOfColumns) /* no starred zero found in this row */
						{
							/* move to step 4 */
							step4(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim, row, col);
							return; // Step 4 will eventually recall step2a or finish
						}
						else // Starred zero found at starCol in the current row
						{
							coveredRows[row] = true;        // Cover this row
							coveredColumns[starCol] = false; // Uncover the column of the starred zero
							zerosFound = true; // Continue search for uncovered zeros
							break; // Break from inner loop (rows) to re-evaluate columns or restart while loop
						}
					}
                }
            }
            if(zerosFound) break; // Break from outer loop (cols) if a zero led to changes
        }
	}

	/* no more uncovered zeros found without needing path augmentation via step4 */
	/* move to step 5 to adjust matrix */
	step5(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

void HungarianAlgorithm::step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col)
{
	int n, starRow, starCol, primeRow, primeCol;
	int nOfElements = nOfRows*nOfColumns;

	/* generate temporary copy of starMatrix */
	for (n = 0; n < nOfElements; n++)
		newStarMatrix[n] = starMatrix[n];

	/* star current zero (the one passed as row, col) */
	newStarMatrix[row + nOfRows*col] = true;

	/* This zero is Z0. Path is Z0, Z1*, Z2', Z3*, ...
	   Current (row, col) is the primed zero that started the path (Z0).
	   We need to find if there's a starred zero in Z0's column.
	*/
	starCol = col; // Column of Z0
	// Find starred zero in this column (starCol)
    starRow = -1; // Initialize to not found
	for (int r_idx = 0; r_idx < nOfRows; r_idx++) {
		if (starMatrix[r_idx + nOfRows*starCol]) {
			starRow = r_idx; // Found Z1*
			break;
		}
    }


	while (starRow != -1) // Loop while we find a starred zero in the column of the last primed zero
	{
		/* unstar the starred zero (Z1*, Z3*, ...) */
		newStarMatrix[starRow + nOfRows*starCol] = false;

		/* find primed zero in current row (starRow) (Z2', Z4', ...) */
		primeRow = starRow; // Row of Z1*, Z3*, ...
        primeCol = -1; // Initialize to not found
		for (int c_idx = 0; c_idx < nOfColumns; c_idx++) {
			if (primeMatrix[primeRow + nOfRows*c_idx]) {
				primeCol = c_idx; // Found Z2', Z4', ...
				break;
			}
        }

        if (primeCol == -1) { // Should not happen if algorithm is correct and path started from valid prime
            break; // Path ends if no prime in row (error or end of path logic)
        }

		/* star the primed zero (Z2', Z4', ...) */
		newStarMatrix[primeRow + nOfRows*primeCol] = true;

		/* find starred zero in current column (primeCol) for next iteration */
		starCol = primeCol; // Column of Z2', Z4', ...
        starRow = -1; // Reset for next search
		for (int r_idx = 0; r_idx < nOfRows; r_idx++) {
			if (starMatrix[r_idx + nOfRows*starCol]) { // Check original starMatrix
				starRow = r_idx;
				break;
			}
        }
	}

	/* use temporary copy as new starMatrix */
	/* delete all primes, uncover all rows */
	for (n = 0; n < nOfElements; n++)
	{
		primeMatrix[n] = false; // Clear all primes
		starMatrix[n] = newStarMatrix[n]; // Update starMatrix
	}
	for (n = 0; n < nOfRows; n++) // Uncover all rows
		coveredRows[n] = false;
    // Columns remain covered as per step2a logic based on new starMatrix

	/* move to step 2a */
	step2a(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}

void HungarianAlgorithm::step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim)
{
	double h, value;
	int row, col;

	/* find smallest uncovered element h */
	h = std::numeric_limits<double>::max(); // Initialize with a large value
	for (row = 0; row < nOfRows; row++) {
		if (!coveredRows[row]) {
			for (col = 0; col < nOfColumns; col++) {
				if (!coveredColumns[col])
				{
					value = distMatrix[row + nOfRows*col];
					if (value < h)
						h = value;
				}
            }
        }
    }

    if (h == std::numeric_limits<double>::max()) { // No uncovered elements, or all are infinity.
        // This might indicate an issue or that no further improvement is possible with this step.
        // Or, if all uncovered are 0, h would be 0.
        // If h is still DBL_MAX, it implies all elements are covered, which step2b should have caught.
        // Or, all uncovered elements are DBL_MAX.
        // This case might need careful handling or indicates a problem with input/logic.
    }


	/* add h to each covered row */
	for (row = 0; row < nOfRows; row++) {
		if (coveredRows[row]) {
			for (col = 0; col < nOfColumns; col++) {
				distMatrix[row + nOfRows*col] += h;
            }
        }
    }

	/* subtract h from each uncovered column */
	for (col = 0; col < nOfColumns; col++) {
		if (!coveredColumns[col]) {
			for (row = 0; row < nOfRows; row++) {
				distMatrix[row + nOfRows*col] -= h;
            }
        }
    }

	/* move to step 3 to look for zeros again */
	step3(assignment, distMatrix, starMatrix, newStarMatrix, primeMatrix, coveredColumns, coveredRows, nOfRows, nOfColumns, minDim);
}
