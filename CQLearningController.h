#pragma once
#include "cdisccontroller.h"
#include "CParams.h"
#include "CDiscCollisionObject.h"
#include <cmath>

//Additions made to the existing code
#include <vector>

//parameters for the algorithm
typedef unsigned int uint;
class CQLearningController :
	public CDiscController
{
private:
	uint _grid_size_x;
	uint _grid_size_y;

	//parameters for the algorithm 
	const double GAMMA = 0.9; //Y
	const double LAMBDA = 0.5; //n

	//Q-table for the 
	/*each of the states needs to keep 4 numbers that represent the values of taking the four differe states:
	*Up, Down, Left, Right therfore for each (x, y) mapping we have a vector. hence we need a 3d array/vector
	*intuition from https://medium.freecodecamp.org/diving-deeper-into-reinforcement-learning-with-q-learning-c18d0db58efe
	*/

	/*the innervector is the vector holding the xs at the specific y*/
	/*since each x (y,x) consists of a vector this has to also hold a vector*/
	std::vector<std::vector<std::vector<double>>> _Qsx_sy_a;

public:
	CQLearningController(HWND hwndMain);
	virtual void InitializeLearningAlgorithm(void);
	double R(uint x, uint y, uint sweeper_no);
	virtual bool Update(void);
	virtual ~CQLearningController(void);
};

