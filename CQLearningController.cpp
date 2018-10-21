/**
		 (
   (     )\ )
 ( )\   (()/(   (    ) (        (        (  (
 )((_)   /(_)) ))\( /( )(   (   )\  (    )\))(
((_)_   (_))  /((_)(_)|()\  )\ |(_) )\ )((_))\
 / _ \  | |  (_))((_)_ ((_)_(_/((_)_(_/( (()(_)
| (_) | | |__/ -_) _` | '_| ' \)) | ' \)) _` |
 \__\_\ |____\___\__,_|_| |_||_||_|_||_|\__, |
										|___/

Refer to Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
for a detailed discussion on Q Learning
*/
#include "CQLearningController.h"
#include <algorithm>

using namespace std;

CQLearningController::CQLearningController (HWND hwndMain) :
	CDiscController (hwndMain),
	_grid_size_x (CParams::WindowWidth / CParams::iGridCellDim + 1),
	_grid_size_y (CParams::WindowHeight / CParams::iGridCellDim + 1) {
}
/**
 The update method should allocate a Q table for each sweeper (this can
 be allocated in one shot - use an offset to store the tables one after the other)

 You can also use a boost multiarray if you wish
*/
void CQLearningController::InitializeLearningAlgorithm (void) {
	//TODO
	for (uint x = 0; x < _grid_size_x; ++x) {
		//declare new variable to hold all the values for each row
		vector<vector<double>> state;
		/*we want to fill the whole column of states with zero*/
		for (uint y = 0; y < _grid_size_y; ++y) {
			state.push_back ({ 0, 0, 0, 0 });//push this to that y_level
		}
		/*add the state to the _Qsx_sy_a vector*/
		_Qsx_sy_a.push_back (state);
	}
}

/**
 The immediate reward function. This computes a reward upon achieving the goal state of
 collecting all the mines on the field. It may also penalize movement to encourage exploring all directions and
 of course for hitting supermines/rocks!
*/
double CQLearningController::R (uint x, uint y, uint sweeper_no) {
	//TODO: roll your own here!
	double _reward_to_be_given = -50;
	int closestObject = (m_vecSweepers[sweeper_no])->CheckForObject (m_vecObjects, CParams::dMineScale);
	if (closestObject > -1) {
		switch (m_vecObjects[closestObject]->getType ()) {
		case CCollisionObject::Mine: {
			if (!m_vecObjects[closestObject]->isDead ()) {//avoid rewarding dead object
				_reward_to_be_given = 100;
			}
			break;
		}
		case CCollisionObject::Rock: {
			_reward_to_be_given = -150;
			break;
		}
		case CCollisionObject::SuperMine: {
			_reward_to_be_given = -200;
			break;
		}
		}
	}
	return _reward_to_be_given;
}

template<typename ReturnType, typename ContainerType>
ReturnType get_maximum (const vector<ContainerType>& actions) {
	return  *std::max_element (actions.begin (), actions.end ());
}

int get_high_rewardAction (const vector<double>& actions) {
	double maximum = get_maximum<double, double> (actions);

	vector<uint> indices_of_maximum;
	for (int i = 0; i < actions.size (); ++i) {
		if (actions [i] == maximum) {
			indices_of_maximum.push_back (i);
		}
	}

	return indices_of_maximum[RandInt (0, indices_of_maximum.size () - 1)];
}
/**
The update method. Main loop body of our Q Learning implementation
See: Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
*/
bool CQLearningController::Update (void) {
	//m_vecSweepers is the array of minesweepers
	//everything you need will be m_[something] ;)
	uint cDead = std::count_if (m_vecSweepers.begin (),
		m_vecSweepers.end (),
		[](CDiscMinesweeper * s)->bool {
		return s->isDead ();
	});
	if (cDead == CParams::iNumSweepers) {
		printf ("All dead ... skipping to next iteration\n");
		m_iTicks = CParams::iNumTicks;
	}

	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw) {
		if (m_vecSweepers[sw]->isDead ()) continue;
		/**
		Q-learning algorithm according to:
		Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
		*/
		//1:::Observe the current state:
		//TODO
		auto position = m_vecSweepers[sw]->Position ();
		position /= 10;
		//2:::Select action with highest historic return:
		//TODO
		int highest_reward_action = get_high_rewardAction (_Qsx_sy_a[position.x][position.y]);
		m_vecSweepers[sw]->setRotation (ROTATION_DIRECTION(highest_reward_action));
		//now call the parents update, so all the sweepers fulfill their chosen action
	}
	CDiscController::Update (); //call the parent's class update. Do not delete this.

	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw) {
		if (m_vecSweepers[sw]->isDead ()) continue;
		//TODO:compute your indexes.. it may also be necessary to keep track of the previous state
		//3:::Observe new state:
		//TODO
		auto previousPosition = m_vecSweepers[sw]->PrevPosition ();
		previousPosition /= 10;
		auto currentPosition = m_vecSweepers[sw]->Position ();
		currentPosition /= 10;

		vector<double> prev_position_Q = _Qsx_sy_a[previousPosition.x][previousPosition.y];
		vector<double> curret_postion_Q = _Qsx_sy_a[currentPosition.x][currentPosition.y];
		auto action = m_vecSweepers[sw]->getRotation ();
		//4:::Update _Q_s_a accordingly:
		//TODO
		double delta_Q = LAMBDA * (R (currentPosition.x, currentPosition.y, sw) + (GAMMA*(get_maximum<double, double> (curret_postion_Q))) - prev_position_Q[action]);
		_Qsx_sy_a[previousPosition.x][previousPosition.y][action] += delta_Q;
	}
	return true;
}

CQLearningController::~CQLearningController (void) {
	//TODO: dealloc stuff here if you need to	
}
