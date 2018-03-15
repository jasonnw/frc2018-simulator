#pragma once

#include <stdint.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

//configuration of scale and switches transmitted by the Field Management System
const bool RED_NORTH_SWITCH_FLAG = true;
const bool BLUE_NORTH_SWITCH_FLAG = true;
const bool RED_NORTH_SCALE_FLAG = true;

const int INVALID_IDX = INT32_MAX;

//coordinate on the field
//(0, 0) is the lower corner on the red alliance side
typedef struct coordinateType {
	double x;
	double y;
}coordinateType;

typedef struct rectangleObjectType {
	coordinateType center;
	double sizeX;
	double sizeY;
	int objectId; //a unique ID for each object on the field
	cv::Scalar color;

public:
	rectangleObjectType()
	{
		memset(this, 0, sizeof(struct rectangleObjectType));
	}

}rectangleObjectType;

typedef struct  cubeStateType
{
	coordinateType position;
	bool availbleFlag;
	int index;

public:
	cubeStateType()
	{
		memset(this, 0, sizeof(struct cubeStateType));
	}

}cubeStateType;

//Robot delay configurations in number of second
typedef struct robotConfigurationType {
	double sizeX;
	double sizeY;
	double randomDelayFactor;
	double liftRobotDelay;
	double inOutTakeDelay;
	double turnDelay;
	double maximumSpeed;
	double accelerationDistance;	
}robotConfigurationType;

typedef enum ownerShipType {
	OWNED_BY_NONE,  //no one owns it
	OWNED_BY_RED,
	OWNED_BY_BLUE,
}ownerShipType;

typedef enum allianceType {
	INVALID_ALLIANCE,
	ALLIANCE_RED,
	ALLIANCE_BLUE
}allianceType;


typedef enum vaultButtonStateType {
	BUTTON_NOT_PUSH = 0,        // no button pushed, the initial state must be 0
	BUTTON_PUSH = 1,            // the button is just pushed
	BUTTON_PUSH_0SEC = 2,       // 0sec after button push
	BUTTON_PUSH_1SEC = 3,       // 1sec after button push
	BUTTON_PUSH_2SEC = 4,       // 2sec after button push
	BUTTON_PUSH_3SEC = 5,       // 3sec after button push
	BUTTON_PUSH_4SEC = 6,       // 4sec after button push
	BUTTON_PUSH_5SEC = 7,       // 5sec after button push
	BUTTON_PUSH_6SEC = 8,       // 6sec after button push
	BUTTON_PUSH_7SEC = 9,       // 7sec after button push
	BUTTON_PUSH_8SEC = 10,       // 8sec after button push
	BUTTON_PUSH_9SEC = 11,       // 9sec after button push
	BUTTON_PUSH_OVER_10SEC = 12  // 10 sec or over after button push
}vaultButtonStateType;

typedef enum actionTypeType {
	CUBE_RED_OFFENSE_SWITCH = 0,  //red team starts here to search the best action
	CUBE_RED_DEFENSE_SWITCH,
	CUBE_RED_SCALE,
	CUBE_RED_FORCE_VAULT,
	CUBE_RED_BOOST_VAULT,
	CUBE_RED_LIFT_VAULT,
	LIFT_ONE_RED_ROBOT,
	RED_ACTION_NONE,
	PUSH_RED_FORCE_BUTTON,
	PUSH_RED_BOOST_BUTTON,
	RED_ROBOT_GOTO_POS,
	CUBE_BLUE_OFFENSE_SWITCH = 100, //blue team starts here to search the best action
	CUBE_BLUE_DEFENSE_SWITCH,
	CUBE_BLUE_SCALE,
	CUBE_BLUE_FORCE_VAULT,
	CUBE_BLUE_BOOST_VAULT,
	CUBE_BLUE_LIFT_VAULT,
	LIFT_ONE_BLUE_ROBOT,
	BLUE_ACTION_NONE,
	PUSH_BLUE_FORCE_BUTTON,
	PUSH_BLUE_BOOST_BUTTON,
	BLUE_ROBOT_GOTO_POS,
	INVALID_ACTION
}actionTypeType;

const int NUMBER_OF_ROBOTS = 3; //the number of robots per alliance
const int INDEX_OF_ROBOT_NONE = INVALID_IDX;  // invalid robot index

//the state of platform
typedef struct platformStateType {
	int scaleRedBlockCount;
	int scaleBlueBlockCount;
	int switchRed_RedBlockCount;
	int switchRed_BlueBlockCount;
	int switchBlue_RedBlockCount;
	int switchBlue_BlueBlockCount;
	ownerShipType scaleOwner;
	ownerShipType switchRedOwner;
	ownerShipType switchBlueOwner;
	int forceRedBlockCount;
	int forceBlueBlockCount;
	int boostRedBlockCount;
	int boostBlueBlockCount;
	int liftRedBlockCount;
	int liftBlueBlockCount;
	int forceRedButtonPushBlockCount;
	int forceBlueButtonPushBlockCount;
	int boostRedButtonPushBlockCount;
	int boostBlueButtonPushBlockCount;
	int liftRedButtonPushBlockCount;
	int liftBlueButtonPushBlockCount;
	int redAutonomousLineCount;
	int blueAutonomousLineCount;
	bool autonomousRankingDoneFlag;
	bool allRedRobotsLiftFlag;
	bool allBlueRobotsLiftFlag;
	bool redCrossAutoFlag[NUMBER_OF_ROBOTS];
	bool blueCrossAutoFlag[NUMBER_OF_ROBOTS];
	bool redLiftFlag[NUMBER_OF_ROBOTS];
	bool blueLiftFlag[NUMBER_OF_ROBOTS];
	vaultButtonStateType redForceButton;
	double redForceButtonTime;
	vaultButtonStateType blueForceButton;
	double blueForceButtonTime;
	vaultButtonStateType redBoostButton;
	double redBoostButtonTime;
	vaultButtonStateType blueBoostButton;
	double blueBoostButtonTime;
	vaultButtonStateType redLiftButton;
	vaultButtonStateType blueLiftButton;
}platformStateType;

//AI configuration
const int CUBE_REWARD_SCORE = 3;
const int MAX_WALL_TO_WALL_MOVES = 4;
const int MAX_TURNS_ON_PATH = MAX_WALL_TO_WALL_MOVES * 8;
//maximum turning points on the path
//the last turning point must be the target position

typedef struct robotPathType {
	coordinateType turnPoints[MAX_TURNS_ON_PATH];
	double turnPointDelay[MAX_TURNS_ON_PATH];
	double firstTurnDelay;  //the delay before start moving
	double initialSpeed;
	double totalDistance;
	int numberOfTurns;  //the number of turns on the path.
	int pickUpCubeIndex; //the turning point to pick up a cube

public:
	robotPathType()
	{
		memset(this, 0, sizeof(struct robotPathType));
	}
}robotPathType;

const int MIN_BLOCK_DIFFERENCE_TO_SCORE = 1; //minimum 2 blocks to own scale or switch
const double ROBOT_TO_WALL_DISTANCE = 6;      //always 6 inches away from the wall

//Game time in seconds
const double MINMUM_TIME_RESOLUTION = (double) 0.1; //second

const double COMPETITION_START_TIME = 0;                      //competition start time
const double AUTONOMOUS_END_TIME = 15;						 // Autonomous session end time 
const double COMPETITION_END_TIME = AUTONOMOUS_END_TIME + 135; //total competition time is 2m15sec
const double CLIMB_END_TIME = COMPETITION_END_TIME + 30;      //climb after competition time is additional 30sec
															 //Note: To simplify the simulator, auto session result will be used to initialize the game state.
															 //      Auto session time is excluded from game time. The game starts on COMPITATION_START_TIME
//action search control parameters
const double TIME_TO_SCORE_FACTOR = 0.1;                     //punish very long delay operation 
const int MAXIMUM_PENDING_ACTIONS = 4;                       //maximum number of look forward actions to search for the best move
const int NUM_OF_POSSIBLE_ACTIONS = PUSH_RED_BOOST_BUTTON + 1;     //the number of possible actions per robot
const int MIN_SCORE_CHECKING_STEP = 4;                       //start check if the action worth to continue
const int SEARCH_CONTINUE_THRESHOLD = 0;                     //the threshold score to continue action search
//Note: SEARCH_GIVEUP_THRESHOLD controls action search speed and quality. Smaller SEARCH_GIVEUP_THRESHOLD will make search slower but may find the best action.

const double PICK_UP_CUBE_DISTANCE = 4 * 12;

const double LARGEST_ROBOT_SIZE = 4 * 12;

//robot delay configurations in number of second
const robotConfigurationType RED_CONFIGURATION[NUMBER_OF_ROBOTS] =
{
	{ 30, 30, /*size*/  0.5,  /*random delay factor*/	20.0, /*lift robot*/ 2.0, /*pick up or dump cube*/ 0.5, /*turn delay */ 17*12, /*max speed*/ 2*12 /*acceleration distance*/ },
	{ 30, 30, /*size*/  0.5,  /*random delay factor*/	20.0, /*lift robot*/ 2.0, /*pick up or dump cube*/ 0.5, /*turn delay */ 17 * 12, /*max speed*/ 2 * 12 /*acceleration distance*/ },
	{ 30, 30, /*size*/  0.5,  /*random delay factor*/	20.0, /*lift robot*/ 2.0, /*pick up or dump cube*/ 0.5, /*turn delay */ 17 * 12, /*max speed*/ 2 * 12 /*acceleration distance*/ },
};
const robotConfigurationType BLUE_CONFIGURATION[NUMBER_OF_ROBOTS] =
{
	{ 30, 30, /*size*/ 6,  /*random delay factor*/  20.0, /*lift robot*/ 2.0, /*pick up or dump cube*/ 0.5, /*turn delay */ 17 * 12, /*max speed*/ 2 * 12 /*acceleration distance*/ },
	{ 30, 30, /*size*/ 6,  /*random delay factor*/  20.0, /*lift robot*/ 2.0, /*pick up or dump cube*/ 0.5, /*turn delay */ 17 * 12, /*max speed*/ 2 * 12 /*acceleration distance*/ },
	{ 30, 30, /*size*/ 6,  /*random delay factor*/  20.0, /*lift robot*/ 2.0, /*pick up or dump cube*/ 0.5, /*turn delay */ 17 * 12, /*max speed*/ 2 * 12 /*acceleration distance*/ },
};

//initialize game setting after auto session is done
const int initRedScore = 0;
const int initBlueScore = 0;
const platformStateType initState =
{
	0, //int scaleRedBlockCount;
	0, //int scaleBlueBlockCount;
	0, //int switchRed_RedBlockCount;
	0, //int switchRed_BlueBlockCount;
	0, //int switchBlue_RedBlockCount;
	0, //int switchBlue_BlueBlockCount;
	OWNED_BY_NONE, //ownerShipType scaleOwner;
	OWNED_BY_NONE, //ownerShipType switchRedOwner;
	OWNED_BY_NONE, //ownerShipType switchBlueOwner;
	0, //int forceRedBlockCount;
	0, //int forceBlueBlockCount;
	0, //int boostRedBlockCount;
	0, //int boostBlueBlockCount;
	0, //int liftRedBlockCount;
	0, //int liftBlueBlockCount;
	0, //int forceRedButtonPushBlockCount;
	0, //int forceBlueButtonPushBlockCount;
	0, //int boostRedButtonPushBlockCount;
	0, //int boostBlueButtonPushBlockCount;
	0, //int liftRedButtonPushBlockCount;
	0, //int liftBlueButtonPushBlockCount;
	0, //int redAutonomousLineCount
	0, //int blueAutonomousLineCount
	false, //bool autonomousRankingDoneFlag
	false, //bool allRedRobotsLiftFlag
	false, //bool allBlueRobotsLiftFlag
	{ false, false, false }, //bool redCrossAutoFlag[3];
	{ false, false, false }, //bool blueCrossAutoFlag[3];
	{ false, false, false }, //bool liftRedRobotFlag[3];
	{ false, false, false }, //bool liftBlueRobotFlag[3];
	BUTTON_NOT_PUSH, //vaultButtonStateType redForceButton;
	0,
	BUTTON_NOT_PUSH, //vaultButtonStateType blueForceButton;
	0,
	BUTTON_NOT_PUSH, //vaultButtonStateType redBoostButton;
	0,
	BUTTON_NOT_PUSH, //vaultButtonStateType blueBoostButton;
	0,
	BUTTON_NOT_PUSH, //vaultButtonStateType redLiftButton;
	BUTTON_NOT_PUSH //vaultButtonStateType blueLiftButton;
};

