#pragma once

#include <stdint.h>

//coordinate on the field
//(0, 0) is the lower corner on the red alliance side
typedef struct coordinateType {
	float x;
	float y;
}coordinateType;

typedef struct rectangleObjectType {
	coordinateType center;
	float sizeX;
	float sizeY;
	int objectId; //a unique ID for each object on the field
}rectangleObjectType;

typedef struct coordinateType cubeStateType;

//Robot delay configurations in number of second
typedef struct robotConfigurationType {
	float randomDelayFactor;
	float boxToVaultDelay;
	float boxToSwitchDelay;
	float boxToScaleDelay;
	float pushButtonDelay;
	float liftRobotDelay;
}robotConfigurationType;

typedef enum ownerShipType {
	OWNED_BY_NONE,  //no one owns it
	OWNED_BY_RED,
	OWNED_BY_BLUE,
}ownerShipType;

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
	CUBE_RED_OFFENCE_SWITCH = 0,  //red team starts here to search the best action
	CUBE_RED_DEFENCE_SWITCH,
	CUBE_RED_SCALE,
	CUBE_RED_FORCE_VAULT,
	CUBE_RED_BOOST_VAULT,
	CUBE_RED_LIFT_VAULT,
	PUSH_RED_FORCE_BUTTON,
	PUSH_RED_BOOST_BUTTON,
	PUSH_RED_LIFT_BUTTON,
	LIFT_ONE_RED_ROBOT,
	RED_ACTION_NONE,
	CUBE_BLUE_OFFENCE_SWITCH = 100, //blue team starts here to search the best action
	CUBE_BLUE_DEFENCE_SWITCH,
	CUBE_BLUE_SCALE,
	CUBE_BLUE_FORCE_VAULT,
	CUBE_BLUE_BOOST_VAULT,
	CUBE_BLUE_LIFT_VAULT,
	PUSH_BLUE_FORCE_BUTTON,
	PUSH_BLUE_BOOST_BUTTON,
	PUSH_BLUE_LIFT_BUTTON,
	LIFT_ONE_BLUE_ROBOT,
	BLUE_ACTION_NONE
}actionTypeType;

const int NUMBER_OF_ROBOTS = 3; //the number of robots per alliance
const int INDEX_OF_ROBOT_NONE = INT32_MAX;  // invalid robot index

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
	bool redLiftFlag[NUMBER_OF_ROBOTS];
	bool blueLiftFlag[NUMBER_OF_ROBOTS];
	vaultButtonStateType redForceButton;
	vaultButtonStateType blueForceButton;
	vaultButtonStateType redBoostButton;
	vaultButtonStateType blueBoostButton;
	vaultButtonStateType redLiftButton;
	vaultButtonStateType blueLiftButton;
}platformStateType;

typedef enum robotActionType {
	PUT_CUBE_ON_SWITCH,
	PUT_CUBE_ON_SCALE,
	PUT_CUBE_ON_EXCHANGE,
	TAKE_CUBE_FROM_EXCHANGE,
	TAKE_CUBE_FROM_GROUND,
	LIFT_ONE_ROBOT,
	ACTION_NONE
}robotActionType;

const int MAX_WALL_TO_WALL_MOVES = 4;
const int MAX_TURNS_ON_PATH = MAX_WALL_TO_WALL_MOVES * 4;
//maximum turning points on the path
//the last turning point must be the target position

typedef struct robotPathType {
	coordinateType turnPoints[MAX_TURNS_ON_PATH];
	int numberOfTurns;  //the number of turns on the path.
	float totalDistance;
}robotPathType;

typedef struct robotStateType {
	rectangleObjectType currentPosition;
	robotPathType path;
	robotActionType action;
	cubeStateType *pCube; //NULL mean no cube
	int elevaterHeight;
}robotStateType;

const int MIN_BLOCK_DIFFERENCE_TO_SCORE = 2; //minimum 2 blocks to own scale or switch
const float ROBOT_TO_WALL_DISTANCE = 2;         //always 2 inches away from the wall

//Game time in seconds
const float COMPETITION_START_TIME = 0;                      //competition start time
const float AUTONOMOUS_END_TIME = 15;						 // Autonomous session end time 
const float COMPETITION_END_TIME = AUTONOMOUS_END_TIME + 135; //total competition time is 2m15sec
const float CLIMB_END_TIME = COMPETITION_END_TIME + 30;      //climb after competition time is additional 30sec
															 //Note: To simplify the simulator, auto session result will be used to initialize the game state.
															 //      Auto session time is excluded from game time. The game starts on COMPITATION_START_TIME
//action search control parameters
const int MAXIMUM_PENDING_ACTIONS = 6;                       //maximum number of look forward actions to search for the best move
const int NUM_OF_POSSIBLE_ACTIONS = RED_ACTION_NONE + 1;     //the number of possible actions
const int MIN_SCORE_CHECKING_STEP = 4;                       //start check if the action worth to continue
const int SEARCH_CONTINUE_THRESHOLD = 0;                     //the threshold score to continue action search
//Note: SEARCH_GIVEUP_THRESHOLD controls action search speed and quality. Smaller SEARCH_GIVEUP_THRESHOLD will make search slower but may find the best action.

//robot delay configurations in number of second
const robotConfigurationType RED_CONFIGURATION[NUMBER_OF_ROBOTS] =
{
	{ 0.5,  /*random delay factor*/	5.5,  /*delay to vault*/ 6.5,  /*delay to switch*/	8.0,  /*delay to scale*/ 1.0, /*push button*/ 20.0 /*lift robot*/ },
	{ 0.5,  /*random delay factor*/	5.5,  /*delay to vault*/ 6.5,  /*delay to switch*/	8.0,  /*delay to scale*/ 1.0, /*push button*/ 20.0 /*lift robot*/ },
	{ 0.5,  /*random delay factor*/	5.5,  /*delay to vault*/ 6.5,  /*delay to switch*/	8.0,  /*delay to scale*/ 1.0, /*push button*/ 20.0 /*lift robot*/ },
};
const robotConfigurationType BLUE_CONFIGURATION[NUMBER_OF_ROBOTS] =
{
	{ 6,  /*random delay factor*/	5.5,  /*delay to vault*/ 6.5,  /*delay to switch*/	8.0,  /*delay to scale*/ 1.0, /*push button*/ 20.0 /*lift robot*/ },
	{ 6,  /*random delay factor*/	5.5,  /*delay to vault*/ 6.5,  /*delay to switch*/	8.0,  /*delay to scale*/ 1.0, /*push button*/ 20.0 /*lift robot*/ },
	{ 6,  /*random delay factor*/	5.5,  /*delay to vault*/ 6.5,  /*delay to switch*/	8.0,  /*delay to scale*/ 1.0, /*push button*/ 20.0 /*lift robot*/ },
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
	{ false, false, false }, //bool liftRedRobotFlag[3];
	{ false, false, false }, //bool liftBlueRobotFlag[3];
	BUTTON_NOT_PUSH, //vaultButtonStateType redForceButton;
	BUTTON_NOT_PUSH, //vaultButtonStateType blueForceButton;
	BUTTON_NOT_PUSH, //vaultButtonStateType redBoostButton;
	BUTTON_NOT_PUSH, //vaultButtonStateType blueBoostButton;
	BUTTON_NOT_PUSH, //vaultButtonStateType redLiftButton;
	BUTTON_NOT_PUSH //vaultButtonStateType blueLiftButton;
};

