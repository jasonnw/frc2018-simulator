
//#include "stdafx.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "platform.h"

const float MINMUM_TIME_RESOLUTION = (float) 0.1; //second

robot::robot()
{
	memset(&m_config, 0, sizeof(m_config));
	memset(&m_plannedAction, 0, sizeof(m_plannedAction));
	
	m_pPlatform = NULL;
	m_plannedAction.actionType = INVALID_ACTION;
}

robot::~robot()
{
}


void robot::setConfiguration(const robotConfigurationType *pConfigIn, platform *pPlatform)
{
	memcpy(&m_config, pConfigIn, sizeof(m_config));
	if (pPlatform != NULL) {
		m_pPlatform = pPlatform;
	}

	m_state.pos.sizeX = m_config.sizeX;
	m_state.pos.sizeY = m_config.sizeY;
}

void robot::setPosition(float xIn, float yIn, int objectIdIn)
{
	m_state.pos.objectId = objectIdIn;
	m_state.pos.center.x = xIn;
	m_state.pos.center.y = yIn;
}

void robot::setPlatformAndCube(platform *pPlatform, int cubeIdxIn)
{
	m_pPlatform = pPlatform;
	m_state.cubeIdx = cubeIdxIn;
	pPlatform->removeCube(cubeIdxIn);
}

void robot::dumpOneCube(void)
{
	m_state.cubeIdx = INT32_MAX;
}

bool robot::isActionNeedCube(actionTypeType actionIn)
{
	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
	case CUBE_RED_DEFENCE_SWITCH:
	case CUBE_RED_SCALE:
	case CUBE_RED_FORCE_VAULT:
	case CUBE_RED_BOOST_VAULT:
	case CUBE_RED_LIFT_VAULT:
	case CUBE_BLUE_OFFENCE_SWITCH:
	case CUBE_BLUE_DEFENCE_SWITCH:
	case CUBE_BLUE_SCALE:
	case CUBE_BLUE_FORCE_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
	case CUBE_BLUE_LIFT_VAULT:
		return true;
	case LIFT_ONE_RED_ROBOT:
	case LIFT_ONE_BLUE_ROBOT:
	default:
		return false;
	}
}

int robot::takeAction(actionTypeType actionIn, float timeIn, int indexIn)
{
	rectangleObjectType stopPosition;
	robotPathType newPath;
	robotPathType oldPath;
	int stopIndex;
	float actionDleay;
	bool oldActionHasCube;

	if ((m_plannedAction.actionType == INVALID_ACTION) ||
	    (m_plannedAction.projectedFinishTime <= timeIn)) {
		//no pending action, just add the current action as the pending action and start it now

		m_plannedAction.actionType = actionIn;
		m_plannedAction.startTime = timeIn;
		m_plannedAction.actionIndex = indexIn;

		actionDleay = getActionDelayInSec(actionIn, timeIn, &m_plannedAction.path);
		m_plannedAction.projectedFinishTime = actionDleay;
	}
	else {
		//find stop position at the current time
		memcpy(&stopPosition, &m_state.pos, sizeof(stopPosition));

		//find if the stop position robot has a cube
		stopIndex = findStopPosition(&m_state.pos.center, &m_plannedAction.path, timeIn, &stopPosition.center, &oldActionHasCube);

		if (stopIndex != -1) {
			//cut the last turn point short
			m_plannedAction.path.turnPoints[stopIndex] = stopPosition.center;
			//cut the number of path short
			m_plannedAction.path.numberOfTurns = stopIndex + 1;
		}

		actionDleay = getActionDelayInSec(actionIn, timeIn, &stopPosition, oldActionHasCube, &newPath);

		//combine two paths to merge action
		memcpy(&oldPath, &m_plannedAction.path, sizeof(oldPath));
		if (combineTwoPathes(&oldPath, &newPath, &m_plannedAction.path) != 0) {
			//path buffer overflow
			return -1;
		}

		//update execution delay with updated cube and robot position
		actionDleay = calculateDelayOnPath(&m_state.pos.center, &m_plannedAction.path);
		m_plannedAction.actionType = actionIn;
		m_plannedAction.actionIndex = indexIn;
		m_plannedAction.projectedFinishTime = actionDleay + m_plannedAction.startTime;
	}

	return 0;
}

float robot::getActionDelayInSec(actionTypeType actionIn, float currentTimeIn, robotPathType *pPathOut) const
{
	bool hasCubeFlag;

	if (m_state.cubeIdx == INT32_MAX) {
		hasCubeFlag = false;
	}
	else {
		hasCubeFlag = true;
	}
	return getActionDelayInSec(actionIn, currentTimeIn, &m_state.pos, hasCubeFlag, pPathOut);
}

float robot::getActionDelayInSec(actionTypeType actionIn, float currentTimeIn, 
	const rectangleObjectType *pStartPosIn,  bool hasCubeFlagIn, robotPathType *pPathOut) const
{
	rectangleObjectType newPos;
	float randomFactor = m_config.randomDelayFactor * ((float)rand() / (float)RAND_MAX);
	float actionDelay;
	coordinateType destination;
	bool isCubeNeeded;
	bool dumpCubeFlag;
	robotPathType path2Cube, path2Destination;
	cubeStateType *pCube;
	allianceType alliance;

	actionDelay = randomFactor;
	pPathOut->pickUpCubeIndex = -1;

	//find alliance
	alliance = ALLIANCE_BLUE;
	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
	case CUBE_RED_DEFENCE_SWITCH:
	case CUBE_RED_SCALE:
	case CUBE_RED_FORCE_VAULT:
	case CUBE_RED_BOOST_VAULT:
	case CUBE_RED_LIFT_VAULT:
	case PUSH_RED_FORCE_BUTTON:
	case PUSH_RED_BOOST_BUTTON:
	case PUSH_RED_LIFT_BUTTON:
	case RED_ACTION_NONE:
	case LIFT_ONE_RED_ROBOT:
		alliance = ALLIANCE_RED;
		break;
	case CUBE_BLUE_OFFENCE_SWITCH:
	case CUBE_BLUE_DEFENCE_SWITCH:
	case CUBE_BLUE_SCALE:
	case CUBE_BLUE_LIFT_VAULT:
	case CUBE_BLUE_FORCE_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
	case PUSH_BLUE_FORCE_BUTTON:
	case PUSH_BLUE_BOOST_BUTTON:
	case PUSH_BLUE_LIFT_BUTTON:
	case BLUE_ACTION_NONE:
	case LIFT_ONE_BLUE_ROBOT:
	default:
		alliance = ALLIANCE_BLUE;
		break;
	}

	//because the robot always move along the wall, the destination point is
	//always by the wall
	dumpCubeFlag = isActionNeedCube(actionIn);
	if (hasCubeFlagIn) {
		isCubeNeeded = false; //already has a cube
	}
	else {
		isCubeNeeded = dumpCubeFlag;
	}
	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
		destination.x = m_pPlatform->getRedSwitchX();
		if (RED_NORTH_SWITCH_FLAG) {
			destination.y = m_pPlatform->getRedSwitchNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getRedSwitchSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		break;
	case CUBE_BLUE_OFFENCE_SWITCH:
		destination.x = m_pPlatform->getBlueSwitchX();
		if (BLUE_NORTH_SWITCH_FLAG) {
			destination.y = m_pPlatform->getBlueSwitchNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getBlueSwitchSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		break;
	case CUBE_RED_DEFENCE_SWITCH:
		destination.x = m_pPlatform->getBlueSwitchX();
		if (BLUE_NORTH_SWITCH_FLAG) {
			destination.y = m_pPlatform->getBlueSwitchSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getBlueSwitchNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		break;
	case CUBE_BLUE_DEFENCE_SWITCH:
		destination.x = m_pPlatform->getRedSwitchX();
		if (RED_NORTH_SWITCH_FLAG) {
			destination.y = m_pPlatform->getRedSwitchSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getRedSwitchNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		break;
	case CUBE_RED_SCALE:
		destination.x = m_pPlatform->getScaleX();
		if (RED_NORTH_SCALE_FLAG) {
			destination.y = m_pPlatform->getScaleNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getScaleSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		break;
	case CUBE_BLUE_SCALE:
		destination.x = m_pPlatform->getScaleX();
		if (RED_NORTH_SCALE_FLAG) {
			destination.y = m_pPlatform->getScaleSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getScaleNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		break;
	case CUBE_RED_FORCE_VAULT:
	case CUBE_RED_BOOST_VAULT:
	case CUBE_RED_LIFT_VAULT:
		destination.x = m_pPlatform->getRedExchangeZoneX() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeX / 2;
		destination.y = m_pPlatform->getRedExchangeZoneY();
		break;
	case CUBE_BLUE_LIFT_VAULT:
	case CUBE_BLUE_FORCE_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
		destination.x = m_pPlatform->getBlueExchangeZoneX() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeX / 2;
		destination.y = m_pPlatform->getBlueExchangeZoneY();
		break;
	case PUSH_RED_FORCE_BUTTON:
	case PUSH_RED_BOOST_BUTTON:
	case PUSH_RED_LIFT_BUTTON:
	case PUSH_BLUE_FORCE_BUTTON:
	case PUSH_BLUE_BOOST_BUTTON:
	case PUSH_BLUE_LIFT_BUTTON:
	case RED_ACTION_NONE:
	case BLUE_ACTION_NONE:
		//no move
		destination = pStartPosIn->center;
		//no delay
		actionDelay = 0;
		break;
	case LIFT_ONE_RED_ROBOT:
		destination = m_pPlatform->getRedLiftZonePosition();
		actionDelay += m_config.liftRobotDelay;
		break;
	case LIFT_ONE_BLUE_ROBOT:
		destination = m_pPlatform->getBlueLiftZonePosition();
		actionDelay += m_config.liftRobotDelay;
		break;
	default:
		//no robot action
		actionDelay = 0;
		break;
	}

	memcpy(&newPos, pStartPosIn, sizeof(newPos));

	//pick up a cube
	if (isCubeNeeded) {
		if(!m_pPlatform->findTheClosestCube(&newPos, alliance, &pCube, &path2Cube)) {
			return CLIMB_END_TIME + 1; //task cannot be done
		}

		actionDelay += m_config.pickUpCubeDelay;
		newPos.center = path2Cube.turnPoints[path2Cube.numberOfTurns - 1];
	}

	//go to destination
	if (m_pPlatform->findAvailablePath(&newPos, destination, false, &path2Destination)) {
		return CLIMB_END_TIME + 1; //task cannot be done
	}

	//combine pick up cube path and destination path
	if (0 != combineTwoPathes(&path2Cube, &path2Destination, pPathOut)) {
		return CLIMB_END_TIME + 1; //number of turns over the limitation
	}

	actionDelay += calculateDelayOnPath(&pStartPosIn->center, pPathOut);

	if (dumpCubeFlag) {
		actionDelay += m_config.dumpCubeDelay;
	}

	//return delay must not be 0
	if (MINMUM_TIME_RESOLUTION > actionDelay) {
		actionDelay = MINMUM_TIME_RESOLUTION;
	}

	return actionDelay;
}

int robot::combineTwoPathes(const robotPathType *pPath1In, const robotPathType *pPath2In, robotPathType *pPathOut) const
{
	if (pPath1In->numberOfTurns + pPath2In->numberOfTurns > MAX_TURNS_ON_PATH) {
		printf("ERROR: combined path %d overflow the path buffer\n", pPath1In->numberOfTurns + pPath2In->numberOfTurns);
		return -1;
	}

	pPathOut->numberOfTurns = 0;
	pPathOut->pickUpCubeIndex = pPath1In->pickUpCubeIndex;

	pPathOut->totalDistance = pPath1In->totalDistance + pPath2In->totalDistance;
	for (int i = 0; i < pPath1In->numberOfTurns; i++) {
		pPathOut->pickUpCubeIndex = pPathOut->numberOfTurns;
		pPathOut->turnPoints[pPathOut->numberOfTurns] = pPath1In->turnPoints[i];
		pPathOut->numberOfTurns++;
	}

	for (int i = 0; i < pPath2In->numberOfTurns; i++) {
		pPathOut->turnPoints[pPathOut->numberOfTurns] = pPath2In->turnPoints[i];
		pPathOut->numberOfTurns++;
	}

	if (pPathOut->pickUpCubeIndex == -1) {
		pPathOut->pickUpCubeIndex = pPath2In->pickUpCubeIndex;
	}
	else {
		if (pPath2In->pickUpCubeIndex != -1) {
			printf("ERROR: combine two pick up cube paths\n");
			return -1;
		}
	}
	return 0;
}

float robot::calculateDelayOnPath(const coordinateType *pStartIn, const robotPathType *pPathIn) const
{
	float delay = 0;
	coordinateType startPos = *pStartIn;
	float distance;
	float maxmimuSpeed;

	for (int i = 0; i < pPathIn->numberOfTurns; i++) {
		distance = (pPathIn->turnPoints[i].x - startPos.x) * (pPathIn->turnPoints[i].x - startPos.x);
		distance += (pPathIn->turnPoints[i].y - startPos.y) * (pPathIn->turnPoints[i].y - startPos.y);

		distance = (float)sqrt(distance);

		if (distance >= m_config.accelerationDistance) {
			//high speed portion
			delay += (distance - m_config.accelerationDistance) / m_config.maximumSpeed;
			//acceleration portion
			delay += ((m_config.accelerationDistance * 2) * 2) / m_config.maximumSpeed;
		}
		else {
			maxmimuSpeed = (m_config.maximumSpeed * distance) / m_config.accelerationDistance;
			delay += (distance * 2) / maxmimuSpeed;
		}

		delay += m_config.turnDelay;
		startPos = pPathIn->turnPoints[i];
	}

	return delay;
}

int robot::findStopPosition(const coordinateType *pStartIn, const robotPathType *pPathIn, 
	 float stopDelayIn, coordinateType *pStopPositionOut, bool *pHasCubFlagOut) const
{
	coordinateType startPos = *pStartIn;
	int stopIndex = -1;
	float delay = 0;
	float turnPointDelay;
	float distance;
	float stopDistance;
	float timeDifference;
	float maxmimumSpeed;
	float accelaterationDuration;
	float actualMoveTime;

	*pHasCubFlagOut = false;
	for (int i = 0; i < pPathIn->numberOfTurns; i++) {
		distance = (pPathIn->turnPoints[i].x - startPos.x) * (pPathIn->turnPoints[i].x - startPos.x);
		distance += (pPathIn->turnPoints[i].y - startPos.y) * (pPathIn->turnPoints[i].y - startPos.y);

		distance = (float)sqrt(distance);
		startPos = pPathIn->turnPoints[i];

		if (distance >= m_config.accelerationDistance) {
			//high speed portion
			turnPointDelay = (distance - m_config.accelerationDistance) / m_config.maximumSpeed;
			//acceleration portion
			turnPointDelay += ((m_config.accelerationDistance * 2) * 2) / m_config.maximumSpeed;
			accelaterationDuration = m_config.accelerationDistance * 2 / m_config.maximumSpeed;
		}
		else {
			maxmimumSpeed = (m_config.maximumSpeed * distance) / m_config.accelerationDistance;
			turnPointDelay = (distance * 2) / maxmimumSpeed; //average speed is half of the maximum speed
			accelaterationDuration = turnPointDelay / 2;
		}

		delay += turnPointDelay;

		if (delay > stopDelayIn) {
			if (turnPointDelay < delay - stopDelayIn) {
				printf("ERROR, it should stop at the previous turn point\n");
			}

			//find the actual point to stop
			timeDifference = delay - stopDelayIn;

			if (timeDifference <= accelaterationDuration) {
				//stop at deceleration portion, the actual distance is timeDifference shorter to reach the next turn point
				stopDistance = distance - (m_config.maximumSpeed / 2) * timeDifference * timeDifference / accelaterationDuration;
			}
			else {
				if (timeDifference <= turnPointDelay / 2) {
					//stop at maximum speed portion
					stopDistance = distance - (timeDifference - accelaterationDuration) * m_config.maximumSpeed - m_config.accelerationDistance;
				}
				else if (timeDifference > turnPointDelay / 2) {

					if (timeDifference <= turnPointDelay - accelaterationDuration) {
						//stop at maximum speed portion
						stopDistance = distance - (timeDifference - accelaterationDuration) * m_config.maximumSpeed - m_config.accelerationDistance;
					}
					else {
						//stop at acceleration portion
						actualMoveTime = timeDifference - (turnPointDelay - accelaterationDuration);
						stopDistance = (m_config.maximumSpeed / 2) * actualMoveTime * actualMoveTime / accelaterationDuration;
					}

				}
			}

			//find the stop point by distance
			pStopPositionOut->x = startPos.x + (pPathIn->turnPoints[i].x - startPos.x) * stopDistance / distance;
			pStopPositionOut->y = startPos.y + (pPathIn->turnPoints[i].y - startPos.y) * stopDistance / distance;

			break; //stop here
		}

		delay += m_config.turnDelay;
		stopIndex = i;

		if (delay >= stopDelayIn) {
			pStopPositionOut->x = pPathIn->turnPoints[i].x;
			pStopPositionOut->y = pPathIn->turnPoints[i].y;
			break; //stop here
		}

		if (i == pPathIn->pickUpCubeIndex) {
			delay += m_config.pickUpCubeDelay;

			if (delay >= stopDelayIn) {
				pStopPositionOut->x = pPathIn->turnPoints[i].x;
				pStopPositionOut->y = pPathIn->turnPoints[i].y;
				break;
			}
			else {
				*pHasCubFlagOut = true;
			}
		}
	}

	return stopIndex;
}


