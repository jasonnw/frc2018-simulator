
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
	resetPreviousPlannedAction();
	m_pPlatform = NULL;
}

robot::~robot()
{
}

void robot::resetPreviousPlannedAction(void)
{
	m_previousPlannedAction.actionType = RED_ACTION_NONE;
	m_previousPlannedAction.projectedFinishTime = CLIMB_END_TIME + 1;
}

void robot::setPreviousPlannedAction(const pendingActionType *pPlannedActionIn)
{
	memcpy(&m_previousPlannedAction, pPlannedActionIn, sizeof(m_previousPlannedAction));
}


void robot::setConfiguration(const robotConfigurationType *pConfigIn, platform *pPlatform)
{
	memcpy(&m_config, pConfigIn, sizeof(m_config));
	if (pPlatform != NULL) {
		m_pPlatform = pPlatform;
	}
}

float robot::getActionDelayInSec(actionTypeType actionIn, float currentTimeIn, bool firstActionAfterUpdateIn)
{
	float randomFactor = m_config.randomDelayFactor * ((float)rand() / (float)RAND_MAX);
	float actionDelay;

	if ((actionIn == m_previousPlannedAction.actionType) && (firstActionAfterUpdateIn)) {
		//the robot can continue the previous action
		if ((m_previousPlannedAction.projectedFinishTime <= currentTimeIn) ||
			(m_previousPlannedAction.projectedFinishTime == INT32_MAX)) {
			//task done already, the minimum delay is 1 sec.
			return 1;
		}
		else {
			return m_previousPlannedAction.projectedFinishTime - currentTimeIn;
		}
	}

	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
	case CUBE_BLUE_OFFENCE_SWITCH:
	case CUBE_RED_DEFENCE_SWITCH:
	case CUBE_BLUE_DEFENCE_SWITCH:
		actionDelay = 6.5 + randomFactor;
		break;
	case CUBE_RED_SCALE:
	case CUBE_BLUE_SCALE:
		actionDelay = 8.0 + randomFactor;
		break;
	case CUBE_RED_FORCE_VAULT:
	case CUBE_RED_BOOST_VAULT:
	case CUBE_RED_LIFT_VAULT:
	case CUBE_BLUE_LIFT_VAULT:
	case CUBE_BLUE_FORCE_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
		actionDelay =5.5 + randomFactor;
		break;
	case PUSH_RED_FORCE_BUTTON:
	case PUSH_RED_BOOST_BUTTON:
	case PUSH_RED_LIFT_BUTTON:
	case PUSH_BLUE_FORCE_BUTTON:
	case PUSH_BLUE_BOOST_BUTTON:
	case PUSH_BLUE_LIFT_BUTTON:
	case RED_ACTION_NONE:
	case BLUE_ACTION_NONE:
		actionDelay = 1.0 + randomFactor;
		break;
	case LIFT_ONE_BLUE_ROBOT:
	case LIFT_ONE_RED_ROBOT:
		actionDelay = m_config.liftRobotDelay + randomFactor;
		break;
	default:
		//no robot action
		actionDelay = 0;
		break;
	}

	//return delay must not be 0
	if (MINMUM_TIME_RESOLUTION > actionDelay) {
		actionDelay = MINMUM_TIME_RESOLUTION;
	}

	return actionDelay;
}

/*

float robot::getActionDelayInSec(actionTypeType actionIn, float currentTimeIn, bool firstActionAfterUpdateIn)
{
	float randomFactor = m_config.randomDelayFactor * ((float)rand() / (float)RAND_MAX);
	float actionDelay;
	coordinateType destination;
	bool isCubeNeeded;
	bool dumpCubeFlag;
	robotPathType path2Cube, path2Destination, path;
	int pickUpCubeIdx;
	cubeStateType *pCube;
	allianceType alliance;

	actionDelay = randomFactor;

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
	isCubeNeeded = false;
	dumpCubeFlag = false;
	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
		destination.x = m_pPlatform->getRedSwitchX();
		if (RED_NORTH_SWITCH_FLAG) {
			destination.y = m_pPlatform->getRedSwitchNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getRedSwitchSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}

		if (m_state.pCube != NULL) {
			isCubeNeeded = true;
		}
		dumpCubeFlag = true;
		break;
	case CUBE_BLUE_OFFENCE_SWITCH:
		destination.x = m_pPlatform->getBlueSwitchX();
		if (BLUE_NORTH_SWITCH_FLAG) {
			destination.y = m_pPlatform->getBlueSwitchNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getBlueSwitchSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		if (m_state.pCube != NULL) {
			isCubeNeeded = true;
		}
		dumpCubeFlag = true;
		break;
	case CUBE_RED_DEFENCE_SWITCH:
		destination.x = m_pPlatform->getBlueSwitchX();
		if (BLUE_NORTH_SWITCH_FLAG) {
			destination.y = m_pPlatform->getBlueSwitchSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getBlueSwitchNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		if (m_state.pCube != NULL) {
			isCubeNeeded = true;
		}
		dumpCubeFlag = true;
		break;
	case CUBE_BLUE_DEFENCE_SWITCH:
		destination.x = m_pPlatform->getRedSwitchX();
		if (RED_NORTH_SWITCH_FLAG) {
			destination.y = m_pPlatform->getRedSwitchSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getRedSwitchNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		if (m_state.pCube != NULL) {
			isCubeNeeded = true;
		}
		dumpCubeFlag = true;
		break;
	case CUBE_RED_SCALE:
		destination.x = m_pPlatform->getScaleX();
		if (RED_NORTH_SCALE_FLAG) {
			destination.y = m_pPlatform->getScaleNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getScaleSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		if (m_state.pCube != NULL) {
			isCubeNeeded = true;
		}
		dumpCubeFlag = true;
		break;
	case CUBE_BLUE_SCALE:
		destination.x = m_pPlatform->getScaleX();
		if (RED_NORTH_SCALE_FLAG) {
			destination.y = m_pPlatform->getScaleSouthY() - ROBOT_TO_WALL_DISTANCE - m_state.pos.sizeY / 2;
		}
		else {
			destination.y = m_pPlatform->getScaleNorthY() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;
		}
		if (m_state.pCube != NULL) {
			isCubeNeeded = true;
		}
		dumpCubeFlag = true;
		break;
	case CUBE_RED_FORCE_VAULT:
	case CUBE_RED_BOOST_VAULT:
	case CUBE_RED_LIFT_VAULT:
		destination.x = m_pPlatform->getRedExchangeZoneX() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeX / 2;
		destination.y = m_pPlatform->getRedExchangeZoneY();
		if (m_state.pCube != NULL) {
			isCubeNeeded = true;
		}
		dumpCubeFlag = true;
		break;
	case CUBE_BLUE_LIFT_VAULT:
	case CUBE_BLUE_FORCE_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
		destination.x = m_pPlatform->getBlueExchangeZoneX() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeX / 2;
		destination.y = m_pPlatform->getBlueExchangeZoneY();
		if (m_state.pCube != NULL) {
			isCubeNeeded = true;
		}
		dumpCubeFlag = true;
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
		destination = m_state.pos.center;
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

	//pick up a cube
	if (isCubeNeeded) {
		if(m_pPlatform->findTheClosestCube(&m_state.pos, alliance, &pCube, &path2Cube)) {
			return CLIMB_END_TIME + 1; //task cannot be done
		}

		actionDelay += m_config.pickUpCubeDelay;
	}

	//go to destination
	if (m_pPlatform->findAvailablePath(&m_state.pos, destination, false, &path2Destination)) {
		return CLIMB_END_TIME + 1; //task cannot be done
	}

	//combine pick up cube path and destination path
	if (0 != combineTwoPathes(&path2Cube, &path2Destination, &path, &pickUpCubeIdx)) {
		return CLIMB_END_TIME + 1; //number of turns over the limitation
	}

	actionDelay += calculateDelayOnPath(&m_state.pos.center, &path);

	if (dumpCubeFlag) {
		actionDelay += m_config.dumpCubeDelay;
	}

	//return delay must not be 0
	if (MINMUM_TIME_RESOLUTION > actionDelay) {
		actionDelay = MINMUM_TIME_RESOLUTION;
	}

	return actionDelay;
}
*/

int robot::combineTwoPathes(const robotPathType *pPath2CubeIn, const robotPathType *pPath2DestinationIn, robotPathType *pPathOut, int *pPickUpIdxOut)
{
	if (pPath2CubeIn->numberOfTurns + pPath2DestinationIn->numberOfTurns > MAX_TURNS_ON_PATH) {
		printf("ERROR: combined path %d overflow the path buffer\n", pPath2CubeIn->numberOfTurns + pPath2DestinationIn->numberOfTurns);
		return -1;
	}

	pPathOut->numberOfTurns = 0;
	*pPickUpIdxOut = -1; //no pick up cube

	pPathOut->totalDistance = pPath2CubeIn->totalDistance + pPath2DestinationIn->totalDistance;
	for (int i = 0; i < pPath2CubeIn->numberOfTurns; i++) {
		*pPickUpIdxOut = pPathOut->numberOfTurns;
		pPathOut->turnPoints[pPathOut->numberOfTurns] = pPath2CubeIn->turnPoints[i];
		pPathOut->numberOfTurns++;
	}

	for (int i = 0; i < pPath2DestinationIn->numberOfTurns; i++) {
		pPathOut->turnPoints[pPathOut->numberOfTurns] = pPath2DestinationIn->turnPoints[i];
		pPathOut->numberOfTurns++;
	}

	return 0;
}

float robot::calculateDelayOnPath(const coordinateType *pStartIn, const robotPathType *pPathIn)
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

coordinateType robot::findStopPosition(const coordinateType *pStartIn, const robotPathType *pPathIn, int pickUpIndexIn, float stopDelayIn, bool *pHasCubFlagOut)
{
	coordinateType stopPos = *pStartIn;
	float delay = 0;
	coordinateType startPos = *pStartIn;
	float distance;
	float maxmimuSpeed;

	*pHasCubFlagOut = false;

	for (int i = 0; i < pPathIn->numberOfTurns; i++) {
		stopPos = startPos;

		distance = (pPathIn->turnPoints[i].x - startPos.x) * (pPathIn->turnPoints[i].x - startPos.x);
		distance += (pPathIn->turnPoints[i].y - startPos.y) * (pPathIn->turnPoints[i].y - startPos.y);

		distance = (float)sqrt(distance);
		startPos = pPathIn->turnPoints[i];

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

		if (delay >= stopDelayIn) {
			break; //stop here
		}

		if (i == pickUpIndexIn) {
			delay += m_config.pickUpCubeDelay;

			if (delay >= stopDelayIn) {
				break;
			}
			else {
				*pHasCubFlagOut = true;
			}
		}
	}

	return stopPos;
}


