
//#include "stdafx.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "platform.h"

//#define GET_RANDOM_VALUE (m_config.randomDelayFactor * (((double) rand() - RAND_MAX/2) / (double)RAND_MAX));
#define GET_RANDOM_VALUE  0

robot::robot()
{
	memset(&m_config, 0, sizeof(m_config));
	memset(&m_plannedAction, 0, sizeof(m_plannedAction));
	
	m_pPlatform = NULL;
	m_plannedAction.actionType = INVALID_ACTION;
	m_plannedAction.projectedFinishTime = CLIMB_END_TIME;
}

robot::~robot()
{
}


void robot::setConfiguration(const robotConfigurationType *pConfigIn, platform *pPlatform, int indexIn)
{
	memcpy(&m_config, pConfigIn, sizeof(m_config));
	if (pPlatform != NULL) {
		m_pPlatform = pPlatform;
	}

	m_state.pos.sizeX = m_config.sizeX;
	m_state.pos.sizeY = m_config.sizeY;
	setRobotIndex(indexIn);
}

void robot::setPosition(double xIn, double yIn, int objectIdIn)
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
	m_state.cubeIdx = INVALID_IDX;
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
	default:
		return false;
	}
}

bool robot::isAutonomousAction(actionTypeType actionIn)
{
	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
	case CUBE_RED_DEFENCE_SWITCH:
	case CUBE_RED_SCALE:
	case CUBE_BLUE_OFFENCE_SWITCH:
	case CUBE_BLUE_DEFENCE_SWITCH:
	case CUBE_BLUE_SCALE:
		return true;
	default:
		return false;
	}
}

bool robot::isHumanPlayerAction(actionTypeType actionIn)
{
	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
	case CUBE_RED_DEFENCE_SWITCH:
	case CUBE_RED_SCALE:
	case CUBE_RED_FORCE_VAULT:
	case CUBE_RED_BOOST_VAULT:
	case CUBE_RED_LIFT_VAULT:
	case LIFT_ONE_RED_ROBOT:
	case CUBE_BLUE_OFFENCE_SWITCH:
	case CUBE_BLUE_DEFENCE_SWITCH:
	case CUBE_BLUE_SCALE:
	case CUBE_BLUE_FORCE_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
	case CUBE_BLUE_LIFT_VAULT:
	case LIFT_ONE_BLUE_ROBOT:
		return false;
	default:
		return true;
	}
}

int robot::forceAction(const pendingActionType *pPlannedActionIn, coordinateType startPosIn, double timeIn, int indexIn)
{
	double actionDelay = pPlannedActionIn->projectedFinishTime - pPlannedActionIn->startTime;
	double picupCubeDelay = pPlannedActionIn->pickUpCubeTime - pPlannedActionIn->startTime;

	//sync the current robot position
	m_state.pos.center = startPosIn;

	memcpy(&m_plannedAction, pPlannedActionIn, sizeof(pendingActionType));
	m_plannedAction.startTime = timeIn;
	m_plannedAction.projectedFinishTime = timeIn + actionDelay;
	m_plannedAction.pickUpCubeTime = timeIn + picupCubeDelay;
	m_plannedAction.actionIndex = indexIn;

	return 0;
}


int robot::takeAction(actionTypeType actionIn, double timeIn, int indexIn)
{
	double actionDleay;
	double pickUpCubeDelay;
	bool hasCubeFlag = false;
	bool interruptFlag = false;

	if (m_plannedAction.actionType != actionIn) {
		//stop the current action and start a new action
		m_plannedAction.actionType = actionIn;
		m_plannedAction.startTime = timeIn;
		m_plannedAction.actionIndex = indexIn;

		if (m_state.cubeIdx != INVALID_IDX) {
			hasCubeFlag = true;
		}

		actionDleay = getActionDelayInSecInternal(actionIn, timeIn, &m_state.pos, hasCubeFlag, interruptFlag,
			                                      &m_plannedAction.path, &pickUpCubeDelay);
		if (timeIn + actionDleay <= CLIMB_END_TIME) {
			m_plannedAction.projectedFinishTime = timeIn + actionDleay;
			m_plannedAction.pickUpCubeTime = timeIn + pickUpCubeDelay;
		}
		else {
			//give up the current action
			m_plannedAction.actionType = m_allianceType == ALLIANCE_RED ? RED_ACTION_NONE : BLUE_ACTION_NONE;
			m_plannedAction.path.pickUpCubeIndex = INVALID_IDX;
			m_plannedAction.path.numberOfTurns = 0;
			m_plannedAction.startTime = timeIn;
			m_plannedAction.actionIndex = indexIn;
			m_plannedAction.projectedFinishTime = timeIn + MINMUM_TIME_RESOLUTION;
			m_plannedAction.pickUpCubeTime = timeIn + MINMUM_TIME_RESOLUTION;
			return -1;
		}
	}
	//else, continue the current action, no change

	return 0;
}


double robot::estimateActionDelayInSec(actionTypeType actionIn, double currentTimeIn, 
	bool interruptFlagIn, coordinateType lastActionStopPosIn, bool lastActionCubeNotUsedFlagIn, coordinateType *pEndPosOut) const
{
	coordinateType stopPosition;
	rectangleObjectType startPosition;
	robotPathType plannedPath;
	int cubeIndex;
	bool hasCubeFlag = false;
	bool giveUpCubeFlag = false;
	bool middleOfLineFlag;
	bool justTimeUpFlag;
	int stopIndex;
	double delayOutput;
	double pickUpCubeDelay;
	double turnPointDelayChange;

	memcpy(&startPosition, &m_state.pos, sizeof(startPosition));

	if (interruptFlagIn) {
		if (m_plannedAction.actionType == actionIn) {
			//no interrupt, continue the current task
			if (m_plannedAction.projectedFinishTime <= currentTimeIn) {
				printf("ERROR: pending task is already done, but not committed\n");
			}
			return m_plannedAction.projectedFinishTime - currentTimeIn;
		}
		else {
			//stop the current task and start the new task
			stopIndex = findStopPosition(&startPosition.center, &m_plannedAction.path, currentTimeIn, &stopPosition,
				&cubeIndex, &turnPointDelayChange, &giveUpCubeFlag, &middleOfLineFlag, &justTimeUpFlag);

			if ((m_state.cubeIdx != INVALID_IDX) || (cubeIndex != INVALID_IDX)) {
				hasCubeFlag = true; //may have cube from the previous action or before stop point
			}

			//switch to the new action
			startPosition.center = stopPosition;
			delayOutput = getActionDelayInSecInternal(actionIn, currentTimeIn, &startPosition, hasCubeFlag, interruptFlagIn,
				          &plannedPath, &pickUpCubeDelay);
		}
	}
	else {
		//start the task after the current task is done
		if (m_plannedAction.path.numberOfTurns != 0) {
			startPosition.center = m_plannedAction.path.turnPoints[m_plannedAction.path.numberOfTurns - 1];
		}
		else {
			startPosition.center = m_state.pos.center;
		}

		hasCubeFlag = lastActionCubeNotUsedFlagIn;

		delayOutput = getActionDelayInSecInternal(actionIn, currentTimeIn, &startPosition, hasCubeFlag, interruptFlagIn,
			                                      &plannedPath, &pickUpCubeDelay);
		//only calculate the delay of the new action, old action delay will be added by the caller
	}

	if (plannedPath.numberOfTurns != 0) {
		*pEndPosOut = plannedPath.turnPoints[plannedPath.numberOfTurns - 1];
	}
	else {
		pEndPosOut->x = pEndPosOut->y = 0;
	}
	return delayOutput;
}

double robot::getActionDelayInSecInternal(actionTypeType actionIn, double currentTimeIn,
	const rectangleObjectType *pStartPosIn, bool hasCubeFlagIn, bool interruptFlagIn,
	robotPathType *pPathOut, double *pPickUpCubeDelayOut) const
{
	rectangleObjectType newPos;
	const rectangleObjectType *pCollisionObject;
	double randomFactor = GET_RANDOM_VALUE;
	double actionDelay;
	double lastTurnDelay;
	double pickUpCubeDelay;
	double offsetX, offsetY;
	coordinateType destination;
	coordinateType targetCenter;
	bool isCubeNeeded;
	bool dumpCubeFlag;
	robotPathType path2Cube, path2Destination;
	cubeStateType *pCube;
	allianceType alliance;
	static int debugCounter = 0;

	actionDelay = randomFactor;
	lastTurnDelay = randomFactor;

	pPathOut->pickUpCubeIndex = INVALID_IDX;
	pPathOut->numberOfTurns = 0;
	pPathOut->totalDistance = 0;

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
	case BLUE_ACTION_NONE:
	case LIFT_ONE_BLUE_ROBOT:
		alliance = ALLIANCE_BLUE;
		break;
	default:
		printf("ERROR: invalid action %d\n", actionIn);
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
	
	offsetX = ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeX / 2 + 24;
	offsetY = ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeY / 2;

	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
		targetCenter.x = m_pPlatform->getRedSwitchX();
		if (RED_NORTH_SWITCH_FLAG) {
			targetCenter.y = m_pPlatform->getRedSwitchNorthY();
		}
		else {
			targetCenter.y = m_pPlatform->getRedSwitchSouthY();
			offsetY = 0 - offsetY;
		}
		break;
	case CUBE_BLUE_OFFENCE_SWITCH:
		targetCenter.x = m_pPlatform->getBlueSwitchX();
		if (BLUE_NORTH_SWITCH_FLAG) {
			targetCenter.y = m_pPlatform->getBlueSwitchNorthY();
		}
		else {
			targetCenter.y = m_pPlatform->getBlueSwitchSouthY();
			offsetY = 0 - offsetY;
		}
		break;
	case CUBE_RED_DEFENCE_SWITCH:
		targetCenter.x = m_pPlatform->getBlueSwitchX();
		if (BLUE_NORTH_SWITCH_FLAG) {
			targetCenter.y = m_pPlatform->getBlueSwitchSouthY();
			offsetY = 0 - offsetY;
		}
		else {
			targetCenter.y = m_pPlatform->getBlueSwitchNorthY();
		}
		break;
	case CUBE_BLUE_DEFENCE_SWITCH:
		targetCenter.x = m_pPlatform->getRedSwitchX();
		if (RED_NORTH_SWITCH_FLAG) {
			targetCenter.y = m_pPlatform->getRedSwitchSouthY();
			offsetY = 0 - offsetY;
		}
		else {
			targetCenter.y = m_pPlatform->getRedSwitchNorthY();
		}
		break;
	case CUBE_RED_SCALE:
		targetCenter.x = m_pPlatform->getScaleX();
		if (RED_NORTH_SCALE_FLAG) {
			targetCenter.y = m_pPlatform->getScaleNorthY();
		}
		else {
			targetCenter.y = m_pPlatform->getScaleSouthY();
			offsetY = 0 - offsetY;
		}
		break;
	case CUBE_BLUE_SCALE:
		targetCenter.x = m_pPlatform->getScaleX();
		if (RED_NORTH_SCALE_FLAG) {
			targetCenter.y = m_pPlatform->getScaleSouthY();
			offsetY = 0 - offsetY;
		}
		else {
			targetCenter.y = m_pPlatform->getScaleNorthY();
		}
		break;
	case CUBE_RED_FORCE_VAULT:
	case CUBE_RED_BOOST_VAULT:
	case CUBE_RED_LIFT_VAULT:
		targetCenter.x = m_pPlatform->getRedExchangeZoneX() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeX / 2;
		targetCenter.y = m_pPlatform->getRedExchangeZoneY();
		offsetY = offsetX = 0;
		break;
	case CUBE_BLUE_LIFT_VAULT:
	case CUBE_BLUE_FORCE_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
		targetCenter.x = m_pPlatform->getBlueExchangeZoneX() + ROBOT_TO_WALL_DISTANCE + m_state.pos.sizeX / 2;
		targetCenter.y = m_pPlatform->getBlueExchangeZoneY();
		offsetY = offsetX = 0;
		break;
	case PUSH_RED_FORCE_BUTTON:
	case PUSH_RED_BOOST_BUTTON:
	case PUSH_BLUE_FORCE_BUTTON:
	case PUSH_BLUE_BOOST_BUTTON:
	case RED_ACTION_NONE:
	case BLUE_ACTION_NONE:
		//no move
		targetCenter = pStartPosIn->center;
		offsetY = offsetX = 0;
		//no delay
		actionDelay = 0;
		break;
	case LIFT_ONE_RED_ROBOT:
		targetCenter = m_pPlatform->getRedLiftZonePosition();
		targetCenter.y += (m_robotIndex - 1) * (m_state.pos.sizeY + ROBOT_TO_WALL_DISTANCE);
		offsetY = offsetX = 0;
		lastTurnDelay += m_config.liftRobotDelay;
		break;
	case LIFT_ONE_BLUE_ROBOT:
		targetCenter = m_pPlatform->getBlueLiftZonePosition();
		targetCenter.y += (m_robotIndex-1) * (m_state.pos.sizeY + ROBOT_TO_WALL_DISTANCE);
		offsetY = offsetX = 0;
		lastTurnDelay += m_config.liftRobotDelay;
		break;
	default:
		printf("ERROR: invalid action %d\n", actionIn);
		break;
	}

	memcpy(&newPos, pStartPosIn, sizeof(newPos));

	destination.x = targetCenter.x;
	destination.y = targetCenter.y + offsetY;
	newPos.center = destination;
	if (m_pPlatform->collisionWithAllOtherObjects(&newPos, destination, &pCollisionObject)) {
		if (offsetY*offsetX == 0) {
			return CLIMB_END_TIME + 1; //no other position available
		}
		//pick the closest x
		if (pStartPosIn->center.x >= targetCenter.x) {
			destination.x = targetCenter.x + offsetX;
		}
		else {
			destination.x = targetCenter.x - offsetX;
		}
		destination.y = targetCenter.y;
		newPos.center = destination;

		if (m_pPlatform->collisionWithAllOtherObjects(&newPos, destination, &pCollisionObject)) {
			//take the far side x
			if (pStartPosIn->center.x >= targetCenter.x) {
				destination.x = targetCenter.x - offsetX;
			}
			else {
				destination.x = targetCenter.x + offsetX;
			}
			destination.y = targetCenter.y;
			newPos.center = destination;

			if (m_pPlatform->collisionWithAllOtherObjects(&newPos, destination, &pCollisionObject)) {
				//no available position to finish the task
				return CLIMB_END_TIME + 1; //task cannot be done
			}
		}
	}

	//reset start point
	newPos.center = pStartPosIn->center;
	//pick up a cube
	if (isCubeNeeded) {
		if (!m_pPlatform->findTheClosestCube(&newPos, alliance, m_config.turnDelay, m_config.inOutTakeDelay, &pCube, &path2Cube)) {
			return CLIMB_END_TIME + 1; //task cannot be done
		}

		if (path2Cube.numberOfTurns != 0) {
			newPos.center = path2Cube.turnPoints[path2Cube.numberOfTurns - 1];
		}
		else {
			newPos.center = m_state.pos.center;
		}

		pickUpCubeDelay = actionDelay + calculateDelayOnPath(&pStartPosIn->center, &path2Cube);
	}
	else {
		path2Cube.numberOfTurns = 0;
		path2Cube.pickUpCubeIndex = INVALID_IDX;
		path2Cube.totalDistance = 0;
		path2Cube.turnPoints[0] = pStartPosIn->center;
		path2Cube.turnPointDelay[0] = 0;
		path2Cube.firstTurnDelay = 0;
		path2Cube.initialSpeed = 0;
		pickUpCubeDelay = 0;
	}

	//go to destination after pick up a cube
	path2Destination.pickUpCubeIndex = INVALID_IDX;
	path2Destination.numberOfTurns = 0;
	path2Destination.totalDistance = 0;
	if (!m_pPlatform->findAvailablePath(&newPos, destination, false, m_config.turnDelay, &path2Destination)) {
		return CLIMB_END_TIME + 1; //task cannot be done
	}

	if ((dumpCubeFlag) && (path2Destination.numberOfTurns != 0)) {
		path2Destination.turnPointDelay[path2Destination.numberOfTurns - 1] += m_config.inOutTakeDelay;
	}
	else if (dumpCubeFlag) {
		//dump cube at the start position
		path2Destination.firstTurnDelay += m_config.inOutTakeDelay;
	}

	path2Destination.turnPointDelay[path2Destination.numberOfTurns - 1] += lastTurnDelay;

	//combine pick up cube path and destination path
	if (0 != combineTwoPathes(&path2Cube, &path2Destination, pPathOut)) {
		return CLIMB_END_TIME + 1; //number of turns over the limitation
	}

	actionDelay += calculateDelayOnPath(&pStartPosIn->center, pPathOut);

	//return delay must not be 0
	if (MINMUM_TIME_RESOLUTION > actionDelay) {
		actionDelay = MINMUM_TIME_RESOLUTION;
	}
	*pPickUpCubeDelayOut = pickUpCubeDelay;
	debugCounter++;
	return actionDelay;
}


void robot::updatePath(int stopIdxIn, int cubeIdxIn, bool middleOfLineFlagIn, double lineDelayChangeIn, robotPathType *pPathInOut) const
{
	if (cubeIdxIn != INVALID_IDX) {
		//pick up cube is done, 
		pPathInOut->pickUpCubeIndex = INVALID_IDX;
	}

	if (stopIdxIn != INVALID_IDX) {
		if (pPathInOut->pickUpCubeIndex != INVALID_IDX) {
			if (pPathInOut->pickUpCubeIndex >= stopIdxIn + 1) {
				pPathInOut->pickUpCubeIndex -= stopIdxIn + 1;
				//Note: if stopIdx is 0, will update the robot position to turnPoint[0] and the
				//first turnPoint is turnPoint[1]
			}
			else {
				if ((pPathInOut->turnPointDelay[stopIdxIn] >= lineDelayChangeIn) &&
					(!middleOfLineFlagIn) &&
					(cubeIdxIn == INVALID_IDX)) {
					//next time, will pick up the cube at the start point
					pPathInOut->pickUpCubeIndex = -1;
				}
			}
		}
		//else, do nothing, no need to pick up a cube for this action.

		if (stopIdxIn <= pPathInOut->numberOfTurns - 1) {

			//adjust turn delay at the next turn point

			if (!middleOfLineFlagIn) {
				//[stopIdx] is the new start point
				if (pPathInOut->turnPointDelay[stopIdxIn] >= lineDelayChangeIn) {
					pPathInOut->turnPointDelay[stopIdxIn] -= lineDelayChangeIn;
					pPathInOut->firstTurnDelay = pPathInOut->turnPointDelay[stopIdxIn];
				}
				else {
					pPathInOut->firstTurnDelay = 0;
					//ignore rounding error
				}
			}
			else {
				//stop at the middle of a line, no turn delay
				pPathInOut->firstTurnDelay = 0;
			}
			for (int i = stopIdxIn + 1; i < pPathInOut->numberOfTurns; i++) {
				pPathInOut->turnPoints[i - stopIdxIn - 1] = pPathInOut->turnPoints[i];
				pPathInOut->turnPointDelay[i - stopIdxIn - 1] = pPathInOut->turnPointDelay[i];
			}
		}
		else {
			printf("ERROR, internal error, stop index is too big\n");
		}
		pPathInOut->numberOfTurns -= stopIdxIn + 1;
	}
	else {
		//stop before the first turn point, 
		if (middleOfLineFlagIn) {
			pPathInOut->firstTurnDelay = 0;
		}
		else {
			if (pPathInOut->firstTurnDelay >= lineDelayChangeIn) {
				pPathInOut->firstTurnDelay -= lineDelayChangeIn;
			}
			else {
				pPathInOut->firstTurnDelay = 0;
				//ignore rounding error
			}
		}
	}
}


int robot::combineTwoPathes(const robotPathType *pPath1In, const robotPathType *pPath2In, robotPathType *pPathOut) const
{
	if (pPath1In->numberOfTurns + pPath2In->numberOfTurns > MAX_TURNS_ON_PATH) {
		printf("ERROR: combined path %d overflow the path buffer\n", pPath1In->numberOfTurns + pPath2In->numberOfTurns);
		return -1;
	}

	pPathOut->numberOfTurns = 0;
	pPathOut->totalDistance = 0;

	if (pPath1In->numberOfTurns != 0) {
		pPathOut->firstTurnDelay = pPath1In->firstTurnDelay;
		pPathOut->pickUpCubeIndex = pPath1In->pickUpCubeIndex;

		pPathOut->totalDistance = pPath1In->totalDistance;
		pPathOut->initialSpeed = 0;
		for (int i = 0; i < pPath1In->numberOfTurns; i++) {
			pPathOut->turnPoints[i] = pPath1In->turnPoints[i];
			pPathOut->turnPointDelay[i] = pPath1In->turnPointDelay[i];
		}

		pPathOut->numberOfTurns += pPath1In->numberOfTurns;
		pPathOut->turnPointDelay[pPathOut->numberOfTurns - 1] += pPath2In->firstTurnDelay;
	}
	else {
		pPathOut->firstTurnDelay = pPath2In->firstTurnDelay;
		pPathOut->pickUpCubeIndex = INVALID_IDX;
	}

	for (int i = 0; i < pPath2In->numberOfTurns; i++) {
		pPathOut->turnPoints[i+pPathOut->numberOfTurns] = pPath2In->turnPoints[i];
		pPathOut->turnPointDelay[i+pPathOut->numberOfTurns] = pPath2In->turnPointDelay[i];
	}
	pPathOut->numberOfTurns += pPath2In->numberOfTurns;
	pPathOut->totalDistance += pPath2In->totalDistance;

	if (pPathOut->pickUpCubeIndex == INVALID_IDX) {
		pPathOut->pickUpCubeIndex = pPath2In->pickUpCubeIndex;
	}
	else {
		if (pPath2In->pickUpCubeIndex != INVALID_IDX) {
			printf("ERROR: combine two pick up cube paths\n");
			return -1;
		}
	}
	return 0;
}


double robot::getLineDelay(coordinateType startPoint, coordinateType endPoint, double maximumSpeedIn, double accelerationDistanceIn) const
{
	double distance;
	double delay;

	distance = platform::calculateDistance(endPoint, startPoint);
	delay = distance / maximumSpeedIn; //simple delay calculation without acceleration

	return delay;
}

double robot::runFromePointToPoint(coordinateType startPoint, coordinateType endPoint, 
	double initialSpeedIn, double maximumSpeedIn, double accelerationDistanceIn,
	double durationIn, coordinateType *pStopPointOut, bool *pIsFinishedFlagOut) const
{
	double finishDuration;
	double distance;
	double totalDistance;

	finishDuration = getLineDelay(startPoint, endPoint, maximumSpeedIn, accelerationDistanceIn);
	if (finishDuration <= durationIn) {
		//stop at the end point
		*pIsFinishedFlagOut = true;
		*pStopPointOut = endPoint;
		return durationIn - finishDuration;
	}
	else {
		totalDistance = platform::calculateDistance(endPoint, startPoint);

		distance = maximumSpeedIn * durationIn;
		*pIsFinishedFlagOut = false;

		if (totalDistance != 0) {
			pStopPointOut->x = startPoint.x + ((endPoint.x - startPoint.x) * distance) / totalDistance;
			pStopPointOut->y = startPoint.y + ((endPoint.y - startPoint.y) * distance) / totalDistance;
		}
		else {
			pStopPointOut->x = startPoint.x;
			pStopPointOut->y = startPoint.y;
		}
		return 0;
	}
}


double robot::calculateDelayOnPath(const coordinateType *pStartIn, const robotPathType *pPathIn) const
{
	double delay = pPathIn->firstTurnDelay;
	coordinateType startPos = *pStartIn;

	for (int i = 0; i < pPathIn->numberOfTurns; i++) {
		delay += getLineDelay(startPos, pPathIn->turnPoints[i], m_config.maximumSpeed, m_config.accelerationDistance);

		delay += pPathIn->turnPointDelay[i];
		startPos = pPathIn->turnPoints[i];
	}

	return delay;
}


actionResultType robot::moveToNextTime(double timeIn)
{
	actionResultType returnResult;
	int stopIdx;
	double delay;
	double turnPointDelayChange;
	coordinateType newPosition;
	actionTypeType actionType;
	int cubeIndex;
	bool giveUpCubeFlag = false;
	bool middleOfLineFlag = false;
	bool justTimeUpFlag;

	returnResult = ACTION_IN_PROGRESS;
	actionType = m_plannedAction.actionType; //save a copy of action type

	if (timeIn < m_plannedAction.startTime) {
		printf("ERROR: robot action start too late\n");
		return ACTION_START_ERROR; //action not started
	}

	delay = timeIn - m_plannedAction.startTime;

	stopIdx = findStopPosition(&m_state.pos.center, &m_plannedAction.path, delay, &newPosition, &cubeIndex,
		&turnPointDelayChange, &giveUpCubeFlag, &middleOfLineFlag, &justTimeUpFlag);

	if (cubeIndex != INVALID_IDX) {
		if (m_state.cubeIdx != INVALID_IDX) {
			printf("ERROR, robot already has a cube\n");
		}
		m_state.cubeIdx = cubeIndex;
		m_plannedAction.path.pickUpCubeIndex = INVALID_IDX;
	}

	//update the current position and time
	m_plannedAction.startTime = timeIn;
	m_state.pos.center = newPosition;

	if (timeIn >= m_plannedAction.projectedFinishTime) {
		//to avoid time drifting, force action done when it passed the original projected finish time
		if (isActionNeedCube(actionType)) {
			if (m_state.cubeIdx != INVALID_IDX) {
				dumpOneCube();
				returnResult = ACTION_DONE;
				m_plannedAction.actionType = INVALID_ACTION;
				m_plannedAction.path.numberOfTurns = 0;
				m_plannedAction.path.pickUpCubeIndex = INVALID_IDX;
				m_plannedAction.path.firstTurnDelay = 0;
			}
			else {
				//action failed, likely pick up cube failed
				//reschedule the task
				m_plannedAction.actionType = INVALID_ACTION;
				if (0 != takeAction(actionType, timeIn, -1)) {
					returnResult = ACTION_TIME_OUT;
					m_plannedAction.actionType = INVALID_ACTION;
					m_plannedAction.path.numberOfTurns = 0;
					m_plannedAction.path.pickUpCubeIndex = INVALID_IDX;
					m_plannedAction.path.firstTurnDelay = 0;
				}
			}
		}
		else {
			returnResult = ACTION_DONE;
			m_plannedAction.actionType = INVALID_ACTION;
			m_plannedAction.path.numberOfTurns = 0;
			m_plannedAction.path.pickUpCubeIndex = INVALID_IDX;
			m_plannedAction.path.firstTurnDelay = 0;
		}
	}
	else if (giveUpCubeFlag) {
		//action failed, likely pick up cube failed
		//reschedule the task
		m_plannedAction.actionType = INVALID_ACTION;
		if (0 != takeAction(actionType, timeIn, -1)) {
			returnResult = ACTION_TIME_OUT;
			m_plannedAction.actionType = INVALID_ACTION;
			m_plannedAction.path.numberOfTurns = 0;
			m_plannedAction.path.pickUpCubeIndex = INVALID_IDX;
			m_plannedAction.path.firstTurnDelay = 0;
		}
	}
	else {
		updatePath(stopIdx, cubeIndex, middleOfLineFlag, turnPointDelayChange, &m_plannedAction.path);
	}

	return returnResult;
}

const double TIME_ROUNDING_ERROR = (double) 1e-4;

int robot::findStopPosition(const coordinateType *pStartIn, const robotPathType *pPathIn, 
	 double stopDelayIn, coordinateType *pStopPositionOut, int *pCubeIndexOut, 
	double *pTrnPointDelayChangeOut, bool *pGiveUpCubeFlagOut, bool *atMiddleOfLineFlagOut, bool *pJustDoneFlagOut) const
{
	coordinateType startPos = *pStartIn;
	int stopIndex = INVALID_IDX;
	bool taskFinishedFlag;
	double totalDelay = 0;
	double delay;

	*pStopPositionOut = *pStartIn;
	*pCubeIndexOut = INVALID_IDX;
	*pGiveUpCubeFlagOut = false;
	*pTrnPointDelayChangeOut = 0;
	*atMiddleOfLineFlagOut = false;
	*pJustDoneFlagOut = false;

	totalDelay = pPathIn->firstTurnDelay;

	if ((totalDelay > stopDelayIn) && (totalDelay - stopDelayIn >= TIME_ROUNDING_ERROR)) {
		//stop before pick up cube
		*pTrnPointDelayChangeOut = stopDelayIn;
		return stopIndex;
	}

	if (pPathIn->pickUpCubeIndex == -1) {
		*pCubeIndexOut = m_pPlatform->pickUpCube(*pStartIn, m_allianceType);
		if (*pCubeIndexOut == INVALID_IDX) {
			//cannot pick up cube at pick up position, likely, the cube is gone
			//stop here to wait for the next command
			*pStopPositionOut = *pStartIn;
			*pGiveUpCubeFlagOut = true;
			return stopIndex;
		}
	}

	if ((totalDelay <= stopDelayIn) || 
		((totalDelay > stopDelayIn) && (totalDelay - stopDelayIn < TIME_ROUNDING_ERROR))) {
		//stop after cube pick up
		if (pPathIn->numberOfTurns == 0) {
			*pJustDoneFlagOut = true;
			return stopIndex;
		}
	}

	for (int i = 0; i < pPathIn->numberOfTurns; i++) {

		delay = getLineDelay(startPos, pPathIn->turnPoints[i], m_config.maximumSpeed, m_config.accelerationDistance);
		if (totalDelay + delay > stopDelayIn) {
			//stop at the middle of a line
			runFromePointToPoint(startPos, pPathIn->turnPoints[i], 0, m_config.maximumSpeed, m_config.accelerationDistance,
					stopDelayIn - totalDelay, pStopPositionOut, &taskFinishedFlag);
				*atMiddleOfLineFlagOut = true;
			break;
		}
		else if (totalDelay + delay == stopDelayIn) {
			//stop at the end of a line but no change on turn point delay
			*pStopPositionOut = pPathIn->turnPoints[i];
			break;
		}

		//else, 
		totalDelay += delay;
		startPos = pPathIn->turnPoints[i];
		*pStopPositionOut = pPathIn->turnPoints[i];

		delay = pPathIn->turnPointDelay[i];
		stopIndex = i;

		if (totalDelay + delay > stopDelayIn + TIME_ROUNDING_ERROR ) {
			//stop at the end of the line with less turn point delay
			*pTrnPointDelayChangeOut = stopDelayIn - totalDelay;
			break;
		}

		//else, because turn point delay is passed, the cube should be picked up already
		totalDelay += delay;
		if (totalDelay <= stopDelayIn) {
			*pStopPositionOut = pPathIn->turnPoints[i];
			if (i+1 == pPathIn->numberOfTurns) {
				*pJustDoneFlagOut = true;
			}
		}

		if (i == pPathIn->pickUpCubeIndex) {
			*pCubeIndexOut = m_pPlatform->pickUpCube(pPathIn->turnPoints[i], m_allianceType);
			if (*pCubeIndexOut == INVALID_IDX) {
				//cannot pick up cube at pick up position, likely, the cube is gone
				//stop here to wait for the next command
				*pStopPositionOut = pPathIn->turnPoints[i];
				*pGiveUpCubeFlagOut = true;
				break;
			}
		}
	}

	return stopIndex;
}
