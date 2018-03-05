
//#include "stdafx.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"
#include "platform.h"

//#define GET_RANDOM_VALUE (m_config.randomDelayFactor * (((float) rand() - RAND_MAX/2) / (float)RAND_MAX));
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

int robot::forceAction(const pendingActionType *pPlannedActionIn, float timeIn, int indexIn)
{
	float actionDleay = pPlannedActionIn->projectedFinishTime - pPlannedActionIn->startTime;

	if (m_plannedAction.actionType != pPlannedActionIn->actionType) {
		memcpy(&m_plannedAction, pPlannedActionIn, sizeof(pendingActionType));
		m_plannedAction.startTime = timeIn;
		m_plannedAction.projectedFinishTime = timeIn + actionDleay;
		m_plannedAction.actionIndex = indexIn;
	}
	//else, continue the current action

	return 0;
}


int robot::takeAction(actionTypeType actionIn, float timeIn, int indexIn)
{
	float actionDleay;
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

		actionDleay = getActionDelayInSecInternal(actionIn, timeIn, &m_state.pos, hasCubeFlag, interruptFlag, &m_plannedAction.path);
		if (timeIn + actionDleay <= CLIMB_END_TIME) {
			m_plannedAction.projectedFinishTime = timeIn + actionDleay;
		}
		else {
			//give up the current action
			m_plannedAction.actionType = m_allianceType == ALLIANCE_RED ? RED_ACTION_NONE : BLUE_ACTION_NONE;
			m_plannedAction.startTime = timeIn;
			m_plannedAction.actionIndex = indexIn;
			m_plannedAction.projectedFinishTime = timeIn + MINMUM_TIME_RESOLUTION;
			return -1;
		}
	}
	//else, continue the current action, no change

	return 0;
}


float robot::estimateActionDelayInSec(actionTypeType actionIn, float currentTimeIn, 
	bool interruptFlagIn, coordinateType lastActionStopPosIn, bool lastActionCubeNotUsedFlagIn, coordinateType *pEndPosOut) const
{
	coordinateType stopPosition;
	rectangleObjectType startPosition;
	robotPathType plannedPath;
	int cubeIndex;
	bool hasCubeFlag = false;
	bool giveUpCubeFlag = false;
	bool middleOfLineFlag;
	int stopIndex;
	float delayOutput;
	float turnPointDelayChange;

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
				&cubeIndex, &turnPointDelayChange, &giveUpCubeFlag, &middleOfLineFlag);

			if ((m_state.cubeIdx != INVALID_IDX) || (cubeIndex != INVALID_IDX)) {
				hasCubeFlag = true; //may have cube from the previous action or before stop point
			}

			//switch to the new action
			startPosition.center = stopPosition;
			delayOutput = getActionDelayInSecInternal(actionIn, currentTimeIn, &startPosition, hasCubeFlag, interruptFlagIn, &plannedPath);
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

		delayOutput = getActionDelayInSecInternal(actionIn, currentTimeIn, &startPosition, hasCubeFlag, interruptFlagIn, &plannedPath);
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

float robot::getActionDelayInSecInternal(actionTypeType actionIn, float currentTimeIn, 
	const rectangleObjectType *pStartPosIn,  bool hasCubeFlagIn, bool interruptFlagIn, robotPathType *pPathOut) const
{
	rectangleObjectType newPos;
	float randomFactor = GET_RANDOM_VALUE;
	float actionDelay;
	coordinateType destination;
	bool isCubeNeeded;
	bool dumpCubeFlag;
	robotPathType path2Cube, path2Destination;
	cubeStateType *pCube;
	allianceType alliance;

	actionDelay = randomFactor;
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
	case PUSH_BLUE_FORCE_BUTTON:
	case PUSH_BLUE_BOOST_BUTTON:
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
		printf("ERROR: invalid action %d\n", actionIn);
		break;
	}

	memcpy(&newPos, pStartPosIn, sizeof(newPos));

	//pick up a cube
	if (isCubeNeeded) {
		if(!m_pPlatform->findTheClosestCube(&newPos, alliance, m_config.turnDelay, m_config.inOutTakeDelay, &pCube, &path2Cube)) {
			return CLIMB_END_TIME + 1; //task cannot be done
		}

		if (path2Cube.numberOfTurns != 0) {
			newPos.center = path2Cube.turnPoints[path2Cube.numberOfTurns - 1];
		}
		else {
			newPos.center = m_state.pos.center;
		}
	}
	else {
		path2Cube.numberOfTurns = 0;
		path2Cube.pickUpCubeIndex = INVALID_IDX;
		path2Cube.totalDistance = 0;
		path2Cube.turnPoints[0] = pStartPosIn->center;
		path2Cube.turnPointDelay[0] = 0;
		path2Cube.firstTurnDelay = 0;
		path2Cube.initialSpeed = 0;
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
	else {
		path2Destination.firstTurnDelay += m_config.inOutTakeDelay;
	}

	//combine pick up cube path and destination path
	if (0 != combineTwoPathes(&path2Cube, &path2Destination, pPathOut)) {
		return CLIMB_END_TIME + 1; //number of turns over the limitation
	}

	actionDelay += calculateDelayOnPath(&pStartPosIn->center, pPathOut);

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
	pPathOut->firstTurnDelay = pPath1In->firstTurnDelay;
	pPathOut->pickUpCubeIndex = pPath1In->pickUpCubeIndex;

	pPathOut->totalDistance = pPath1In->totalDistance + pPath2In->totalDistance;
	pPathOut->initialSpeed = 0;
	for (int i = 0; i < pPath1In->numberOfTurns; i++) {
		pPathOut->pickUpCubeIndex = pPathOut->numberOfTurns;
		pPathOut->turnPoints[pPathOut->numberOfTurns] = pPath1In->turnPoints[i];
		pPathOut->turnPointDelay[pPathOut->numberOfTurns] = pPath1In->turnPointDelay[i];
		pPathOut->numberOfTurns++;
	}

	pPathOut->turnPointDelay[pPathOut->numberOfTurns - 1] += pPath2In->firstTurnDelay;

	for (int i = 0; i < pPath2In->numberOfTurns; i++) {
		pPathOut->turnPoints[i+pPathOut->numberOfTurns] = pPath2In->turnPoints[i];
		pPathOut->turnPointDelay[i+pPathOut->numberOfTurns] = pPath2In->turnPointDelay[i];
		pPathOut->numberOfTurns++;
	}

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


float robot::getLineDelay(coordinateType startPoint, coordinateType endPoint, float maximumSpeedIn, float accelerationDistanceIn) const
{
	float distance;
	float delay;

	distance = (endPoint.x - startPoint.x) * (endPoint.x - startPoint.x);
	distance += (endPoint.y - startPoint.y) * (endPoint.y - startPoint.y);

	distance = (float)sqrt(distance);
	delay = distance / maximumSpeedIn; //simple delay calculation without acceleration

	return delay;
}

float robot::runFromePointToPoint(coordinateType startPoint, coordinateType endPoint, 
	float initialSpeedIn, float maximumSpeedIn, float accelerationDistanceIn,
	float durationIn, coordinateType *pStopPointOut, bool *pIsFinishedFlagOut) const
{
	float finishDuration;
	float distance;
	float totalDistance;

	finishDuration = getLineDelay(startPoint, endPoint, maximumSpeedIn, accelerationDistanceIn);
	if (finishDuration <= durationIn) {
		//stop at the end point
		*pIsFinishedFlagOut = true;
		*pStopPointOut = endPoint;
		return durationIn - finishDuration;
	}
	else {
		totalDistance = (endPoint.x - startPoint.x) * (endPoint.x - startPoint.x);
		totalDistance += (endPoint.y - startPoint.y) * (endPoint.y - startPoint.y);
		totalDistance = (float) sqrt(totalDistance);

		distance = maximumSpeedIn * durationIn;
		*pIsFinishedFlagOut = false;

		pStopPointOut->x = startPoint.x + ((endPoint.x - startPoint.x) * distance) / totalDistance;
		pStopPointOut->y = startPoint.y + ((endPoint.y - startPoint.y) * distance) / totalDistance;
		return 0;
	}
}


float robot::calculateDelayOnPath(const coordinateType *pStartIn, const robotPathType *pPathIn) const
{
	float delay = pPathIn->firstTurnDelay;
	coordinateType startPos = *pStartIn;

	for (int i = 0; i < pPathIn->numberOfTurns; i++) {
		delay += getLineDelay(startPos, pPathIn->turnPoints[i], m_config.maximumSpeed, m_config.accelerationDistance);

		delay += pPathIn->turnPointDelay[i];
		startPos = pPathIn->turnPoints[i];
	}

	return delay;
}


actionResultType robot::moveToNextTime(float timeIn)
{
	actionResultType returnResult;
	int stopIdx;
	float delay;
	float turnPointDelayChange;
	coordinateType newPosition;
	actionTypeType actionType;
	int cubeIndex;
	bool giveUpCubeFlag = false;
	bool middleOfLineFlag = false;

	returnResult = ACTION_IN_PROGRESS;
	actionType = m_plannedAction.actionType; //save a copy of action type

	if (timeIn < m_plannedAction.startTime) {
		printf("ERROR: robot action start too late\n");
		return ACTION_START_ERROR; //action not started
	}

	delay = timeIn - m_plannedAction.startTime;

	stopIdx = findStopPosition(&m_state.pos.center, &m_plannedAction.path, delay, &newPosition, &cubeIndex,
		&turnPointDelayChange, &giveUpCubeFlag, &middleOfLineFlag);

	if (cubeIndex != INVALID_IDX) {
		m_state.cubeIdx = cubeIndex;
		m_plannedAction.path.pickUpCubeIndex = INVALID_IDX;
	}

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

		if (stopIdx != INVALID_IDX) {
			if (m_plannedAction.path.pickUpCubeIndex != INVALID_IDX) {
				if (m_plannedAction.path.pickUpCubeIndex >= stopIdx+1) {
					m_plannedAction.path.pickUpCubeIndex -= stopIdx + 1;
					//Note: if stopIdx is 0, will update the robot position to turnPoint[0] and the
					//first turnPoint is turnPoint[1]
				}
				else {
					if (cubeIndex != INVALID_IDX) {
						//pick up cube is done, 
						m_plannedAction.path.pickUpCubeIndex = INVALID_IDX;
					}
					else {
						if ((m_plannedAction.path.turnPointDelay[stopIdx] >= turnPointDelayChange) && (!middleOfLineFlag)) {
							//next time, will pick up the cube at the start point
							m_plannedAction.path.pickUpCubeIndex = -1;
						}
						else {
							printf("ERROR: internal logic error\n");
						}
					}
				}
			}
			//else, do nothing, no need to pick up a cube for this action.

			if (stopIdx <= m_plannedAction.path.numberOfTurns - 1) {

				//adjust turn delay at the next turn point

				if (!middleOfLineFlag) {
					//[stopIdx] is the new start point
					if (m_plannedAction.path.turnPointDelay[stopIdx] >= turnPointDelayChange) {
						m_plannedAction.path.turnPointDelay[stopIdx] -= turnPointDelayChange;
						m_plannedAction.path.firstTurnDelay = m_plannedAction.path.turnPointDelay[stopIdx];
					}
					else {
						printf("ERROR, robot stay at turn point longer than expected\n");
					}
				}
				else {
					//stop at the middle of a line, no turn delay
					m_plannedAction.path.firstTurnDelay = 0;
				}
				for (int i = stopIdx+1; i < m_plannedAction.path.numberOfTurns; i++) {
					m_plannedAction.path.turnPoints[i - stopIdx-1] = m_plannedAction.path.turnPoints[i];
					m_plannedAction.path.turnPointDelay[i - stopIdx-1] = m_plannedAction.path.turnPointDelay[i];
				}
			}
			else {
				printf("ERROR, internal error, stop index is too big\n");
			}
			m_plannedAction.startTime = timeIn;
			m_plannedAction.path.numberOfTurns -= stopIdx + 1;
		}
		else {
			//stop before the first turn point, 
			if (m_plannedAction.path.firstTurnDelay >= turnPointDelayChange) {
				m_plannedAction.path.firstTurnDelay -= turnPointDelayChange;
			}
			else {
				printf("ERROR, robot stay at star point longer than expected\n");
			}
		}
	}

	return returnResult;
}


int robot::findStopPosition(const coordinateType *pStartIn, const robotPathType *pPathIn, 
	 float stopDelayIn, coordinateType *pStopPositionOut, int *pCubeIndexOut, 
	float *tpTrnPointDelayChangeOut, bool *pGiveUpCubeFlagOut, bool *atMiddleOfLineFlagOut) const
{
	coordinateType startPos = *pStartIn;
	int stopIndex = INVALID_IDX;
	bool taskFinishedFlag;
	float totalDelay = 0;
	float delay;

	*pStopPositionOut = m_state.pos.center;
	*pCubeIndexOut = INVALID_IDX;
	*pGiveUpCubeFlagOut = false;
	*tpTrnPointDelayChangeOut = 0;
	*atMiddleOfLineFlagOut = false;

	totalDelay = pPathIn->firstTurnDelay;

	if (totalDelay >= stopDelayIn) {
		*tpTrnPointDelayChangeOut = stopDelayIn;
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

	for (int i = 0; i < pPathIn->numberOfTurns; i++) {

		delay = getLineDelay(startPos, pPathIn->turnPoints[i], m_config.maximumSpeed, m_config.accelerationDistance);
		if (totalDelay + delay > stopDelayIn) {
			//stop on the middle of a line
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

		delay = pPathIn->turnPointDelay[i];
		stopIndex = i;

		if (totalDelay + delay > stopDelayIn) {
			//stop at the end of the line with less turn point delay
			*pStopPositionOut = pPathIn->turnPoints[i];
			*tpTrnPointDelayChangeOut = stopDelayIn - totalDelay;
			break;
		}

		//else, because turn point delay is passed, the cube should be picked up already
		totalDelay += delay;
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
