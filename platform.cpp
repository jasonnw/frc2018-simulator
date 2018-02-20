
//#include "stdafx.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "platform.h"

#define ROUNDING_METHOD(x) ((int)floor((x) + 0.5))

platform::platform()
{
	memset(&m_state, 0, sizeof(m_state));
	m_timeInSec = 0;
	m_redScore = 0;
	m_blueScore = 0;
	m_pLogFIle = NULL;
	m_liftRedRobotIndex = INVALID_IDX;
	m_liftBlueRobotIndex = INVALID_IDX;

	//give each robot access with the platform
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_redRobots[i].setPlatformAndCube(this, i+ CUBE_ON_RED_ROBOTS);
		m_blueRobots[i].setPlatformAndCube(this, i + CUBE_ON_BLUE_ROBOTS);
	}

	//field layout, in unit of inch,
	//red alliance is on the left side, blue is on the right side
	//the whole field is (288*2 + 72) x (264 + 48*2)
	m_platformStructure.blueAutoLine = (288 * 2 + 72) - 10 * 12;

	m_platformStructure.blueExchangeZone.objectId = 1;
	m_platformStructure.blueExchangeZone.center.x = (288 * 2 + 72) - 36;
	m_platformStructure.blueExchangeZone.center.y = (264 + 48 * 2) / 2 - 12 - 24;
	m_platformStructure.blueExchangeZone.sizeX = 36;
	m_platformStructure.blueExchangeZone.sizeY = 48;

	m_platformStructure.blueLiftZone.objectId = 2;
	m_platformStructure.blueLiftZone.center.x = (float) ((288 * 2 + 72) - 261.74);
	m_platformStructure.blueLiftZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.blueLiftZone.sizeX = (float) ((288 * 2 + 72)/2 - 261.74);
	m_platformStructure.blueLiftZone.sizeY = 9 * 12;

	m_platformStructure.bluePlatformZone.objectId = 3;
	m_platformStructure.bluePlatformZone.center.x = (288 * 2 + 72) - 196;
	m_platformStructure.bluePlatformZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.bluePlatformZone.sizeX = (288 * 2 + 72) / 2 - 196;
	m_platformStructure.bluePlatformZone.sizeY = 9 * 12;

	m_platformStructure.bluePowerCubeZone.objectId = 4;
	m_platformStructure.bluePowerCubeZone.center.x = (288 * 2 + 72) - 98 - 42 / 2;
	m_platformStructure.bluePowerCubeZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.bluePowerCubeZone.sizeX = 42;
	m_platformStructure.bluePowerCubeZone.sizeY = 45;

	m_platformStructure.blueSwitchNorthPlate.objectId = 5;
	m_platformStructure.blueSwitchNorthPlate.center.x = (288 * 2 + 72) - 140 - 56 / 2;
	m_platformStructure.blueSwitchNorthPlate.center.y = (264 + 48 * 2) / 2 + 6 * 12;
	m_platformStructure.blueSwitchNorthPlate.sizeX = 4 * 12;
	m_platformStructure.blueSwitchNorthPlate.sizeY = 3 * 12;

	m_platformStructure.blueSwitchSouthPlate.objectId = 6;
	m_platformStructure.blueSwitchSouthPlate.center.x = (288 * 2 + 72) - 140 - 56 / 2;
	m_platformStructure.blueSwitchSouthPlate.center.y = (264 + 48 * 2) / 2 - 6 * 12;
	m_platformStructure.blueSwitchSouthPlate.sizeX = 4 * 12;
	m_platformStructure.blueSwitchSouthPlate.sizeY = 3 * 12;

	m_platformStructure.eastWall = (288 * 2 + 72);
	m_platformStructure.northWall = (264 + 48 * 2);
	m_platformStructure.redAutoLine = 10 * 12;
	
	m_platformStructure.redExchangeZone.objectId = 7;
	m_platformStructure.redExchangeZone.center.x = 36;
	m_platformStructure.redExchangeZone.center.y = (264 + 48 * 2) / 2 + 12 + 24;
	m_platformStructure.redExchangeZone.sizeX = 36;
	m_platformStructure.redExchangeZone.sizeY = 48;

	m_platformStructure.redLiftZone.objectId = 8;
	m_platformStructure.redLiftZone.center.x = (float) 261.74;
	m_platformStructure.redLiftZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.redLiftZone.sizeX = (float)((288 * 2 + 72) / 2 - 261.74);
	m_platformStructure.redLiftZone.sizeY = 9 * 12;

	m_platformStructure.redPlatformZone.objectId = 9;
	m_platformStructure.redPlatformZone.center.x = 196;
	m_platformStructure.redPlatformZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.redPlatformZone.sizeX = (288 * 2 + 72) / 2 - 196;
	m_platformStructure.redPlatformZone.sizeY = 9 * 12;

	m_platformStructure.redPowerCubeZone.objectId = 10;
	m_platformStructure.redPowerCubeZone.center.x = 98 + 42 / 2;
	m_platformStructure.redPowerCubeZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.redPowerCubeZone.sizeX = 42;
	m_platformStructure.redPowerCubeZone.sizeY = 45;

	m_platformStructure.redSwitchNorthPlate.objectId = 11;
	m_platformStructure.redSwitchNorthPlate.center.x = 140 + 56 / 2;
	m_platformStructure.redSwitchNorthPlate.center.y = (264 + 48 * 2) / 2 + 6 * 12;
	m_platformStructure.redSwitchNorthPlate.sizeX = 4 * 12;
	m_platformStructure.redSwitchNorthPlate.sizeY = 3 * 12;

	m_platformStructure.redSwitchSouthPlate.objectId = 12;
	m_platformStructure.redSwitchSouthPlate.center.x = 140 + 56 / 2;
	m_platformStructure.redSwitchSouthPlate.center.y = (264 + 48 * 2) / 2 - 6 * 12;
	m_platformStructure.redSwitchSouthPlate.sizeX = 4 * 12;
	m_platformStructure.redSwitchSouthPlate.sizeY = 3 * 12;

	m_platformStructure.scaleNorthPlate.objectId = 13;
	m_platformStructure.scaleNorthPlate.center.x = (288 * 2 + 72) / 2;
	m_platformStructure.scaleNorthPlate.center.y = (264 + 48 * 2) / 2 + (15 * 12 / 2);
	m_platformStructure.scaleNorthPlate.sizeX = 4 * 12;
	m_platformStructure.scaleNorthPlate.sizeY = 3 * 12;

	m_platformStructure.scaleSouthPlate.objectId = 14;
	m_platformStructure.scaleSouthPlate.center.x = (288 * 2 + 72) / 2;
	m_platformStructure.scaleSouthPlate.center.y = (264 + 48 * 2) / 2 - (15 * 12 / 2);
	m_platformStructure.scaleSouthPlate.sizeX = 4 * 12;
	m_platformStructure.scaleSouthPlate.sizeY = 3 * 12;

	m_platformStructure.southWall = 0;
	m_platformStructure.westWall = 0;

	m_platformStructure.structures[RED_SWITCH_ZONE].objectId = 15;
	m_platformStructure.structures[RED_SWITCH_ZONE].center.x = 140 + 56 / 2;
	m_platformStructure.structures[RED_SWITCH_ZONE].center.y = (264 + 48 * 2) / 2;
	m_platformStructure.structures[RED_SWITCH_ZONE].sizeX = 4 * 12 + 8;
	m_platformStructure.structures[RED_SWITCH_ZONE].sizeY = 12 * 12 + 9.5;

	m_platformStructure.structures[BLUE_SWITCH_ZONE].objectId = 16;
	m_platformStructure.structures[BLUE_SWITCH_ZONE].center.x = (288 * 2 + 72) - 140 - 56 / 2;
	m_platformStructure.structures[BLUE_SWITCH_ZONE].center.y = (264 + 48 * 2) / 2;
	m_platformStructure.structures[BLUE_SWITCH_ZONE].sizeX = 4 * 12 + 8;
	m_platformStructure.structures[BLUE_SWITCH_ZONE].sizeY = 12 * 12 + 9.5;

	m_platformStructure.structures[SCALE_ZONE].objectId = 17;
	m_platformStructure.structures[SCALE_ZONE].center.x = (288 * 2 + 72) / 2;
	m_platformStructure.structures[SCALE_ZONE].center.y = (264 + 48 * 2) / 2;
	m_platformStructure.structures[SCALE_ZONE].sizeX = 4 * 12;
	m_platformStructure.structures[SCALE_ZONE].sizeY = 15 * 12;

	for (int i = CUBE_BY_RED_SWITCH; i < CUBE_BY_BLUE_SWITCH; i++) {
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position.x = 196 + 6;
		m_cubes[i].position.y = (float) ((264 + 48 * 2) / 2 - (12 * 12 + 9.5) / 2 + i * 24);
	}
	for (int i = CUBE_BY_BLUE_SWITCH; i < CUBE_BY_RED_POWER_ZONE; i++) {
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position.x = (288 * 2 + 72) - 196 - 6;
		m_cubes[i].position.y = (float)((264 + 48 * 2) / 2 - (12 * 12 + 9.5) / 2 + i * 24);
	}
	for (int i = CUBE_BY_RED_POWER_ZONE; i < CUBE_BY_BLUE_POWER_ZONE; i++) {
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position = m_platformStructure.redPowerCubeZone.center;
	}
	for (int i = CUBE_BY_BLUE_POWER_ZONE; i < CUBE_BY_RED_EXCHANGE_ZONE; i++) {
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position = m_platformStructure.bluePowerCubeZone.center;
	}
	for (int i = CUBE_BY_RED_EXCHANGE_ZONE; i < CUBE_BY_BLUE_EXCHANGE_ZONE; i++) {
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position = m_platformStructure.redExchangeZone.center;
	}
	for (int i = CUBE_BY_BLUE_EXCHANGE_ZONE; i < CUBE_LAST; i++) {
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position = m_platformStructure.blueExchangeZone.center;
	}

	//Note: center is not the actual object center. It is the position for the robot to arrive.
}


//Robot object ID is from 18 to 23, should convert these IDs to an enum, JWJW
void platform::configRedRobots(const robotConfigurationType config1In[NUMBER_OF_ROBOTS])
{
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_redRobots[i].setConfiguration(&config1In[i], this);
		m_redRobots[i].setAllianceType(ALLIANCE_RED);
	}
	//set robot initial positions
	m_redRobots[0].setPosition(config1In[0].sizeX / 2, config1In[0].sizeY * 2, 18);
	m_redRobots[1].setPosition(config1In[1].sizeX / 2, (264 + 48 * 2) / 2, 19);
	m_redRobots[2].setPosition(config1In[0].sizeX / 2, (264 + 48 * 2) - config1In[0].sizeY * 2, 20);
}

void platform::configBlueRobots(const robotConfigurationType config1In[NUMBER_OF_ROBOTS])
{
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_blueRobots[i].setConfiguration(&config1In[i], this);
		m_blueRobots[i].setAllianceType(ALLIANCE_BLUE);
	}
	//set robot initial positions
	m_blueRobots[0].setPosition((288 * 2 + 72) - config1In[0].sizeX / 2, config1In[0].sizeY * 2, 21);
	m_blueRobots[1].setPosition((288 * 2 + 72) - config1In[1].sizeX / 2, (264 + 48 * 2) / 2, 22);
	m_blueRobots[2].setPosition((288 * 2 + 72) - config1In[0].sizeX / 2, (264 + 48 * 2) - config1In[0].sizeY * 2, 23);
}


platform::~platform()
{
}


void platform::getFinalScore(int *pRedScoreOut, int *pBlueScoreOut)
{
	if (CLIMB_END_TIME > m_timeInSec) {
		updateScore(CLIMB_END_TIME - m_timeInSec);
	}
	*pRedScoreOut = getRedScore();
	*pBlueScoreOut = getBlueScore();
	return;
}

bool platform::isRobotLifted(allianceType allianceIn, int robotIdxIn)
{
	if (allianceIn == ALLIANCE_RED) {
		return m_state.redLiftFlag[robotIdxIn];
	}
	else {
		return m_state.blueLiftFlag[robotIdxIn];
	}
}

bool platform::hasPendingAction(int robotIndexIn, allianceType allianceIn)
{
	robot *pRobots;
	const pendingActionType *pPlannedAction;

	if (allianceIn == ALLIANCE_RED) {
		pRobots = m_redRobots;
	}
	else {
		pRobots = m_blueRobots;
	}

	pPlannedAction = pRobots[robotIndexIn].getPlannedAction();
	if (pPlannedAction->actionType != INVALID_ACTION) {
		return true;
	}
	else {
		return false;
	}
}


void platform::setRobotAction(searchActionType *pActionListInOut, allianceType allianceIn, int indexIn)
{
	robot *pRobots;
	int robotIdx = pActionListInOut->robotIndex;

	if (allianceIn == ALLIANCE_RED) {
		pRobots = m_redRobots;
	}
	else {
		pRobots = m_blueRobots;
	}

	pRobots[robotIdx].takeAction(pActionListInOut->actionType, m_timeInSec, indexIn);
	//update the projected start and finished time
	pActionListInOut->startTime = m_timeInSec;
	pActionListInOut->projectedFinishTime = pRobots[robotIdx].getPlannedActionFinishTime();
	//Note: pActionListInOut only has estimated start and stop time. After moves of other robots, the
	//      start and finish time may change.
}

int platform::commitAction(int indexIn)
{
	int updateActionResult = 0;
	bool isActionDone;
	float earliestFinishTime;
	const pendingActionType *pPlannetAction;
	actionTypeType actionType;

	if (m_timeInSec >= CLIMB_END_TIME) {
		return updateActionResult; //game is over, do nothing
		//Note: assume that all robots are trying to climb at the last 30 sec.
	}

	earliestFinishTime = CLIMB_END_TIME + 1;
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		if (hasPendingAction(i, ALLIANCE_RED)) {
			if (earliestFinishTime > m_redRobots[i].getPlannedActionFinishTime()) {
				earliestFinishTime = m_redRobots[i].getPlannedActionFinishTime();
			}
		}
		if (hasPendingAction(i, ALLIANCE_BLUE)) {
			if (earliestFinishTime > m_blueRobots[i].getPlannedActionFinishTime()) {
				earliestFinishTime = m_blueRobots[i].getPlannedActionFinishTime();
			}
		}
	}

	if (earliestFinishTime >= CLIMB_END_TIME) {
		//skip the action and only update the game score
		updateScore(CLIMB_END_TIME - m_timeInSec);
		return updateActionResult;
	}

	//run action of each robot
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		pPlannetAction = m_redRobots[i].getPlannedAction();
		actionType = pPlannetAction->actionType;
		if (actionType != INVALID_ACTION) {
			isActionDone = m_redRobots[i].moveToNextTime(earliestFinishTime);
			if (isActionDone) {
				updateActionResult = updateOneAction(actionType, earliestFinishTime, i, ALLIANCE_RED, indexIn);
			}

			if (updateActionResult != 0) {
				//error message is printed in updateOneAction();
				return updateActionResult;
			}
		}

		pPlannetAction = m_blueRobots[i].getPlannedAction();
		actionType = pPlannetAction->actionType;
		if (actionType != INVALID_ACTION) {
			isActionDone = m_blueRobots[i].moveToNextTime(earliestFinishTime);
			if (isActionDone) {
				updateActionResult = updateOneAction(actionType, earliestFinishTime, i, ALLIANCE_BLUE, indexIn);
			}

			if (updateActionResult != 0) {
				//error message is printed in updateOneAction();
				return updateActionResult;
			}
		}
	}

	return updateActionResult;
}

int platform::updateOneAction(actionTypeType actionIn, float timeIn, int robotIndexIn, allianceType allianceIn, int indexIn)
{
	int liftRebotCount = 0;

	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
		m_state.switchRed_RedBlockCount++;
		break;
	case CUBE_RED_DEFENCE_SWITCH:
		m_state.switchBlue_RedBlockCount++;
		break;
	case CUBE_RED_SCALE:
		m_state.scaleRedBlockCount++;
		break;
	case CUBE_RED_FORCE_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.forceRedBlockCount++;
		if (m_state.forceRedBlockCount > 3) {
			m_state.forceRedBlockCount = 3;
		}
		else {
			m_redScore += 5;
		}
		break;
	case CUBE_RED_BOOST_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.boostRedBlockCount++;
		if (m_state.boostRedBlockCount > 3) {
			m_state.boostRedBlockCount = 3;
		}
		else {
			m_redScore += 5;
		}
		break;
	case CUBE_RED_LIFT_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.liftRedBlockCount++;
		if (m_state.liftRedBlockCount > 3) {
			m_state.liftRedBlockCount = 3;
		}
		else {
			m_redScore += 5;
		}
		break;
	case PUSH_RED_FORCE_BUTTON:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		if (m_state.redForceButton == BUTTON_NOT_PUSH) {
			m_state.redForceButton = BUTTON_PUSH;
			m_state.forceRedButtonPushBlockCount = m_state.forceRedBlockCount;
		}
		else {
			return -1;
		}
		break;
	case PUSH_RED_BOOST_BUTTON:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		if (m_state.redBoostButton == BUTTON_NOT_PUSH) {
			m_state.redBoostButton = BUTTON_PUSH;
			m_state.boostRedButtonPushBlockCount = m_state.boostRedBlockCount;
		}
		else {
			return -1;
		}
	break;
	case PUSH_RED_LIFT_BUTTON:
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
			liftRebotCount += (m_state.redLiftFlag[i] == true);

		if ((m_state.liftRedBlockCount < 3) ||
			(liftRebotCount >= NUMBER_OF_ROBOTS) ||
			(m_timeInSec < COMPETITION_END_TIME)) {
			return -1;
		}else if (m_state.redLiftButton == BUTTON_NOT_PUSH) {
			m_state.redLiftButton = BUTTON_PUSH;
			m_state.liftRedButtonPushBlockCount = m_state.liftRedBlockCount;

		}
		else {
			return -1;
		}
		break;
	case LIFT_ONE_RED_ROBOT:
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
			liftRebotCount += (m_state.redLiftFlag[i] == true);

		//lift button plus lift robots are enough
		if ((liftRebotCount == NUMBER_OF_ROBOTS - 1) &&
			(m_state.redLiftButton != BUTTON_NOT_PUSH)) {
			return -1;
		}

		if ((m_state.redLiftFlag[robotIndexIn]) ||
			(m_timeInSec < COMPETITION_END_TIME)) {
			return -1;
		}
		m_liftRedRobotIndex = robotIndexIn;
		break;
		////////////////////
	case CUBE_BLUE_OFFENCE_SWITCH:
		m_state.switchBlue_BlueBlockCount++;
		break;
	case CUBE_BLUE_DEFENCE_SWITCH:
		m_state.switchRed_BlueBlockCount++;
		break;
	case CUBE_BLUE_SCALE:
		m_state.scaleBlueBlockCount++;
		break;
	case CUBE_BLUE_FORCE_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.forceBlueBlockCount++;
		if (m_state.forceBlueBlockCount > 3) {
			m_state.forceBlueBlockCount = 3;
		}
		else {
			m_blueScore += 5;
		}
		break;
	case CUBE_BLUE_BOOST_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.boostBlueBlockCount++;
		if (m_state.boostBlueBlockCount > 3) {
			m_state.boostBlueBlockCount = 3;
		}
		else {
			m_blueScore += 5;
		}
		break;
	case CUBE_BLUE_LIFT_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.liftBlueBlockCount++;
		if (m_state.liftBlueBlockCount > 3) {
			m_state.liftBlueBlockCount = 3;
		}
		else {
			m_blueScore += 5;
		}
		break;
	case PUSH_BLUE_FORCE_BUTTON:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		if (m_state.blueForceButton == BUTTON_NOT_PUSH) {
			m_state.blueForceButton = BUTTON_PUSH;
			m_state.forceBlueButtonPushBlockCount = m_state.forceBlueBlockCount;
		}
		else {
			return -1;
		}
		break;
	case PUSH_BLUE_BOOST_BUTTON:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		if (m_state.blueBoostButton == BUTTON_NOT_PUSH) {
			m_state.blueBoostButton = BUTTON_PUSH;
			m_state.boostBlueButtonPushBlockCount = m_state.boostBlueBlockCount;
		}
		else {
			return -1;
		}
		break;
	case PUSH_BLUE_LIFT_BUTTON:
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
			liftRebotCount += (m_state.blueLiftFlag[i] == true);

		if ((m_state.liftBlueBlockCount < 3) ||
			(liftRebotCount >= NUMBER_OF_ROBOTS) ||
			(m_timeInSec < COMPETITION_END_TIME)) {
			return -1;
		}else if (m_state.blueLiftButton == BUTTON_NOT_PUSH) {
			m_state.blueLiftButton = BUTTON_PUSH;
			m_state.liftBlueButtonPushBlockCount = m_state.liftBlueBlockCount;
		}
		else {
			return -1;
		}
		break;
	case LIFT_ONE_BLUE_ROBOT:
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
			liftRebotCount += (m_state.blueLiftFlag[i] == true);

		if ((liftRebotCount == NUMBER_OF_ROBOTS - 1) &&
			(m_state.blueLiftButton != BUTTON_NOT_PUSH)) {
			return -1;
		}

		if ((m_state.blueLiftFlag[robotIndexIn]) ||
		    (m_timeInSec < COMPETITION_END_TIME)) {
			return -1;
		}
		m_liftBlueRobotIndex = robotIndexIn;
		break;
	case RED_ACTION_NONE:
	case BLUE_ACTION_NONE:
		break; //no action, just time pass
	default:
		printf("ERROR: invalid action %d\n", actionIn);
		break;
	}

	if (timeIn < m_timeInSec) {
		printf("Error, time reverse, from %3.2f to %3.2f\n", m_timeInSec, timeIn);
	}
	updateScore(timeIn - m_timeInSec);
	logAction(actionIn, timeIn, robotIndexIn, indexIn);
	return 0;
}

int platform::isGameTimeOver(void)
{
	if (m_timeInSec >= CLIMB_END_TIME) {
		return 1;
	}
	else {
		return 0;
	}
}

int platform::updateScaleSwitchScore(float secondsIn, int vaultForceBlockCountIn, int vaultBoostBlockCountIn, int balanceBlockDifferenceIn,
	vaultButtonStateType forceVaultButtonIn, vaultButtonStateType boostVaultButtonIn,
	int vaultBlockSelectionIn, ownerShipType newOnershipIn, ownerShipType *pOwnershipInOut)
{
	int scores = 0;
	bool ownershipChangeFlag;

	ownershipChangeFlag = false;
	if ((balanceBlockDifferenceIn >= MIN_BLOCK_DIFFERENCE_TO_SCORE) ||
		((forceVaultButtonIn >= BUTTON_PUSH_0SEC) && (forceVaultButtonIn <= BUTTON_PUSH_9SEC) &&
		((vaultForceBlockCountIn == vaultBlockSelectionIn) || (vaultForceBlockCountIn == 3)))) {

		//get one more point for balance change
		if (balanceBlockDifferenceIn >= MIN_BLOCK_DIFFERENCE_TO_SCORE) {
			if (*pOwnershipInOut != newOnershipIn) {
				*pOwnershipInOut = newOnershipIn;
				ownershipChangeFlag = true;
				scores++;
				if (m_timeInSec + secondsIn < AUTONOMOUS_END_TIME) {
					scores++; //in autonomous session, ownership change worth 2 points
				}
			}
		}

		if ((*pOwnershipInOut == newOnershipIn) &&
			(ownershipChangeFlag == false)) { //ownership change happens before

			scores += ROUNDING_METHOD(secondsIn);
			if (m_timeInSec + secondsIn < AUTONOMOUS_END_TIME) {
				scores += ROUNDING_METHOD(secondsIn);
			}
			else if (m_timeInSec < AUTONOMOUS_END_TIME) {
				scores += ROUNDING_METHOD(AUTONOMOUS_END_TIME - m_timeInSec);
			}
			//Note: because button push is not allowed in autonomous session,
			//      the logic above doesn't distinguish the ownership caused by cubes
			//      or force button.
		}
		else if ((forceVaultButtonIn >= BUTTON_PUSH_0SEC) && (forceVaultButtonIn <= BUTTON_PUSH_9SEC) &&
			     ((vaultForceBlockCountIn == vaultBlockSelectionIn) || (vaultForceBlockCountIn == 3))) {
			//it must be force button on
			//only get scores within force button 10 seconds
			if (forceVaultButtonIn + secondsIn <= BUTTON_PUSH_OVER_10SEC) {
				scores += ROUNDING_METHOD(secondsIn);
			}
			else {
				scores += BUTTON_PUSH_OVER_10SEC - forceVaultButtonIn;
			}
		}

		if ((boostVaultButtonIn >= BUTTON_PUSH_0SEC) && (boostVaultButtonIn < BUTTON_PUSH_OVER_10SEC) &&
			((vaultBoostBlockCountIn == vaultBlockSelectionIn) || (vaultBoostBlockCountIn == 3))) {

			//double the score
			if ((*pOwnershipInOut == newOnershipIn) && (ownershipChangeFlag == false)) {
				if (boostVaultButtonIn + secondsIn <= BUTTON_PUSH_OVER_10SEC) {
					scores += ROUNDING_METHOD(secondsIn);
				}
				else {
					scores += BUTTON_PUSH_OVER_10SEC - boostVaultButtonIn;
				}
			}
			else {
				//by force button
				if ((forceVaultButtonIn + secondsIn <= BUTTON_PUSH_OVER_10SEC) &&
					(boostVaultButtonIn + secondsIn <= BUTTON_PUSH_OVER_10SEC)) {
					scores += ROUNDING_METHOD(secondsIn);
				}
				else {
					if (forceVaultButtonIn >= boostVaultButtonIn) {
						scores += BUTTON_PUSH_OVER_10SEC - forceVaultButtonIn;
					}
					else {
						scores += BUTTON_PUSH_OVER_10SEC - boostVaultButtonIn;
					}
				}
			}
		}
	}
	else {
		if (*pOwnershipInOut == newOnershipIn) {
			*pOwnershipInOut = OWNED_BY_NONE;
		}
	}

	return scores;
}

void platform::updateScore(float secondsIn)
{
	int liftRebotCount;

	vaultButtonStateType previousRedForceButton = BUTTON_NOT_PUSH;
	vaultButtonStateType previousBlueForceButton = BUTTON_NOT_PUSH;
	vaultButtonStateType previousRedBoostButton = BUTTON_NOT_PUSH;
	vaultButtonStateType previousBlueBoostButton = BUTTON_NOT_PUSH;

	//force button could be queued if the opponent is in force
	if ((m_state.blueForceButton >= BUTTON_PUSH) && (m_state.blueForceButton != BUTTON_PUSH_OVER_10SEC)) {
		//skip update of red force and boost button, wait blue force done
	}
	else {
		if ((m_state.redForceButton != BUTTON_NOT_PUSH) &&
			((m_state.redForceButton != BUTTON_PUSH_OVER_10SEC))) {

			previousRedForceButton = m_state.redForceButton;  //red force is enabled

			m_state.redForceButton = (vaultButtonStateType)(m_state.redForceButton + ROUNDING_METHOD(secondsIn));
			if (m_state.redForceButton > BUTTON_PUSH_OVER_10SEC) {
				m_state.redForceButton = BUTTON_PUSH_OVER_10SEC;
			}
		}

		if ((m_state.redBoostButton != BUTTON_NOT_PUSH) &&
			((m_state.redBoostButton != BUTTON_PUSH_OVER_10SEC))) {

			previousRedBoostButton = m_state.redBoostButton; //red boost is enabled

			m_state.redBoostButton = (vaultButtonStateType)(m_state.redBoostButton + ROUNDING_METHOD(secondsIn));
			if (m_state.redBoostButton > BUTTON_PUSH_OVER_10SEC) {
				m_state.redBoostButton = BUTTON_PUSH_OVER_10SEC;
			}
		}
	}
	if ((m_state.redForceButton >= BUTTON_PUSH) && (m_state.redForceButton != BUTTON_PUSH_OVER_10SEC)) {
		//skip update of blue force and boost button, wait red force button done
	}
	else {
		if ((m_state.blueForceButton != BUTTON_NOT_PUSH) &&
			((m_state.blueForceButton != BUTTON_PUSH_OVER_10SEC))) {

			previousBlueForceButton = m_state.blueForceButton;

			m_state.blueForceButton = (vaultButtonStateType)(m_state.blueForceButton + ROUNDING_METHOD(secondsIn));
			if (m_state.blueForceButton > BUTTON_PUSH_OVER_10SEC) {
				m_state.blueForceButton = BUTTON_PUSH_OVER_10SEC;
			}
		}

		if ((m_state.blueBoostButton != BUTTON_NOT_PUSH) &&
			((m_state.blueBoostButton != BUTTON_PUSH_OVER_10SEC))) {

			previousBlueBoostButton = m_state.blueBoostButton;

			m_state.blueBoostButton = (vaultButtonStateType)(m_state.blueBoostButton + ROUNDING_METHOD(secondsIn));
			if (m_state.blueBoostButton > BUTTON_PUSH_OVER_10SEC) {
				m_state.blueBoostButton = BUTTON_PUSH_OVER_10SEC;
			}
		}
	}

	m_redScore += updateScaleSwitchScore(secondsIn, m_state.forceRedButtonPushBlockCount, m_state.boostRedButtonPushBlockCount,
		m_state.scaleRedBlockCount - m_state.scaleBlueBlockCount,
		previousRedForceButton, previousRedBoostButton, 2, OWNED_BY_RED, &m_state.scaleOwner);

	m_blueScore += updateScaleSwitchScore(secondsIn, m_state.forceBlueButtonPushBlockCount, m_state.boostBlueButtonPushBlockCount,
		m_state.scaleBlueBlockCount - m_state.scaleRedBlockCount,
		previousBlueForceButton, previousBlueBoostButton, 2, OWNED_BY_BLUE, &m_state.scaleOwner);

	m_redScore += updateScaleSwitchScore(secondsIn, m_state.forceRedButtonPushBlockCount, m_state.boostRedButtonPushBlockCount,
		m_state.switchRed_RedBlockCount - m_state.switchRed_BlueBlockCount,
		previousRedForceButton, previousRedBoostButton, 2, OWNED_BY_RED, &m_state.switchRedOwner);

	m_blueScore += updateScaleSwitchScore(secondsIn, m_state.forceBlueButtonPushBlockCount, m_state.boostBlueButtonPushBlockCount,
		m_state.switchBlue_BlueBlockCount - m_state.switchBlue_RedBlockCount,
		previousBlueForceButton, previousBlueBoostButton, 2, OWNED_BY_BLUE, &m_state.switchBlueOwner);

	if (m_timeInSec >= COMPETITION_END_TIME) {
		//lift button is only allowed at the last 30 seconds

		//red lifting
		if ((m_state.redLiftButton == BUTTON_PUSH) && (m_state.liftRedBlockCount >= 3)) {
			m_redScore += 30;
			m_state.redLiftButton = BUTTON_PUSH_OVER_10SEC;
		}
		else if(m_state.liftRedButtonPushBlockCount < 3) {
			m_state.redLiftButton = BUTTON_NOT_PUSH; //less than 3 cubes, button push ignored
		}

		if (m_liftRedRobotIndex < NUMBER_OF_ROBOTS) {
			m_state.redLiftFlag[m_liftRedRobotIndex] = true;
			m_redScore += 30;
			m_liftRedRobotIndex = INVALID_IDX;
		}

		liftRebotCount = (m_state.redLiftButton == BUTTON_PUSH_OVER_10SEC);
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
			liftRebotCount += (m_state.redLiftFlag[i] == true);

		if (liftRebotCount == NUMBER_OF_ROBOTS) {
			m_redScore += 1;
		}

		//blue lifting
		if ((m_state.blueLiftButton == BUTTON_PUSH) && (m_state.liftBlueBlockCount >= 3)) {
			m_blueScore += 30;
			m_state.blueLiftButton = BUTTON_PUSH_OVER_10SEC;
		}
		else if (m_state.liftBlueButtonPushBlockCount < 3) {
			m_state.blueLiftButton = BUTTON_NOT_PUSH; //less than 3 cubes, button push ignored
		}

		if (m_liftBlueRobotIndex < NUMBER_OF_ROBOTS) {
			m_state.blueLiftFlag[m_liftBlueRobotIndex] = true;
			m_blueScore += 30;
			m_liftBlueRobotIndex = INVALID_IDX;
		}

		liftRebotCount = (m_state.blueLiftButton == BUTTON_PUSH_OVER_10SEC);
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
			liftRebotCount += (m_state.blueLiftFlag[i] == true);

		if (liftRebotCount == NUMBER_OF_ROBOTS) {
			m_blueScore += 1;
		}
	}
	else {
		//otherwise, the push signal is ignored and reset the button state
		m_state.redLiftButton = BUTTON_NOT_PUSH;
		m_state.blueLiftButton = BUTTON_NOT_PUSH;
	}

	//update game time
	m_timeInSec += secondsIn;
}

void platform::logAction(actionTypeType actionIn, float timeIn, int robotIndexIn, int indexIn)
{
	if (m_pLogFIle == NULL) {
		return;
	}
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
	case LIFT_ONE_RED_ROBOT:
	case RED_ACTION_NONE:
		fprintf(m_pLogFIle, "%d. Time %3.2f (sec), Red alliance robot[%d]: ", indexIn, timeIn, robotIndexIn);
		break;
	case CUBE_BLUE_OFFENCE_SWITCH:
	case CUBE_BLUE_DEFENCE_SWITCH:
	case CUBE_BLUE_SCALE:
	case CUBE_BLUE_FORCE_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
	case CUBE_BLUE_LIFT_VAULT:
	case PUSH_BLUE_FORCE_BUTTON:
	case PUSH_BLUE_BOOST_BUTTON:
	case PUSH_BLUE_LIFT_BUTTON:
	case LIFT_ONE_BLUE_ROBOT:
	case BLUE_ACTION_NONE:
		fprintf(m_pLogFIle, "%d. Time %3.2f (sec), Blue alliance robot[%d]: ", indexIn, timeIn, robotIndexIn);
		break;
	default:
		fprintf(m_pLogFIle, "%d. Time %3.2f (sec), unknown alliance (ERROR)", indexIn, timeIn);
		break;
	}
	
	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
		fprintf(m_pLogFIle, "put a cube on red side switch");
		break;
	case CUBE_BLUE_OFFENCE_SWITCH:
		fprintf(m_pLogFIle, "put a cube on blue side switch");
		break;
	case CUBE_RED_DEFENCE_SWITCH:
		fprintf(m_pLogFIle, "put a cube on blue side switch");
		break;
	case CUBE_BLUE_DEFENCE_SWITCH:
		fprintf(m_pLogFIle, "put a cube on red side switch");
		break;
	case CUBE_RED_SCALE:
	case CUBE_BLUE_SCALE:
		fprintf(m_pLogFIle, "put a cube on the scale");
		break;
	case CUBE_RED_FORCE_VAULT:
	case CUBE_BLUE_FORCE_VAULT:
		fprintf(m_pLogFIle, "add a cube to force vault");
		break;
	case CUBE_RED_BOOST_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
		fprintf(m_pLogFIle, "add a cube to boost vault");
		break;
	case CUBE_RED_LIFT_VAULT:
	case CUBE_BLUE_LIFT_VAULT:
		fprintf(m_pLogFIle, "add a cube to lift vault");
		break;
	case PUSH_RED_FORCE_BUTTON:
	case PUSH_BLUE_FORCE_BUTTON:
		fprintf(m_pLogFIle, "push force vault button");
		break;
	case PUSH_RED_BOOST_BUTTON:
	case PUSH_BLUE_BOOST_BUTTON:
		fprintf(m_pLogFIle, "push boost vault button");
		break;
	case PUSH_RED_LIFT_BUTTON:
	case PUSH_BLUE_LIFT_BUTTON:
		fprintf(m_pLogFIle, "push lift vault button");
		break;
	case LIFT_ONE_RED_ROBOT:
	case LIFT_ONE_BLUE_ROBOT:
		fprintf(m_pLogFIle, "lift robot on platform");
		break;
	case RED_ACTION_NONE:
	case BLUE_ACTION_NONE:
		fprintf(m_pLogFIle, "no action");
		break;
	default:
		fprintf(m_pLogFIle, "unknown action (ERROR)");
		break;
	}
	fprintf(m_pLogFIle, ", red score %d, blue score %d\n", getRedScore(), getBlueScore());
}

void platform::logFinalScore(void)
{
	int finalRedScore, finalBlueScore;
	if (m_pLogFIle == NULL) {
		return;
	}
	getFinalScore(&finalRedScore, &finalBlueScore);
	fprintf(m_pLogFIle, "================= The final score is (red score %d, blue score %d) ================\n",
		finalRedScore, finalBlueScore);
	fflush(m_pLogFIle);
}

float platform::findOneCube(float shortestPathIn, int startSearchIdxIn, int endSearchIdxIn, bool isAllCubeSameFlag,
	const rectangleObjectType *pMovingObjectIn, cubeStateType **pCubeOut, robotPathType *pPathOut)
{
	float shortestPath = shortestPathIn;
	robotPathType nextPath;

	//search for cubs by the switch
	for (int i = startSearchIdxIn; i < endSearchIdxIn; i++) {
		if (!m_cubes[i].availbleFlag) {
			continue;
		}

		if (findAvailablePath(pMovingObjectIn, m_cubes[i].position, true, &nextPath)) {
			if (shortestPath > nextPath.totalDistance) {
				memcpy(pPathOut, &nextPath, sizeof(robotPathType));
				*pCubeOut =  &m_cubes[i];
				shortestPath = nextPath.totalDistance;
			}
		}
		//It is a simple implementation, the number of turns is not counted.

		if (isAllCubeSameFlag) {
			break; //all the cubes are the same, find available one is enough
		}
	}

	return shortestPath;
}

const cubeSearchRangeType redCubeSearchRange[] =
{	{ CUBE_BY_RED_SWITCH , CUBE_BY_RED_POWER_ZONE },  //cubes by switch
	{ CUBE_BY_RED_POWER_ZONE , CUBE_BY_BLUE_POWER_ZONE },  //CUBE BY POWER ZONE
	{ CUBE_BY_RED_EXCHANGE_ZONE , CUBE_BY_BLUE_EXCHANGE_ZONE }   //CUBE BY EXCHANGE ZONE
};

const cubeSearchRangeType blueCubeSearchRange[] =
{	{ CUBE_BY_RED_SWITCH , CUBE_BY_RED_POWER_ZONE },  //cubes by switch
	{ CUBE_BY_BLUE_POWER_ZONE , CUBE_BY_RED_EXCHANGE_ZONE },  //CUBE BY POWER ZONE
	{ CUBE_BY_BLUE_EXCHANGE_ZONE , CUBE_LAST }   //CUBE BY EXCHANGE ZONE
};

const float DISTANCE_OUT_OF_RANGE = 1000000;
bool platform::findTheClosestCube(const rectangleObjectType *pMovingObjectIn, allianceType allianceIn, cubeStateType **pCubeOut, robotPathType *pPathOut)
{
	float shortestPath = DISTANCE_OUT_OF_RANGE;

	pPathOut->numberOfTurns = 0;

	//search power zone and exchange zone
	if (allianceIn == ALLIANCE_RED) {
		for (int i = 0; i < sizeof(redCubeSearchRange) / sizeof(cubeSearchRangeType); i++) {
			shortestPath = findOneCube(shortestPath, redCubeSearchRange[i].startIdx, redCubeSearchRange[i].endIdx, true,
				pMovingObjectIn, pCubeOut, pPathOut);

		}
	}
	else {
		for (int i = 0; i < sizeof(blueCubeSearchRange) / sizeof(cubeSearchRangeType); i++) {
			shortestPath = findOneCube(shortestPath, blueCubeSearchRange[i].startIdx, blueCubeSearchRange[i].endIdx, true,
				pMovingObjectIn, pCubeOut, pPathOut);

		}
	}

	if (shortestPath >= DISTANCE_OUT_OF_RANGE) {
		return false;
	}

	//the last turn point is the point to pick up a cube
	pPathOut->pickUpCubeIndex = pPathOut->numberOfTurns - 1;
	return true;
}

int platform::pickUpCube(coordinateType positionIn, allianceType allianceIn)
{
	if (allianceIn == ALLIANCE_RED) {
		for (int i = 0; i < sizeof(redCubeSearchRange) / sizeof(cubeSearchRangeType); i++) {
			if (tryPickOneCube(positionIn, m_cubes[i].position) ){
				return i;
			}
		}
	}
	else {
		for (int i = 0; i < sizeof(blueCubeSearchRange) / sizeof(cubeSearchRangeType); i++) {
			if (tryPickOneCube(positionIn, m_cubes[i].position)) {
				return i;
			}
		}
	}
	return INVALID_IDX;
}

bool  platform::tryPickOneCube(coordinateType robotPosIn, coordinateType cubePosIn)
{
	float distance;

	distance = (robotPosIn.x - cubePosIn.x) * (robotPosIn.x - cubePosIn.x) + (robotPosIn.y - cubePosIn.y) * (robotPosIn.y - cubePosIn.y);
	distance = (float) sqrt(distance);

	if (distance <= PICK_UP_CUBE_DISTANCE) {
		return true;
	}
	else {
		return false;
	}
}


bool platform::findAvailablePath(const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn,
	bool isTargetACubeIn, robotPathType *pPathOut)
{
	//scan all static objects to find out a shortest path to the end point

	//the path selection algorithm is,
	//1. First, the direct path to the target position is tried. 
	//2. If the first try failed, the program always searching for a path beside north or south walls. 
	//3. If there is a robot block the path, go to the opposite wall and back to step 2. Otherwise, proceed to the desired x coordinate
	//4. directly go the target position. if failed, goto the opposite wall and repeat step 2.

	//Note: if the target is a cube, and, the path is blocked by a cube, just stop at the cube

	bool collisionDetectedFlag;
	const rectangleObjectType *pCollisionObject;
	rectangleObjectType movingObject;
	int turnPointIndex = 0;
	float distance;
	coordinateType startPoint;
	coordinateType arroundPos;

	memcpy(&movingObject, pMovingObjectIn, sizeof(movingObject));

	//check if it is arrived 
	if ((movingObject.center.x == endPointIn.x) && (movingObject.center.y == endPointIn.y)) {
		pPathOut->numberOfTurns = 0;
		pPathOut->totalDistance = 0;
		return true;
	}

	//try the shortest path
	collisionDetectedFlag = collisionWithAllOtherObjects(&movingObject, endPointIn, &pCollisionObject);

	if (!collisionDetectedFlag) {
		//return the shortest path
		pPathOut->turnPoints[turnPointIndex] = endPointIn;
		turnPointIndex++;
		pPathOut->numberOfTurns = turnPointIndex;
		distance = (endPointIn.x - pMovingObjectIn->center.x)*(endPointIn.x - pMovingObjectIn->center.x) +
			(endPointIn.y - pMovingObjectIn->center.y)*(endPointIn.y - pMovingObjectIn->center.y);

		pPathOut->totalDistance = (float) sqrt(distance);
		return true;
	}

	//else, try go along the wall

	//Y direction first
	coordinateType upToWall1;
	coordinateType upToWall2;
	coordinateType oppositeWall;
	bool retryGoToWallFlag = false;
	bool aroundBlockingObjectFailedFlag;

	if (endPointIn.y >= pMovingObjectIn->center.y) {
		upToWall1.x = pMovingObjectIn->center.x;
		upToWall1.y = m_platformStructure.northWall - pMovingObjectIn->sizeY / 2 - ROBOT_TO_WALL_DISTANCE;

		upToWall2.x = pMovingObjectIn->center.x;
		upToWall2.y = m_platformStructure.southWall + pMovingObjectIn->sizeY / 2 + ROBOT_TO_WALL_DISTANCE;
	}
	else {
		upToWall2.x = pMovingObjectIn->center.x;
		upToWall2.y = m_platformStructure.northWall - pMovingObjectIn->sizeY / 2 - ROBOT_TO_WALL_DISTANCE;

		upToWall1.x = pMovingObjectIn->center.x;
		upToWall1.y = m_platformStructure.southWall + pMovingObjectIn->sizeY / 2 + ROBOT_TO_WALL_DISTANCE;
	}

	oppositeWall = upToWall2;
	collisionDetectedFlag = collisionWithAllOtherObjects(&movingObject, upToWall1, &pCollisionObject);
	if (collisionDetectedFlag) {
		aroundBlockingObjectFailedFlag = true;
		if ((pCollisionObject->objectId >= 18) && (pCollisionObject->objectId <= 23)) {
			//try move a little around the blocking robot
			if (upToWall1.x > pCollisionObject->sizeX) {
				arroundPos.x = upToWall1.x - (pCollisionObject->sizeX + ROBOT_TO_WALL_DISTANCE);
			}
			else {
				arroundPos.x = upToWall1.x + pCollisionObject->sizeX + ROBOT_TO_WALL_DISTANCE;
			}
			arroundPos.y = pMovingObjectIn->center.y;

			collisionDetectedFlag = collisionWithAllOtherObjects(&movingObject, arroundPos, &pCollisionObject);
			if (!collisionDetectedFlag) {
				aroundBlockingObjectFailedFlag = false;

				if (turnPointIndex >= MAX_TURNS_ON_PATH) {
					printf("ERROR, turning points buffer overflow\n");
					return false;
				}
				pPathOut->turnPoints[turnPointIndex] = arroundPos;
				turnPointIndex++;

				movingObject.center = arroundPos;

				//adjust x position of up to wall point
				upToWall1.x = arroundPos.x;
				retryGoToWallFlag = true;
			}
			//else, just go to the opposite wall
		}

		if(aroundBlockingObjectFailedFlag) {
			//cannot go to the wall, try the opposite wall
			oppositeWall = upToWall1;
			upToWall1 = upToWall2;
			retryGoToWallFlag = true;
		}
	}

	for (int i = 0; i < MAX_WALL_TO_WALL_MOVES; i++) {

		if (retryGoToWallFlag) {
			collisionDetectedFlag = collisionWithAllOtherObjects(&movingObject, upToWall1, &pCollisionObject);
			if (collisionDetectedFlag) {
				return false; //cannot find a path to the opposite wall, give up
			}
		}

		//save the path to the wall
		if (turnPointIndex >= MAX_TURNS_ON_PATH) {
			printf("ERROR, turning points buffer overflow\n");
			return false;
		}
		pPathOut->turnPoints[turnPointIndex] = upToWall1;
		turnPointIndex++;
		movingObject.center = upToWall1;

		//move the object along the wall
		coordinateType alongWall;
		alongWall.y = upToWall1.y;
		alongWall.x = endPointIn.x;

		//move as far as possible along the wall
		do {
			collisionDetectedFlag = collisionWithAllOtherObjects(&movingObject, alongWall, &pCollisionObject);

			if (collisionDetectedFlag) {
				if (alongWall.x > movingObject.center.x) {
					alongWall.x = pCollisionObject->center.x - pCollisionObject->sizeX / 2 - movingObject.sizeX / 2 - ROBOT_TO_WALL_DISTANCE;

					if (alongWall.x <= movingObject.center.x + ROBOT_TO_WALL_DISTANCE) {
						//no move at all, give up
						return false;
					}
				}
				else {
					alongWall.x = pCollisionObject->center.x + pCollisionObject->sizeX / 2 + movingObject.sizeX / 2 + ROBOT_TO_WALL_DISTANCE;

					if (alongWall.x >= movingObject.center.x - ROBOT_TO_WALL_DISTANCE) {
						//no move at all, give up
						return false;
					}

				}
			}
		} while (collisionDetectedFlag);

		//check again if the direct path is OK
		movingObject.center = alongWall;
		collisionDetectedFlag = collisionWithAllOtherObjects(&movingObject, endPointIn, &pCollisionObject);

		if (!collisionDetectedFlag) {
			//success
			if (turnPointIndex+1 >= MAX_TURNS_ON_PATH) {
				printf("ERROR, turning points buffer overflow\n");
				return false;
			}

			pPathOut->turnPoints[turnPointIndex] = alongWall;
			turnPointIndex++;
			pPathOut->turnPoints[turnPointIndex] = endPointIn;
			turnPointIndex++;
			pPathOut->numberOfTurns = turnPointIndex;

			startPoint = pMovingObjectIn->center;
			pPathOut->totalDistance = 0;

			for (int p = 0; p < turnPointIndex; p++) {

				distance = (pPathOut->turnPoints[p].x - startPoint.x)*(pPathOut->turnPoints[p].x - startPoint.x) +
					(pPathOut->turnPoints[p].y - startPoint.y)*(pPathOut->turnPoints[p].y - startPoint.y);

				pPathOut->totalDistance += (float)sqrt(distance);
			}

			return true;
		}

		//TODO, please confirm that blue switch is on the right side, JWJW
		float rightMostPath = m_platformStructure.blueSwitchSouthPlate.center.x +
			m_platformStructure.blueSwitchSouthPlate.sizeX / 2 +
			m_platformStructure.bluePowerCubeZone.sizeX +
			ROBOT_TO_WALL_DISTANCE;
		float secondRightPath = m_platformStructure.scaleSouthPlate.center.x +
			m_platformStructure.scaleSouthPlate.sizeX / 2 +
			ROBOT_TO_WALL_DISTANCE;
		float thirdRightPath = m_platformStructure.scaleSouthPlate.center.x -
			m_platformStructure.scaleSouthPlate.sizeX / 2 -
			ROBOT_TO_WALL_DISTANCE;
		float forthRightPath = m_platformStructure.blueSwitchSouthPlate.center.x +
			m_platformStructure.blueSwitchSouthPlate.sizeX / 2 +
			m_platformStructure.bluePowerCubeZone.sizeX +
			ROBOT_TO_WALL_DISTANCE;

		//else, adjust along the wall to the next opening between alongWall and upToWall1
		if (alongWall.x > upToWall1.x) {

			if ((alongWall.x >= rightMostPath) && (upToWall1.x <= rightMostPath)) {
				alongWall.x = rightMostPath;
			}
			else if ((alongWall.x >= secondRightPath) && (upToWall1.x <= secondRightPath)) {
				alongWall.x = secondRightPath;
			}
			else if ((alongWall.x >= thirdRightPath) && (upToWall1.x <= thirdRightPath)) {
				alongWall.x = thirdRightPath;
			}
			else {
				alongWall.x = forthRightPath;
			}
		}
		else {
			if ((alongWall.x >= forthRightPath) && (upToWall1.x <= forthRightPath)) {
				alongWall.x = forthRightPath;
			}
			else if ((alongWall.x >= thirdRightPath) && (upToWall1.x <= thirdRightPath)) {
				alongWall.x = thirdRightPath;
			}
			else if ((alongWall.x >= secondRightPath) && (upToWall1.x <= secondRightPath)) {
				alongWall.x = secondRightPath;
			}
			else {
				alongWall.x = rightMostPath;
			}

		}

		//save the path along the wall
		if (turnPointIndex >= MAX_TURNS_ON_PATH) {
			printf("ERROR, turning points buffer overflow\n");
			return false;
		}

		pPathOut->turnPoints[turnPointIndex] = alongWall;
		turnPointIndex++;
		movingObject.center = alongWall;

		//goto the opposite wall
		upToWall2 = oppositeWall;
		oppositeWall = upToWall1;
		upToWall1 = upToWall2;
		retryGoToWallFlag = true;
	}

	//cannot find any path
	return false;
}

bool platform::collisionWithAllOtherObjects(const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn,
	                                        const rectangleObjectType **pCollisionObjectOut)
{
	const robotStateType *pRobotState;
	const rectangleObjectType *pRobotPosition;

	for (int i = 0; i < NUM_STILL_STRUCTURE; i++) {
		if (collisionDectection(&m_platformStructure.structures[i], pMovingObjectIn, endPointIn)) {
			*pCollisionObjectOut = &m_platformStructure.structures[i];
			return true;
		}
	}

	//else
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		pRobotState =  m_redRobots[i].getState();
		pRobotPosition = &pRobotState->pos;

		for (int j = 0; j < 2; j++) {
			if (pRobotPosition->objectId != pMovingObjectIn->objectId) {
				if (collisionDectection(pRobotPosition, pMovingObjectIn, endPointIn)) {
					*pCollisionObjectOut = pRobotPosition;
					return true;
				}
			}

			pRobotState = m_blueRobots[i].getState();
			pRobotPosition = &pRobotState->pos;
		}
	}
	return false;
}


bool platform::collisionDectection(const rectangleObjectType *pStillObjectIn, const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn)
{
	float mLeftX, mTopY, mBottomY, mRightX;
	float sLeftX, sTopY, sBottomY, sRightX;

	//find moving object covered rectangle
	mLeftX = pMovingObjectIn->center.x - pMovingObjectIn->sizeX / 2;
	mRightX = pMovingObjectIn->center.x + pMovingObjectIn->sizeX / 2;
	mTopY = pMovingObjectIn->center.y + pMovingObjectIn->sizeY / 2;
	mBottomY = pMovingObjectIn->center.y - pMovingObjectIn->sizeY / 2;

	if (endPointIn.x > pMovingObjectIn->center.x) {
		//move to right
		mRightX += endPointIn.x - pMovingObjectIn->center.x;
	}
	else {
		//move left
		mLeftX -= pMovingObjectIn->center.x - endPointIn.x;
	}

	if (endPointIn.y > pMovingObjectIn->center.y) {
		//move up
		mTopY += endPointIn.y - pMovingObjectIn->center.y;
	}
	else {
		//move down
		mBottomY -= pMovingObjectIn->center.y - endPointIn.y;
	}

	//find the still object rectangle
	sLeftX = pStillObjectIn->center.x - pStillObjectIn->sizeX/2;
	sRightX = pStillObjectIn->center.x + pStillObjectIn->sizeX/2;
	sTopY = pStillObjectIn->center.y + pStillObjectIn->sizeY / 2;
	sBottomY = pStillObjectIn->center.y - pStillObjectIn->sizeY / 2;

	//test if two rectangles overlap on 4 corners
	if (pointInRectangle(mLeftX, mTopY, mBottomY, mRightX, sLeftX, sTopY)) {
		return true;
	}
	if (pointInRectangle(mLeftX, mTopY, mBottomY, mRightX, sRightX, sTopY)) {
		return true;
	}
	if (pointInRectangle(mLeftX, mTopY, mBottomY, mRightX, sLeftX, sBottomY)) {
		return true;
	}
	if (pointInRectangle(mLeftX, mTopY, mBottomY, mRightX, sRightX, sBottomY)) {
		return true;
	}

	return false;
}