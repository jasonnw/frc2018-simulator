
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
	m_liftRedRobotIndex = INT_MAX;
	m_liftBlueRobotIndex = INT_MAX;
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

int platform::takeAction(actionTypeType actionIn, float timeIn, int robotIndexIn, int indexIn)
{
	int liftRebotCount = 0;

	if (m_timeInSec >= CLIMB_END_TIME) {
		return 0; //game is over, do nothing
		//Note: assume that all robots are trying to climb at the last 30 sec.
	}
	if (timeIn >= CLIMB_END_TIME) {
		//skip the action and only update the game score
		updateScore(CLIMB_END_TIME - m_timeInSec);
		return 0;
	}

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
	default:
		break; //no action, just time pass
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
			m_liftRedRobotIndex = INT_MAX;
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
			m_liftBlueRobotIndex = INT_MAX;
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