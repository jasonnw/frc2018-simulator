#pragma once

#include "robot.h"

class robotBlue0: public robot
{
private:
	int m_idleCount;

public:
	robotBlue0()
	{
		m_isAiRobotFlag = false;
		m_allianceType = ALLIANCE_BLUE;
		m_robotIndex = 0;
		m_idleCount = 0;
	}

	virtual void getNextAction(platform *pPlatformInOut, searchActionType * pActionOut)
	{
		double currentTime = m_pPlatform->getTime();
		const platformStateType *pPlatformState = m_pPlatform->getState();
		const robotStateType *pRobotState = getState();
		const pendingActionType *pPlannedAction;
		bool robotHasCubeFlag = pRobotState->cubeIdx == INVALID_IDX ? false : true;

		initTaskToNoAction(pActionOut);
		if (pPlatformInOut->isRobotLifted(m_allianceType, m_robotIndex)) {
			return;
		}

		//the first priority is lifting
		if (currentTime > COMPETITION_END_TIME + 5) {
			pActionOut->actionType = LIFT_ONE_BLUE_ROBOT;
			//check if the action is feasible
			if (checkIfActionFeasible(
						pRobotState->pos.center,  //current root position
						robotHasCubeFlag,         //cube is already loaded flag
						ALLIANCE_BLUE,            //alliance name
						pPlatformInOut,           //platform object
						pActionOut)) {            //output action plan

				m_idleCount = 0;
				return;
			}
		}

		//second priority is the previous task
		pPlannedAction = getPlannedAction();
		if (pPlannedAction->actionType != INVALID_ACTION) {
			//robot still busy, don't create a new task
			return;
		}

		//third priority, this robot is assigned to the scale
		if (pPlatformState->scaleBlueBlockCount < pPlatformState->scaleRedBlockCount + 2 + m_idleCount/4) {
			//we want to keep our side two blocks more than the opponent

			pActionOut->actionType = CUBE_BLUE_SCALE;
			//check if the action is feasible
			if (checkIfActionFeasible(
						pRobotState->pos.center,  //current root position
						robotHasCubeFlag,         //cube is already loaded flag
						ALLIANCE_BLUE,            //alliance name
						pPlatformInOut,           //platform object
						pActionOut)) {            //output action plan

				m_idleCount = 0;
				return;
			}
		}

		//fourth priority action, force vault
		if (pPlatformState->forceBlueBlockCount < 3) {
			pActionOut->actionType = CUBE_BLUE_FORCE_VAULT;
			//check if the action is feasible
			if (checkIfActionFeasible(
						pRobotState->pos.center,  //current root position
						robotHasCubeFlag,         //cube is already loaded flag
						ALLIANCE_BLUE,            //alliance name
						pPlatformInOut,           //platform object
						pActionOut)) {            //output action plan

				m_idleCount = 0;
				return;
			}
		}
		else if (pPlatformState->blueForceButton == BUTTON_NOT_PUSH) {
			//push force vault button
			pActionOut->actionType = PUSH_BLUE_FORCE_BUTTON;
			return; //push button always success, no checking for this action
		}

		m_idleCount++;

		//fifth priority action, block opponent robots
		//at the top of left switch zone
		pActionOut->actionType = BLUE_ROBOT_GOTO_POS;
		if (pRobotState->pos.center.y <= 180) {
			pActionOut->actionDonePos = { 60, 280 };
		}
		else if (pRobotState->pos.center.y >= 280) {
			pActionOut->actionDonePos = { 60, 50 };
		}
		else {
			pActionOut->actionDonePos = { 60, 180 };
		}

		return;
		//Note: 
		//  if BLUE_ROBOT_GOTO_POS is not feasible, the command will be rejected,
		//  and the robot will be idle.
	}
};
