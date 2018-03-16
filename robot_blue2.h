#pragma once

#include "robot.h"

class robotBlue2 : public robot
{
private:
	int m_idleCount;

public:
	robotBlue2()
	{
		m_isAiRobotFlag = false;
		m_allianceType = ALLIANCE_BLUE;
		m_robotIndex = 2;
		m_idleCount = 0;
	}

	virtual void getNextAction(platform *pPlatformInOut, searchActionType * pActionOut)
	{
		double currentTime = pPlatformInOut->getTime();
		const platformStateType *pPlatformState = pPlatformInOut->getState();
		const robotStateType *pRobotState = getState();
		const pendingActionType *pPlannedAction;
		bool robotHasCubeFlag = pRobotState->cubeIdx == INVALID_IDX ? false : true;

		initTaskToNoAction(pActionOut);
		if (pPlatformInOut->isRobotLifted(m_allianceType, m_robotIndex)) {
			return;
		}

		//first move is auto line 
		if (currentTime < 5) {
			//pass the auto line
			pActionOut->actionType = BLUE_ROBOT_GOTO_POS;
			pActionOut->actionDonePos = { 248, 342 };
			return;
		}

		//the first priority is lifting
		if (currentTime > COMPETITION_END_TIME) {
			pActionOut->actionType = LIFT_ONE_BLUE_ROBOT;
			//check if the action is feasible
			if (checkIfActionFeasible(
				pRobotState->pos.center,  //current root position
				robotHasCubeFlag,         //cube is already loaded flag
				ALLIANCE_BLUE,            //alliance name
				pPlatformInOut,           //platform object
				pActionOut)) {            //output action plan

				m_idleCount = 0; //planned a real action
				return;
			}
		}

		//second priority is the previous task
		pPlannedAction = getPlannedAction();
		if (pPlannedAction->actionType != INVALID_ACTION) {
			//robot still busy, don't create a new task
			return;
		}


		//third priority, this robot is assigned to defense red switch
		if (pPlatformState->switchRed_BlueBlockCount < pPlatformState->switchRed_RedBlockCount + 2 + m_idleCount/4) {
			//we want to more than necessary blocks on our side
			pActionOut->actionType = CUBE_BLUE_DEFENSE_SWITCH;
			//check if the action is feasible
			if (checkIfActionFeasible(
				pRobotState->pos.center,  //current root position
				robotHasCubeFlag,         //cube is already loaded flag
				ALLIANCE_BLUE,            //alliance name
				pPlatformInOut,           //platform object
				pActionOut)) {            //output action plan

				m_idleCount = 0; //planned a real action
				return;
			}
		}

		//fourth priority action, boost vault
		if (pPlatformState->boostBlueBlockCount < 3) {
			pActionOut->actionType = CUBE_BLUE_BOOST_VAULT;
			//check if the action is feasible
			if (checkIfActionFeasible(
				pRobotState->pos.center,  //current root position
				robotHasCubeFlag,         //cube is already loaded flag
				ALLIANCE_BLUE,            //alliance name
				pPlatformInOut,           //platform object
				pActionOut)) {            //output action plan

				m_idleCount = 0; //planned a real action
				return;
			}
		}
		else if (pPlatformState->blueBoostButton == BUTTON_NOT_PUSH) {
			//push force vault button
			pActionOut->actionType = PUSH_BLUE_BOOST_BUTTON;
			return; //push button always success, no checking for this action
		}

		//no real actions to do
		m_idleCount++;

		//fifth priority action, block opponent robots
		//at the center of left switch zone
		pActionOut->actionType = BLUE_ROBOT_GOTO_POS;
		if (pRobotState->pos.center.y <= 200) {
			pActionOut->actionDonePos = { 60, 250 };
		}
		else {
			pActionOut->actionDonePos = { 60, 50 };
		}

		return;
		//Note: 
		//  if BLUE_ROBOT_GOTO_POS is not feasible, the command will be rejected,
		//  and the robot will be idle.
	}

};


