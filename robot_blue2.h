#pragma once

#include "robot.h"

class robotBlue2 : public robot
{
public:
	robotBlue2()
	{
		m_isAiRobotFlag = false;
		m_allianceType = ALLIANCE_BLUE;
		m_robotIndex = 2;
	}

	virtual void getNextAction(platform *pPlatformInOut, searchActionType * pActionOut) const
	{
		double currentTime = pPlatformInOut->getTime();
		const platformStateType *pPlatformState = pPlatformInOut->getState();
		const robotStateType *pRobotState = getState();
		const pendingActionType *pPlannedAction;
		bool robotHasCubeFlag = pRobotState->cubeIdx == INVALID_IDX ? false : true;

		initTaskToNoAction(pActionOut);

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
		if (pPlatformState->switchRed_BlueBlockCount < pPlatformState->switchRed_RedBlockCount + 2) {
			//we want to keep our side two blocks more than the opponent
			pActionOut->actionType = CUBE_BLUE_DEFENSE_SWITCH;
			//check if the action is feasible
			if (checkIfActionFeasible(
				pRobotState->pos.center,  //current root position
				robotHasCubeFlag,         //cube is already loaded flag
				ALLIANCE_BLUE,            //alliance name
				pPlatformInOut,           //platform object
				pActionOut)) {            //output action plan
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
				return;
			}
		}
		else if (pPlatformState->blueBoostButton == BUTTON_NOT_PUSH) {
			//push force vault button
			pActionOut->actionType = PUSH_BLUE_BOOST_BUTTON;
			return; //push button always success, no checking for this action
		}

		//fifth priority action, block opponent robots
		//at the center of left switch zone
		pActionOut->actionType = BLUE_ROBOT_GOTO_POS;
		if (pRobotState->pos.center.y <= 180) {
			pActionOut->actionDonePos = { 60, 250 };
		}
		else if(pRobotState->pos.center.y >= 250) {
			pActionOut->actionDonePos = { 60, 180 };
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


