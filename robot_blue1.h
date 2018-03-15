#pragma once

#include "robot.h"

class robotBlue1 : public robot
{
public:
	robotBlue1()
	{
		m_isAiRobotFlag = false;
		m_allianceType = ALLIANCE_BLUE;
		m_robotIndex = 1;
	}

	virtual void getNextAction(platform *pPlatformInOut, searchActionType * pActionOut) const
	{
		double currentTime = m_pPlatform->getTime();

		const platformStateType *pPlatformState = m_pPlatform->getState();
		const robotStateType *pRobotState = getState();
		const pendingActionType *pPlannedAction;
		bool robotHasCubeFlag = pRobotState->cubeIdx == INVALID_IDX ? false : true;

		initTaskToNoAction(pActionOut);

		//the first priority is lifting other robots
		if (currentTime > COMPETITION_END_TIME) {
			pActionOut->actionType = BLUE_ROBOT_GOTO_POS;
			pActionOut->actionDonePos = { 366.25, 180 };
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

		//the second priority is the previous task
		pPlannedAction = getPlannedAction();
		if (pPlannedAction->actionType != INVALID_ACTION) {
			//robot still busy, don't create a new task
			return;
		}

		//third priority, this robot is assigned to control blue switch
		if (pPlatformState->switchBlue_BlueBlockCount < pPlatformState->switchBlue_RedBlockCount + 2) {
			//we want to keep our side two blocks more than the opponent

			pActionOut->actionType = CUBE_BLUE_OFFENSE_SWITCH;
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

		//fourth priority action, lift vault
		if (pPlatformState->liftBlueBlockCount < 3) {
			pActionOut->actionType = CUBE_BLUE_LIFT_VAULT;
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
		//else, lift button is auto, no need to push

		//stay close to offense switch for quick response
		pActionOut->actionType = BLUE_ROBOT_GOTO_POS;
		pActionOut->actionDonePos = { 600, 50 };

		return;
		//Note: 
		//  if BLUE_ROBOT_GOTO_POS is not feasible, the command will be rejected,
		//  and the robot will be idle.
	}

};
