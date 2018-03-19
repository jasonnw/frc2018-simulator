#pragma once

#include "robot.h"

class robotBlue1 : public robot
{
private:
	int m_idleCount;
public:
	robotBlue1()
	{
		m_isAiRobotFlag = false;
		m_allianceType = ALLIANCE_BLUE;
		m_robotIndex = 1;
		m_idleCount = 0;
	}

	virtual void getNextAction(platform *pPlatformInOut, searchActionType * pActionOut)
	{
		const pendingActionType *pPlannedAction;
		double currentTime = pPlatformInOut->getTime();
		const platformStateType *pPlatformState = pPlatformInOut->getState();
		coordinateType robotPosition = pPlatformInOut->getRobotPos(ALLIANCE_BLUE, m_robotIndex);
		coordinateType rampRobotDestination = pPlatformInOut->getBlueLiftZonePosition();
		bool cubesAvailableFlag = pPlatformInOut->hasMoreBlueCubesFlag();

		//initialize the default output as NO ACTION
		initTaskToNoAction(pActionOut);

		if ((currentTime > CLIMB_START_TIME - 3) || (!cubesAvailableFlag)) {
			if ((robotPosition.y == rampRobotDestination.y) &&
				(robotPosition.x == rampRobotDestination.x)) {
				//stay at this position and wait for other robots to climb
				pActionOut->actionType = INVALID_ACTION;
				return;
			}
			else {
				pActionOut->actionType = BLUE_ROBOT_GOTO_POS;
				pActionOut->actionDonePos = rampRobotDestination;
				//check if the action is feasible
				if (checkIfActionFeasible(
					ALLIANCE_BLUE,            //alliance name
					pPlatformInOut,           //platform object
					pActionOut)) {            //output action plan

					m_idleCount = 0;
					return;
				}
			}
		}

		//the second priority is the previous task
		initTaskToNoAction(pActionOut);
		pPlannedAction = getPlannedAction();
		if (pPlannedAction->actionType != INVALID_ACTION) {
			//robot still busy, don't create a new task
			return;
		}

		//third priority, this robot is assigned to control blue switch
		if ((pPlatformState->switchBlue_BlueBlockCount < pPlatformState->switchBlue_RedBlockCount + 2 + m_idleCount/8) &&
		    (cubesAvailableFlag)) {
			//we want to keep our side two blocks more than the opponent

			pActionOut->actionType = CUBE_BLUE_OFFENSE_SWITCH;
			//check if the action is feasible
			if (checkIfActionFeasible(
						ALLIANCE_BLUE,            //alliance name
						pPlatformInOut,           //platform object
						pActionOut)) {            //output action plan

				m_idleCount = 0;
				return;
			}
		}

		//fourth priority action, lift vault
		initTaskToNoAction(pActionOut);
		if ((pPlatformState->liftBlueBlockCount < 3) && (cubesAvailableFlag)) {
			pActionOut->actionType = CUBE_BLUE_LIFT_VAULT;
			//check if the action is feasible
			if (checkIfActionFeasible(
						ALLIANCE_BLUE,            //alliance name
						pPlatformInOut,           //platform object
						pActionOut)) {            //output action plan

				m_idleCount = 0;
				return;
			}
		}
		//else, lift button is auto, no need to push

		m_idleCount++;

		//stay close to offense switch for quick response
		initTaskToNoAction(pActionOut);
		pActionOut->actionType = BLUE_ROBOT_GOTO_POS;
		pActionOut->actionDonePos = coordinateType(600, 250 );

		return;
		//Note: 
		//  if BLUE_ROBOT_GOTO_POS is not feasible, the command will be rejected,
		//  and the robot will be idle.
	}

};
