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
	~robotBlue2()
	{
	}


	virtual void getNextAction(platform *pPlatformInOut, searchActionType * pActionOut)
	{
		const pendingActionType *pPlannedAction;
		double currentTime = pPlatformInOut->getTime();
		const platformStateType *pPlatformState = pPlatformInOut->getState();
		coordinateType robotPosition = pPlatformInOut->getRobotPos(ALLIANCE_BLUE, m_robotIndex);
		coordinateType rampRobotDestination = pPlatformInOut->getBlueLiftZonePosition();
		coordinateType rampRobotCurrentPosition = pPlatformInOut->getRobotPos(m_allianceType, 1);
		bool cubesAvailableFlag = pPlatformInOut->hasMoreBlueCubesFlag();

		initTaskToNoAction(pActionOut);
		if (pPlatformInOut->isRobotLifted(m_allianceType, m_robotIndex)) {
			return;
		}

		//first move is auto line 
		if (currentTime < 5) {
			//pass the auto line
			pActionOut->actionType = BLUE_ROBOT_GOTO_POS;
			pActionOut->actionDonePos = coordinateType(248, 342);
			return;
		}

		//the first priority is lifting
		initTaskToNoAction(pActionOut);
		if ((currentTime > CLIMB_START_TIME) || (!cubesAvailableFlag)) {
			if ((rampRobotDestination.x == rampRobotCurrentPosition.x) &&
				(rampRobotDestination.y == rampRobotCurrentPosition.y)) {
				//ready for lifting

				pActionOut->actionType = LIFT_ONE_BLUE_ROBOT;
				//check if the action is feasible
				if (checkIfActionFeasible(
					ALLIANCE_BLUE,            //alliance name
					pPlatformInOut,           //platform object
					pActionOut)) {            //output action plan

					m_idleCount = 0; //planned a real action
					return;
				}
			}
		}

		//second priority is the previous task
		initTaskToNoAction(pActionOut);
		pPlannedAction = getPlannedAction();
		if (pPlannedAction->actionType != INVALID_ACTION) {
			//robot still busy, don't create a new task
			return;
		}


		//third priority, this robot is assigned to defense red switch
		if ((pPlatformState->switchRed_BlueBlockCount < pPlatformState->switchRed_RedBlockCount + 2) &&
		    (cubesAvailableFlag)) {
			//we want to more than necessary blocks on our side
			pActionOut->actionType = CUBE_BLUE_DEFENSE_SWITCH;
			//check if the action is feasible
			if (checkIfActionFeasible(
				ALLIANCE_BLUE,            //alliance name
				pPlatformInOut,           //platform object
				pActionOut)) {            //output action plan

				m_idleCount = 0; //planned a real action
				return;
			}
		}

		//fourth priority action, boost vault
		initTaskToNoAction(pActionOut);
		if ((pPlatformState->boostBlueBlockCount < 3) && (cubesAvailableFlag)) {
			pActionOut->actionType = CUBE_BLUE_BOOST_VAULT;
			//check if the action is feasible
			if (checkIfActionFeasible(
				ALLIANCE_BLUE,            //alliance name
				pPlatformInOut,           //platform object
				pActionOut)) {            //output action plan

				m_idleCount = 0; //planned a real action
				return;
			}
		}
		else if ((pPlatformState->blueBoostButton == BUTTON_NOT_PUSH) &&
		         ((pPlatformState->boostBlueBlockCount > 1))) {
			//push force vault button
			pActionOut->actionType = PUSH_BLUE_BOOST_BUTTON;
			return; //push button always success, no checking for this action
		}

		//no real actions to do
		m_idleCount++;

		//fifth priority action, block opponent robots
		//at the center of left switch zone
		initTaskToNoAction(pActionOut);
		pActionOut->actionType = BLUE_ROBOT_GOTO_POS;
		if (robotPosition.x <= 200) {
			pActionOut->actionDonePos = coordinateType( 400, 320 );
		}
		else {
			pActionOut->actionDonePos = coordinateType(100, 320 );
		}

		return;
		//Note: 
		//  if BLUE_ROBOT_GOTO_POS is not feasible, the command will be rejected,
		//  and the robot will be idle.
	}

};


