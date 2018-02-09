
//#include "stdafx.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"

const float MINMUM_TIME_RESOLUTION = (float) 0.1; //second

robot::robot()
{
	memset(&m_config, 0, sizeof(m_config));
	resetPreviousPlannedAction();
}

robot::~robot()
{
}

void robot::resetPreviousPlannedAction(void)
{
	m_previousPlannedAction.actionType = RED_ACTION_NONE;
	m_previousPlannedAction.projectedFinishTime = CLIMB_END_TIME + 1;
}

void robot::setPreviousPlannedAction(const pendingActionType *pPlannedActionIn)
{
	memcpy(&m_previousPlannedAction, pPlannedActionIn, sizeof(m_previousPlannedAction));
}


void robot::setConfiguration(const robotConfigurationType *pConfigIn)
{
	memcpy(&m_config, pConfigIn, sizeof(m_config));
}

float robot::getActionDelayInSec(actionTypeType actionIn, float currentTimeIn, bool firstActionAfterUpdateIn)
{
	float randomFactor = m_config.randomDelayFactor * ((float)rand() / (float)RAND_MAX);
	float actionDelay;

	if ((actionIn == m_previousPlannedAction.actionType) && (firstActionAfterUpdateIn)) {
		//the robot can continue the previous action
		if ((m_previousPlannedAction.projectedFinishTime <= currentTimeIn) ||
		    (m_previousPlannedAction.projectedFinishTime == INT32_MAX)) {
			//task done already, the minimum delay is 1 sec.
			return 1;
		}
		else {
			return m_previousPlannedAction.projectedFinishTime - currentTimeIn;
		}
	}

	switch (actionIn) {
	case CUBE_RED_OFFENCE_SWITCH:
	case CUBE_BLUE_OFFENCE_SWITCH:
	case CUBE_RED_DEFENCE_SWITCH:
	case CUBE_BLUE_DEFENCE_SWITCH:
		actionDelay = m_config.boxToSwitchDelay + randomFactor;
		break;
	case CUBE_RED_SCALE:
	case CUBE_BLUE_SCALE:
		actionDelay = m_config.boxToScaleDelay + randomFactor;
		break;
	case CUBE_RED_FORCE_VAULT:
	case CUBE_RED_BOOST_VAULT:
	case CUBE_RED_LIFT_VAULT:
	case CUBE_BLUE_LIFT_VAULT:
	case CUBE_BLUE_FORCE_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
		actionDelay = m_config.boxToVaultDelay + randomFactor;
		break;
	case PUSH_RED_FORCE_BUTTON:
	case PUSH_RED_BOOST_BUTTON:
	case PUSH_RED_LIFT_BUTTON:
	case PUSH_BLUE_FORCE_BUTTON:
	case PUSH_BLUE_BOOST_BUTTON:
	case PUSH_BLUE_LIFT_BUTTON:
	case RED_ACTION_NONE:
	case BLUE_ACTION_NONE:
		actionDelay = m_config.pushButtonDelay + randomFactor;
		break;
	case LIFT_ONE_BLUE_ROBOT:
	case LIFT_ONE_RED_ROBOT:
		actionDelay = m_config.liftRobotDelay + randomFactor;
		break;
	default:
		//no robot action
		actionDelay = 0;
		break; 
	}

	//return delay must not be 0
	if (MINMUM_TIME_RESOLUTION > actionDelay) {
		actionDelay = MINMUM_TIME_RESOLUTION;
	}

	return actionDelay;
}