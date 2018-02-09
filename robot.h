#pragma once

#include <string.h>
#include "config.h"


typedef struct pendingActionType {
	actionTypeType actionType;
	int robotIndex;
	float projectedFinishTime;
	int projectedFinalScore;
	//Note: if it is the last pending action, the score is the score of game over time.
	int previousIndex;  //the previous action
}pendingActionType;

class robot
{
private:
	robotConfigurationType m_config;
	pendingActionType m_previousPlannedAction;
	robotStateType m_state;

public:
	robot();
	~robot();

	const robotConfigurationType *getConfiguration(void) const
	{
		return &m_config;
	}

	const robotStateType *getState(void) const
	{
		return &m_state;
	}
	void setConfiguration(const robotConfigurationType *pConfigIn);
	float getActionDelayInSec(actionTypeType actionIn, float currentTimeIn, bool firstActionAfterUpdateIn);

	void setPreviousPlannedAction(const pendingActionType *pPlannedActionIn);
	void resetPreviousPlannedAction(void);

	const robot & operator = (const robot &srcIn)
	{
		setConfiguration(srcIn.getConfiguration());
		memcpy(&m_state, srcIn.getState(), sizeof(m_state));
		return srcIn;
	}
};

