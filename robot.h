#pragma once

#include "config.h"
#include "platform.h"


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

public:
	robot();
	~robot();

	void setConfiguration(const robotConfigurationType *pConfigIn);
	float getActionDelayInSec(actionTypeType actionIn, float currentTimeIn, bool firstActionAfterUpdateIn);

	void setPreviousPlannedAction(const pendingActionType *pPlannedActionIn);
	void resetPreviousPlannedAction(void);
};

