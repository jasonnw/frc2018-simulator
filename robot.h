#pragma once

#include <string.h>
#include "config.h"


typedef struct robotStateType {
	//real state
	rectangleObjectType pos;
	cubeStateType *pCube; //NULL mean no cube

	//pending action
	actionTypeType pendingAction;
	robotPathType pendingPath;
	int pendingPickUpCubeIdx; //the turn point index to pick up a cube
}robotStateType;

typedef struct pendingActionType {
	actionTypeType actionType;
	int robotIndex;
	float projectedFinishTime;
	int projectedFinalScore;
	//Note: if it is the last pending action, the score is the score of game over time.
	int previousIndex;  //the previous action
}pendingActionType;

class platform;

class robot
{
private:
	platform *m_pPlatform;
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
	void setConfiguration(const robotConfigurationType *pConfigIn, platform *pPlatform);
	float getActionDelayInSec(actionTypeType actionIn, float currentTimeIn, bool firstActionAfterUpdateIn);

	void setPreviousPlannedAction(const pendingActionType *pPlannedActionIn);
	void resetPreviousPlannedAction(void);

	const robot & operator = (const robot &srcIn)
	{
		setConfiguration(srcIn.getConfiguration(), NULL);
		memcpy(&m_state, srcIn.getState(), sizeof(m_state));
		return srcIn;
	}

protected:
	int combineTwoPathes(const robotPathType *pPath2CubeIn, const robotPathType *pPath2DestinationIn, robotPathType *pPathOut, int *pPickUpIdxOut);
	float calculateDelayOnPath(const coordinateType *pStartIn, const robotPathType *pPathIn);
	coordinateType findStopPosition(const coordinateType *pStartIn, const robotPathType *pPathIn, int pickUpIndexIn, float stopDelayIn, bool *pHasCubFlagOut);
};

