#pragma once

#include <string.h>
#include "config.h"


typedef struct robotStateType {
	//real state
	rectangleObjectType pos;
	int cubeIdx; //INT32_MAX mean no cube

	//pending action
	actionTypeType pendingAction;
	robotPathType pendingPath;
	int pendingPickUpCubeIdx; //the turn point index to pick up a cube
}robotStateType;

typedef struct searchActionType {
	actionTypeType actionType;
	float startTime;
	float projectedFinishTime;
	int projectedFinalScore;
	int actionIndex;
	int robotIndex;
	int previousIndex;
}searchActionType;


typedef struct pendingActionType {
	actionTypeType actionType;
	float startTime;
	float projectedFinishTime;
	int actionIndex;
	robotPathType path;
}pendingActionType;

class platform;

class robot
{
private:
	platform *m_pPlatform;
	robotConfigurationType m_config;
	robotStateType m_state;
	pendingActionType m_plannedAction;

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
	void setPosition(float xIn, float yIn, int objectIdIn);
	void setPlatformAndCube(platform *pPlatform, int cubeIdxIn);
	void dumpOneCube(void);

	float getActionDelayInSec(actionTypeType actionIn, float currentTimeIn, robotPathType *pPathOut) const;

	const robot & operator = (const robot &srcIn)
	{
		setConfiguration(srcIn.getConfiguration(), NULL);
		memcpy(&m_state, srcIn.getState(), sizeof(m_state));
		return srcIn;
	}

	int takeAction(actionTypeType actionIn, float timeIn, int indexIn);

	float getPlannedActionFinishTime(void)
	{
		return m_plannedAction.projectedFinishTime;
	}

	static bool isActionNeedCube(actionTypeType actionIn);

protected:
	int combineTwoPathes(const robotPathType *pPath1In, const robotPathType *pPath2In, robotPathType *pPathOut) const;
	float calculateDelayOnPath(const coordinateType *pStartIn, const robotPathType *pPathIn) const;
	int findStopPosition(const coordinateType *pStartIn, const robotPathType *pPathIn, float stopDelayIn, coordinateType *pStopPositionOut, bool *pHasCubFlagOut) const;
	float getActionDelayInSec(actionTypeType actionIn, float currentTimeIn, const rectangleObjectType *pStartPosIn, bool hasCubeFlagIn, robotPathType *pPathOut) const;
};

