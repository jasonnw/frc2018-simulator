#pragma once

#include <string.h>
#include "config.h"


typedef struct robotStateType {
	//real state
	rectangleObjectType pos;
	int cubeIdx; //INT32_MAX mean no cube
}robotStateType;

typedef struct searchActionType {
	actionTypeType actionType;
	coordinateType actionDonePos;
	bool actionDoneWithCube;
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
	allianceType m_allianceType;

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

	coordinateType getPosition(void) const
	{
		return m_state.pos.center;
	}

	const pendingActionType *getPlannedAction(void) const
	{
		return &m_plannedAction;
	}

	bool hasCube(void) const
	{
		return m_state.cubeIdx == INVALID_IDX ? false : true;
	}

	void setAllianceType(allianceType allianceTypeIn)
	{
		m_allianceType = allianceTypeIn;
	}

	void setConfiguration(const robotConfigurationType *pConfigIn, platform *pPlatform);
	void setPosition(float xIn, float yIn, int objectIdIn);
	void setPlatformAndCube(platform *pPlatform, int cubeIdxIn);
	void dumpOneCube(void);
	void pickUpOneCube(int cubeIdxIn);

	float estimateActionDelayInSec(actionTypeType actionIn, float currentTimeIn, bool interruptFlagIn,
		  coordinateType lastActionStopPosIn, bool lastActionCubeNotUsedFlagIn, coordinateType *pEndPosOut) const;

	const robot & operator = (const robot &srcIn)
	{
		setConfiguration(srcIn.getConfiguration(), NULL);
		memcpy(&m_state, srcIn.getState(), sizeof(m_state));
		return srcIn;
	}

	void takeAction(actionTypeType actionIn, float timeIn, int indexIn);

	float getPlannedActionFinishTime(void)
	{
		return m_plannedAction.projectedFinishTime;
	}

	static bool isActionNeedCube(actionTypeType actionIn);

	bool moveToNextTime(float timeIn);

protected:
	int combineTwoPathes(const robotPathType *pPath1In, const robotPathType *pPath2In, robotPathType *pPathOut) const;
	float calculateDelayOnPath(const coordinateType *pStartIn, const robotPathType *pPathIn) const;
	int findStopPosition(const coordinateType *pStartIn, const robotPathType *pPathIn, float stopDelayIn,
		coordinateType *pStopPositionOut, int *pCubeIndexOutt, bool *pGiveUpCubeFlag) const;
	float getActionDelayInSecInternal(actionTypeType actionIn, float currentTimeIn, const rectangleObjectType *pStartPosIn, bool hasCubeFlagIn,
		bool interruptFlagIn, robotPathType *pPathOut) const;

};

