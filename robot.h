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

typedef enum actionResultType {
	ACTION_DONE,
	ACTION_TIME_OUT, //action will not finish before game over
	ACTION_FAILED,   //implementation error
	ACTION_START_ERROR, //start time inverse error
	ACTION_IN_PROGRESS
}actionResultType;



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

	allianceType getAlliance(void) const
	{
		return m_allianceType;
	}
	const robotConfigurationType *getConfiguration(void) const
	{
		return &m_config;
	}

	const robotStateType *getState(void) const
	{
		return &m_state;
	}

	const rectangleObjectType *getPosition(void) const
	{
		return &m_state.pos;
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
		if (m_allianceType == ALLIANCE_RED) {
			m_state.pos.color = { 0, 0, 180 };
		}
		else {
			m_state.pos.color = { 180, 0, 0 };
		}
	}

	void setConfiguration(const robotConfigurationType *pConfigIn, platform *pPlatform);
	void setPosition(float xIn, float yIn, int objectIdIn);
	void setPlatformAndCube(platform *pPlatform, int cubeIdxIn);
	void dumpOneCube(void);

	float estimateActionDelayInSec(actionTypeType actionIn, float currentTimeIn, bool interruptFlagIn,
		  coordinateType lastActionStopPosIn, bool lastActionCubeNotUsedFlagIn, coordinateType *pEndPosOut) const;

	const robot & operator = (const robot &srcIn)
	{
		setConfiguration(srcIn.getConfiguration(), NULL);
		memcpy(&m_state, srcIn.getState(), sizeof(m_state));
		memcpy(&m_plannedAction, srcIn.getPlannedAction(), sizeof(m_plannedAction));
		m_allianceType = srcIn.getAlliance();

		return srcIn;
	}

	int takeAction(actionTypeType actionIn, float timeIn, int indexIn);
	int forceAction(const pendingActionType *pPlannedActionIn, coordinateType startPosIn, float timeIn, int indexIn);

	float getPlannedActionFinishTime(void)
	{
		return m_plannedAction.projectedFinishTime;
	}

	static bool isActionNeedCube(actionTypeType actionIn);
	static bool isAutonomousAction(actionTypeType actionIn);
	static bool isHumanPlayerAction(actionTypeType actionIn);

	actionResultType moveToNextTime(float timeIn);

protected:
	int combineTwoPathes(const robotPathType *pPath1In, const robotPathType *pPath2In, robotPathType *pPathOut) const;
	float calculateDelayOnPath(const coordinateType *pStartIn, const robotPathType *pPathIn) const;
	int findStopPosition(const coordinateType *pStartIn, const robotPathType *pPathIn, float stopDelayIn,
		coordinateType *pStopPositionOut, int *pCubeIndexOut, float *pTrnPointDelayChangeOut,
		bool *pGiveUpCubeFlagOut, bool *atMiddleOfLineFlagOut, bool *pJustDoneFlagOut) const;

	float getActionDelayInSecInternal(actionTypeType actionIn, float currentTimeIn, const rectangleObjectType *pStartPosIn, bool hasCubeFlagIn,
		bool interruptFlagIn, robotPathType *pPathOut) const;


	//TODO, apply motion profile with following functions, JWJW
	float getLineDelay(coordinateType startPoint, coordinateType endPoint, float maximumSpeedIn, float accelerationDistanceIn) const;
	float runFromePointToPoint(coordinateType startPoint, coordinateType endPoint, 
		float initialSpeedIn, float maximumSpeedIn, float accelerationDistanceIn,
		float durationIn, coordinateType *pStopPointOut, bool *pIsFinishedFlagOut) const;

};

