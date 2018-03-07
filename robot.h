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
	double startTime;
	double projectedFinishTime;
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
	double startTime;
	double projectedFinishTime;
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
	void setPosition(double xIn, double yIn, int objectIdIn);
	void setPlatformAndCube(platform *pPlatform, int cubeIdxIn);
	void dumpOneCube(void);

	double estimateActionDelayInSec(actionTypeType actionIn, double currentTimeIn, bool interruptFlagIn,
		  coordinateType lastActionStopPosIn, bool lastActionCubeNotUsedFlagIn, coordinateType *pEndPosOut) const;

	const robot & operator = (const robot &srcIn)
	{
		setConfiguration(srcIn.getConfiguration(), NULL);
		memcpy(&m_state, srcIn.getState(), sizeof(m_state));
		memcpy(&m_plannedAction, srcIn.getPlannedAction(), sizeof(m_plannedAction));
		m_allianceType = srcIn.getAlliance();

		return srcIn;
	}

	int takeAction(actionTypeType actionIn, double timeIn, int indexIn);
	int forceAction(const pendingActionType *pPlannedActionIn, coordinateType startPosIn, double timeIn, int indexIn);

	double getPlannedActionFinishTime(void)
	{
		return m_plannedAction.projectedFinishTime;
	}

	static bool isActionNeedCube(actionTypeType actionIn);
	static bool isAutonomousAction(actionTypeType actionIn);
	static bool isHumanPlayerAction(actionTypeType actionIn);

	actionResultType moveToNextTime(double timeIn);

protected:
	int combineTwoPathes(const robotPathType *pPath1In, const robotPathType *pPath2In, robotPathType *pPathOut) const;
	double calculateDelayOnPath(const coordinateType *pStartIn, const robotPathType *pPathIn) const;
	int findStopPosition(const coordinateType *pStartIn, const robotPathType *pPathIn, double stopDelayIn,
		coordinateType *pStopPositionOut, int *pCubeIndexOut, double *pTrnPointDelayChangeOut,
		bool *pGiveUpCubeFlagOut, bool *atMiddleOfLineFlagOut, bool *pJustDoneFlagOut) const;

	double getActionDelayInSecInternal(actionTypeType actionIn, double currentTimeIn, const rectangleObjectType *pStartPosIn, bool hasCubeFlagIn,
		bool interruptFlagIn, robotPathType *pPathOut) const;

	void updatePath(int stopIdxIn, int cubeIdxIn, bool middleOfLineFlagIn, double lineDelayChangeIn, robotPathType *pPathInOut) const;

	//TODO, apply motion profile with following functions, JWJW
	double getLineDelay(coordinateType startPoint, coordinateType endPoint, double maximumSpeedIn, double accelerationDistanceIn) const;
	double runFromePointToPoint(coordinateType startPoint, coordinateType endPoint, 
		double initialSpeedIn, double maximumSpeedIn, double accelerationDistanceIn,
		double durationIn, coordinateType *pStopPointOut, bool *pIsFinishedFlagOut) const;

};

