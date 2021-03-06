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
	double taskChainFinishTime;
	int projectedFinalScore;
	int projectedFinalRank;
	int actionIndex;
	int robotIndex;
	int previousIndex;

	searchActionType()
	{
		memset(this, 0, sizeof(struct searchActionType));

	}

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
	double pickUpCubeTime;
	int actionIndex;
	robotPathType path;

public:
	pendingActionType()
	{
		memset(this, 0, sizeof(struct pendingActionType));

	}

}pendingActionType;

class platform;

class robot
{
private:
	robotConfigurationType m_config;
	robotStateType m_state;
	pendingActionType m_plannedAction;

protected:
	platform *m_pPlatform;
	bool m_isAiRobotFlag;  //robot is controlled by AI
	allianceType m_allianceType;
	int m_robotIndex;

public:
	robot();
	virtual ~robot();

	virtual void getNextAction(platform *pPlatformInOut, searchActionType * pActionOut)
	{
		pActionOut->actionType = INVALID_ACTION;
	}

	bool getAiControlledFlag(void) const
	{
		return m_isAiRobotFlag;
	}

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
	const int getCubeIdx(void) const
	{
		return m_state.cubeIdx;
	}
	const int getRobotIndex(void) const
	{
		return m_robotIndex;
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
			m_state.pos.color = cv::Scalar(0, 0, 180);
		}
		else {
			m_state.pos.color = cv::Scalar(180, 0, 0);
		}
	}

	void setRobotIndex(int indexIn)
	{
		m_robotIndex = indexIn;
	}

	void setConfiguration(const robotConfigurationType *pConfigIn, platform *pPlatform, int indexIn);
	void setPosition(double xIn, double yIn, int objectIdIn);
	void setPlatformAndCube(platform *pPlatform, int cubeIdxIn);
	void dumpOneCube(void);

	double estimateActionDelayInSec(actionTypeType actionIn, double currentTimeIn, bool interruptFlagIn,
		  coordinateType lastActionStopPosIn, bool lastActionCubeNotUsedFlagIn,
		  coordinateType *pEndPosOut, bool *pDontInterruptFlagOut) const;

	const robot & operator = (const robot &srcIn)
	{
		setConfiguration(srcIn.getConfiguration(), NULL, srcIn.getRobotIndex());
		memcpy(&m_state, srcIn.getState(), sizeof(m_state));
		memcpy(&m_plannedAction, srcIn.getPlannedAction(), sizeof(m_plannedAction));
		m_allianceType = srcIn.getAlliance();

		return srcIn;
	}

	int takeAction(actionTypeType actionIn, coordinateType actionDonePosIn, double timeIn, int indexIn);
	int forceAction(const pendingActionType *pPlannedActionIn, coordinateType startPosIn, double timeIn, int indexIn);

	double getPlannedActionFinishTime(void)
	{
		return m_plannedAction.projectedFinishTime;
	}
	double getNextStopTime(void)
	{
		if ((m_plannedAction.actionType != INVALID_ACTION) &&
			(m_plannedAction.actionType != RED_ACTION_NONE) &&
			(m_plannedAction.actionType != BLUE_ACTION_NONE)) {
			if (m_plannedAction.path.pickUpCubeIndex != INVALID_IDX) {
				if (m_plannedAction.pickUpCubeTime >= m_plannedAction.projectedFinishTime) {
					printf("ERROR, pick up cube too early\n");
				}
				return m_plannedAction.pickUpCubeTime;
			}
			else {
				return m_plannedAction.projectedFinishTime;
			}
		}
		else {
			return CLIMB_END_TIME;
		}
	}

	static bool isActionNeedCube(actionTypeType actionIn);
	static bool isAutonomousAction(actionTypeType actionIn);
	static bool isHumanPlayerAction(actionTypeType actionIn);

	actionResultType moveToNextTime(double timeIn);

	bool checkIfActionFeasible(allianceType allianceIn,	platform *pPlatformInOut, searchActionType * pActionInOut) const;

protected:
	int combineTwoPathes(const robotPathType *pPath1In, const robotPathType *pPath2In, robotPathType *pPathOut) const;
	double calculateDelayOnPath(const coordinateType *pStartIn, const robotPathType *pPathIn) const;
	int findStopPosition(const coordinateType *pStartIn, const robotPathType *pPathIn, double stopDelayIn,
		coordinateType *pStopPositionOut, int *pCubeIndexOut, double *pTrnPointDelayChangeOut,
		bool *pGiveUpCubeFlagOut, bool *atMiddleOfLineFlagOut, bool *pJustDoneFlagOut) const;

	double getActionDelayInSecInternal(actionTypeType actionIn, coordinateType actionDonePosIn, double currentTimeIn, const rectangleObjectType *pStartPosIn,
		bool hasCubeFlagIn,	bool interruptFlagIn, robotPathType *pPathOut, double *pPickUpCubeDelayOut) const;

	void updatePath(int stopIdxIn, int cubeIdxIn, bool middleOfLineFlagIn, double lineDelayChangeIn, robotPathType *pPathInOut) const;

	//TODO, apply motion profile with following functions, JWJW
	double getLineDelay(coordinateType startPoint, coordinateType endPoint, double maximumSpeedIn, double accelerationDistanceIn) const;
	double runFromePointToPoint(coordinateType startPoint, coordinateType endPoint, 
		double initialSpeedIn, double maximumSpeedIn, double accelerationDistanceIn,
		double durationIn, coordinateType *pStopPointOut, bool *pIsFinishedFlagOut) const;

	//init action plan data structure
	void initTaskToNoAction(searchActionType * pActionOut) const;
};

