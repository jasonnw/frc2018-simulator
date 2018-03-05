#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "robot.h"

typedef enum structuresType {
	RED_SWITCH_ZONE,
	BLUE_SWITCH_ZONE,
	SCALE_ZONE,
	NUM_STILL_STRUCTURE
}structuresType;


typedef struct platformLayoutType {

	//still structures for collision detection
	float southWall;
	float northWall;
	float eastWall;
	float westWall;
	float redAutoLine;
	float blueAutoLine;
	rectangleObjectType structures[NUM_STILL_STRUCTURE];

	//possible robot destination
	rectangleObjectType redPowerCubeZone;
	rectangleObjectType bluePowerCubeZone;
	rectangleObjectType redPlatformZone;
	rectangleObjectType bluePlatformZone;
	rectangleObjectType redSwitchNorthPlate;
	rectangleObjectType redSwitchSouthPlate;
	rectangleObjectType blueSwitchNorthPlate;
	rectangleObjectType blueSwitchSouthPlate;

	rectangleObjectType scaleNorthPlate;
	rectangleObjectType scaleSouthPlate;

	rectangleObjectType redExchangeZone;
	rectangleObjectType blueExchangeZone;

	rectangleObjectType redLiftZone;
	rectangleObjectType blueLiftZone;
}platformLayoutType;

//the indexes to all cubes in the cube array
const int MAX_CUBES = 2000;
typedef enum cubeIndexType {
	CUBE_ON_RED_ROBOTS = 0,
	CUBE_ON_BLUE_ROBOTS = 0,
	CUBE_BY_RED_SWITCH = 6,
	CUBE_BY_BLUE_SWITCH = 12,
	CUBE_BY_RED_POWER_ZONE = 18,
	CUBE_BY_BLUE_POWER_ZONE = 28,
	CUBE_BY_RED_EXCHANGE_ZONE = 38,
	CUBE_BY_BLUE_EXCHANGE_ZONE = 1064,
	CUBE_LAST = MAX_CUBES - 1
}cubeIndexType;

typedef struct cubeSearchRangeType
{
	int startIdx;
	int endIdx;
	bool allCubeSameFlag;
}cubeSearchRangeType;

const int MAXIMUM_STACK_SIZE = 64;

template <class T> class dataStack {
private:
	T m_stackBuffer[MAXIMUM_STACK_SIZE];
	int m_pushPopIdx;  //the next buffer index 

public:
	dataStack(void)
	{
		reset(void);
	}
	~dataStack(void)
	{
	}

	bool isStackEmpty(void)
	{
		if (m_pushPopIdx == 0) {
			return true;
		}
		else {
			return false;
		}
	}

	bool isStackFull(void)
	{
		if (m_pushPopIdx >= MAXIMUM_STACK_SIZE) {
			return true;
		}
		else {
			return false;
		}
	}


	void push(const T *pMessageIn)
	{
		if (m_pushPopIdx >= MAXIMUM_STACK_SIZE) {
			return;
		}
		//else
		memcpy(&m_stackBuffer[m_pushPopIdx], pMessageIn, sizeof(T));
		m_pushPopIdx++;
		return;
	}

	void pop(T* pMessageOut)
	{
		if (m_pushPopIdx == 0) {
			return;
		}
		//else
		memcpy(pMessageOut, &m_stackBuffer[m_pushPopIdx-1], sizeof(T));
		m_pushPopIdx--;
		return;
	}

	void reset(void)
	{
		m_pushPopIdx = 0;
	}
};



class platform
{
private:
	float m_timeInSec; 
	int m_redScore;
	int m_blueScore;
	int m_liftRedRobotIndex;
	int m_liftBlueRobotIndex;
	int m_debugCounter;
	FILE *m_pLogFIle;

protected:
	platformStateType m_state;
	platformLayoutType m_platformStructure;
	robot m_redRobots[NUMBER_OF_ROBOTS];
	robot m_blueRobots[NUMBER_OF_ROBOTS];


	cubeStateType m_cubes[MAX_CUBES];
	//6 cubes by each switch
	//10 cubes at each power cube zone
	//unlimited cubes at exchange zone
	//constructor will put cube to the start position;

public:
	platform();
	~platform();

	const platformStateType * getState(void) const { return &m_state; }
	float getTime(void) const { return m_timeInSec; }
	int getRedScore(void) const { return m_redScore; }
	int getBlueScore(void) const { return m_blueScore; }
	const cubeStateType *getCubes(void) const { return m_cubes; }
	const robot * getRedRobots(void) const { return m_redRobots; }
	const robot * getBlueRobots(void) const { return m_blueRobots; }

	float getScaleX(void) { return m_platformStructure.scaleNorthPlate.center.x; }
	float getScaleNorthY(void) { 
		return m_platformStructure.scaleNorthPlate.center.y + m_platformStructure.scaleNorthPlate.sizeY/2;
	}
	float getScaleSouthY(void) {
		return m_platformStructure.scaleSouthPlate.center.y - m_platformStructure.scaleSouthPlate.sizeY / 2;
	}

	float getRedSwitchX(void) {
		return m_platformStructure.redSwitchNorthPlate.center.x;
	}
	float getBlueSwitchX(void) {
		return m_platformStructure.blueSwitchNorthPlate.center.x;
	}
	float getRedSwitchNorthY(void) {
		return m_platformStructure.redSwitchNorthPlate.center.y + m_platformStructure.redSwitchNorthPlate.sizeY/2;
	}
	float getRedSwitchSouthY(void) {
		return m_platformStructure.redSwitchSouthPlate.center.y - m_platformStructure.redSwitchSouthPlate.sizeY / 2;
	}

	float getBlueSwitchNorthY(void) {
		return m_platformStructure.blueSwitchNorthPlate.center.y + m_platformStructure.blueSwitchNorthPlate.sizeY / 2;
	}
	float getBlueSwitchSouthY(void) {
		return m_platformStructure.blueSwitchSouthPlate.center.y - m_platformStructure.blueSwitchSouthPlate.sizeY / 2;
	}

	float getRedExchangeZoneX(void) {
		return m_platformStructure.redExchangeZone.center.x + m_platformStructure.redExchangeZone.sizeX/2;
		//Note red exchange zone is on the left side
	}
	float getRedExchangeZoneY(void) {
		return m_platformStructure.redExchangeZone.center.y;
	}
	float getBlueExchangeZoneX(void) {
		return m_platformStructure.blueExchangeZone.center.x - m_platformStructure.blueExchangeZone.sizeX / 2;
		//NOte: blue exchange zone is on the right side
	}
	float getBlueExchangeZoneY(void) {
		return m_platformStructure.blueExchangeZone.center.y;
	}

	coordinateType getRedLiftZonePosition(void) {
		return m_platformStructure.redLiftZone.center;
	}

	coordinateType getBlueLiftZonePosition(void) {
		return m_platformStructure.blueLiftZone.center;
	}

	int getRedLiftRobotIndex(void) const { return m_liftRedRobotIndex; }
	int getBlueLiftRobotIndex(void) const { return m_liftBlueRobotIndex; }

	const pendingActionType *getRobotAction(allianceType allianceIn, int robotIdxIn) const;


	void setState(const platformStateType *pStateIn) { memcpy(&m_state, pStateIn,  sizeof(m_state)); }
	void setTime(float timeIn) { m_timeInSec = timeIn; }
	void setRedScore(int redScoreIn) { m_redScore = redScoreIn; }
	void setBlueScore(int blueScoreIn) { m_blueScore = blueScoreIn; }
	void setLogFile(FILE *pFileIn) { m_pLogFIle = pFileIn; }
	void setCubes(const cubeStateType *pCubesIn) {
		memcpy(m_cubes, pCubesIn, sizeof(cubeStateType)*MAX_CUBES);
	}

	void setRedLiftRobotIndex(int idxIn) { m_liftRedRobotIndex = idxIn; }
	void setBlueLiftRobotIndex(int idxIn) { m_liftBlueRobotIndex = idxIn; }

	void setRedRobots(const robot *pRobotsIn) {
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
			m_redRobots[i] = pRobotsIn[i];
		}
	}
	void setBlueRobots(const robot *pRobotsIn) {
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
			m_blueRobots[i] = pRobotsIn[i];
		}
	}

	const platform & operator = (const platform &srcIn)
	{
		setState(srcIn.getState());
		setTime(srcIn.getTime());
		setRedScore(srcIn.getRedScore());
		setBlueScore(srcIn.getBlueScore());
		setCubes(srcIn.getCubes());
		setRedRobots(srcIn.getRedRobots());
		setBlueRobots(srcIn.getBlueRobots());
		setRedLiftRobotIndex(srcIn.getRedLiftRobotIndex());
		setBlueLiftRobotIndex(srcIn.getBlueLiftRobotIndex());

		return srcIn;
	}

	void finishAllPendingActions(int actionIndexIn, allianceType activeAllianceIn)
	{
		float earliestFinishTime;

		//finish all pending actions and stop
		while (hasPendingActions()) {
			earliestFinishTime = getEarliestFinishTime();
			if (0 != commitAction(earliestFinishTime, actionIndexIn, activeAllianceIn)) {
				printf("Error: Pending action is rejected\n");
			}
		}
	}

	void configRedRobots(const robotConfigurationType config1In[NUMBER_OF_ROBOTS]);
	void configBlueRobots(const robotConfigurationType config1In[NUMBER_OF_ROBOTS]);

	void removeCube(int cubeIdxIn) { m_cubes[cubeIdxIn].availbleFlag = false; }

	int setRobotAction(searchActionType *pActionListInOut, allianceType allianceIn, int indexIn);
	void forceRobotAction(const pendingActionType *pPlannedActionIn, allianceType allianceIn, int robotIdxIn, int indexIn);

	float getEarliestFinishTime(void);
	int commitAction(float nextTimeIn, int indexIn, allianceType activeAllianceIn);
	bool hasPendingActions(void);

	int isGameTimeOver(void);
	void getFinalScore(int *pRedScoreOut, int *pBlueScoreOut);
	void logFinalScore(void);
	bool isRobotLifted(allianceType allianceIn, int robotIdxIn);
	bool hasPendingAction(int robotIndexIn, allianceType allianceIn);

	//find a path to go around all static objects
	bool findAvailablePath(const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn, 
		bool isTargetACubeIn, float robotTurnDelayIn, robotPathType *pPathOut);
	//Note: cube will not block robot because it could be pushed out

	bool findTheClosestCube(const rectangleObjectType *pMovingObjectIn, allianceType allianceIn, 
		float robotTurnDelayIn, float robotCubeDelayIn, cubeStateType **pCubeOut, robotPathType *pPathOut);
	int pickUpCube(coordinateType positionIn, allianceType allianceIn);

protected:
	int updateOneAction(actionTypeType actionIn, float timeIn, int robotIndexIn, allianceType allianceIn, int indexIn);

	float findOneCube(float shortestPathIn, int startSearchIdxIn, int endSearchIdxIn, bool isAllCubeSameFlag,
		const rectangleObjectType *pMovingObjectIn, float robotTurnDelayIn, float robotCubeDelayIn, 
		cubeStateType **pCubeOut, robotPathType *pPathOut);

	void updateScore(float secondsIn);
	int updateScaleSwitchScore(float secondsIn, int vaultForceBlockCountIn, int vaultBoostBlockCountIn, int balanceBlockDifferenceIn,
		vaultButtonStateType forceVaultButtonIn, vaultButtonStateType boostVaultButtonIn,
		int vaultBlockSelectionIn, ownerShipType newOnerShipIn, ownerShipType *pOwnerShipInOut);

	void logAction(actionTypeType actionIn, float timeIn, int robotIndexIn, int indexIn, bool successFlagIn);

	bool collisionWithAllOtherObjects(const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn, const rectangleObjectType **pCollisionObjectOut);

	//detect collision of two objects
	bool collisionDectection(const rectangleObjectType *pStillObjectIn, const rectangleObjectType *pMovingObjectIn, 
		coordinateType endPointIn);

	inline bool pointInRectangle(float leftXIn, float topYIn, float bottomYIn, float rightXIn, float pointXIn, float pointY) const
	{
		if ((pointXIn >= leftXIn) && (pointXIn <= rightXIn) && (pointY >= bottomYIn) && (pointY <= topYIn)) {
			return true;
		}
		else {
			return false;
		}
	}

	bool tryPickOneCube(coordinateType robotPosIn, coordinateType cubePosIn, bool cubeAvailableFlagIn);
};

