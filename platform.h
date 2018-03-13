#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "robot.h"

typedef enum structuresType {
	RED_SWITCH_ZONE,
	BLUE_SWITCH_ZONE,
	SCALE_ZONE,
	NUM_STILL_STRUCTURE
}structuresType;


typedef enum robotMoveZoneType {
	TOP_CORRIDOR,
	BOTTOM_CORRIDOR,
	LEFT_OF_RED_SWITCH,
	RIGHT_OF_RED_SWITCH,
	RIGHT_OF_SCALE,
	RIGHT_OF_BLUE_SWITCH,
	NUM_OF_ZONES,
	INVALID_ZONE
}robotMoveZoneType;

const int NUMBER_OF_ZONES_ON_PATH = 2;
const int MAXIMUM_PATH_NUMBER = 4;

typedef struct zoneConnectionType {
	robotMoveZoneType connections[NUMBER_OF_ZONES_ON_PATH];
}zoneConnectionType;

typedef struct zonePathType {
	zoneConnectionType path[MAXIMUM_PATH_NUMBER];
	int pathNUmber;
}zonePathType;


typedef struct zoneType {
	rectangleObjectType area;
	coordinateType connectionPoints[NUM_OF_ZONES]; 
	coordinateType workaroundPoints[8];   //intermediate points to around blocking objects
	int numberOfWorkaroundPoints;
}zoneType;

typedef struct platformLayoutType {

	//still structures for collision detection
	double southWall;
	double northWall;
	double eastWall;
	double westWall;
	double redAutoLine;
	double blueAutoLine;
	rectangleObjectType structures[NUM_STILL_STRUCTURE];
	zoneType zones[NUM_OF_ZONES];

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

public:
	platformLayoutType()
	{
		memset(this, 0, sizeof(struct platformLayoutType));

	}

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
	zonePathType m_zone2ZonePath[NUM_OF_ZONES][NUM_OF_ZONES];
	zonePathType m_sortedZonePath;

	double m_timeInSec; 
	double m_lastScoreUpdateTime;
	double m_redScore;
	double m_blueScore;
	int m_liftRedRobotIndex;
	int m_liftBlueRobotIndex;
	int m_debugCounter;
	FILE *m_pLogFIle;

protected:
	platformStateType m_state;
	platformLayoutType m_platformStructure;
	robot m_redRobots[NUMBER_OF_ROBOTS];
	robot m_blueRobots[NUMBER_OF_ROBOTS];
	bool m_isDisplayPlatform;

	cubeStateType m_cubes[MAX_CUBES];
	//6 cubes by each switch
	//10 cubes at each power cube zone
	//unlimited cubes at exchange zone
	//constructor will put cube to the start position;

public:
	platform();
	~platform();

	const platformStateType * getState(void) const { return &m_state; }
	double getTime(void) const { return m_timeInSec; }
	double getScoreUpdateTime(void) const { return m_lastScoreUpdateTime; }
	double getRedScore(void) const { return m_redScore; }
	double getBlueScore(void) const { return m_blueScore; }
	const cubeStateType *getCubes(void) const { return m_cubes; }
	const robot * getRedRobots(void) const { return m_redRobots; }
	const robot * getBlueRobots(void) const { return m_blueRobots; }

	double getScaleX(void) { return m_platformStructure.scaleNorthPlate.center.x; }
	double getScaleNorthY(void) { 
		return m_platformStructure.scaleNorthPlate.center.y + m_platformStructure.scaleNorthPlate.sizeY/2;
	}
	double getScaleSouthY(void) {
		return m_platformStructure.scaleSouthPlate.center.y - m_platformStructure.scaleSouthPlate.sizeY / 2;
	}

	double getRedSwitchX(void) {
		return m_platformStructure.redSwitchNorthPlate.center.x;
	}
	double getBlueSwitchX(void) {
		return m_platformStructure.blueSwitchNorthPlate.center.x;
	}
	double getRedSwitchNorthY(void) {
		return m_platformStructure.redSwitchNorthPlate.center.y + m_platformStructure.redSwitchNorthPlate.sizeY/2;
	}
	double getRedSwitchSouthY(void) {
		return m_platformStructure.redSwitchSouthPlate.center.y - m_platformStructure.redSwitchSouthPlate.sizeY / 2;
	}

	double getBlueSwitchNorthY(void) {
		return m_platformStructure.blueSwitchNorthPlate.center.y + m_platformStructure.blueSwitchNorthPlate.sizeY / 2;
	}
	double getBlueSwitchSouthY(void) {
		return m_platformStructure.blueSwitchSouthPlate.center.y - m_platformStructure.blueSwitchSouthPlate.sizeY / 2;
	}

	double getRedExchangeZoneX(void) {
		return m_platformStructure.redExchangeZone.center.x + m_platformStructure.redExchangeZone.sizeX/2;
		//Note red exchange zone is on the left side
	}
	double getRedExchangeZoneY(void) {
		return m_platformStructure.redExchangeZone.center.y;
	}
	double getBlueExchangeZoneX(void) {
		return m_platformStructure.blueExchangeZone.center.x - m_platformStructure.blueExchangeZone.sizeX / 2;
		//NOte: blue exchange zone is on the right side
	}
	double getBlueExchangeZoneY(void) {
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
	coordinateType getRobotPos(allianceType allianceIn, int robotIdxIn) const;

	bool getRobotHasCubeFlag(allianceType allianceIn, int robotIdxIn) const;
	int getRobotCubeIdx(allianceType allianceIn, int robotIdxIn) const;

	void setState(const platformStateType *pStateIn) { memcpy(&m_state, pStateIn,  sizeof(m_state)); }
	void setTime(double timeIn) { m_timeInSec = timeIn; }
	void setScoreUpdateTime(double timeIn) { m_lastScoreUpdateTime = timeIn; }
	void setRedScore(double redScoreIn) { m_redScore = redScoreIn; }
	void setBlueScore(double blueScoreIn) { m_blueScore = blueScoreIn; }
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
		setScoreUpdateTime(getScoreUpdateTime());
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
		double earliestFinishTime;

		//finish all pending actions and stop
		while (hasPendingActions()) {
			earliestFinishTime = getEarliestStopTime();
			if (0 != commitAction(earliestFinishTime, actionIndexIn, activeAllianceIn)) {
				printf("Error: Pending action is rejected\n");
			}
		}
	}

	void configRedRobots(const robotConfigurationType config1In[NUMBER_OF_ROBOTS]);
	void configBlueRobots(const robotConfigurationType config1In[NUMBER_OF_ROBOTS]);

	void removeCube(int cubeIdxIn) { m_cubes[cubeIdxIn].availbleFlag = false; }

	int setRobotAction(searchActionType *pActionListInOut, allianceType allianceIn, int indexIn);
	void forceRobotAction(const pendingActionType *pPlannedActionIn, coordinateType startPosIn, int cubeIdxIn,
		allianceType allianceIn, int robotIdxIn, int indexIn);

	double getEarliestStopTime(void);
	int commitAction(double nextTimeIn, int indexIn, allianceType activeAllianceIn);
	bool hasPendingActions(void);

	int isGameTimeOver(void);
	void getFinalScore(int *pRedScoreOut, int *pBlueScoreOut);
	void logFinalScore(void);
	bool isRobotLifted(allianceType allianceIn, int robotIdxIn);
	bool hasPendingAction(int robotIndexIn, allianceType allianceIn);

	//find a path to go around all static objects
	bool findAvailablePath(const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn, 
		bool isTargetACubeIn, double robotTurnDelayIn, robotPathType *pPathOut);
	//Note: cube will not block robot because it could be pushed out

	bool findTheClosestCube(const rectangleObjectType *pMovingObjectIn, allianceType allianceIn, 
		double robotTurnDelayIn, double robotCubeDelayIn, cubeStateType **pCubeOut, robotPathType *pPathOut);
	int pickUpCube(coordinateType positionIn, allianceType allianceIn);

	static double calculateDistance(coordinateType point1In, coordinateType point2In)
	{
		double distance;
		distance = (point1In.x - point2In.x)*(point1In.x - point2In.x) + (point1In.y - point2In.y)*(point1In.y - point2In.y);

		distance = sqrt(distance);
		return distance;
	}

	bool collisionWithAllOtherObjects(const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn,
		const rectangleObjectType **pCollisionObjectOut);

protected:
	int updateOneAction(actionTypeType actionIn, double timeIn, int robotIndexIn, allianceType allianceIn, int indexIn);

	double findOneCube(double shortestPathIn, int startSearchIdxIn, int endSearchIdxIn, bool isAllCubeSameFlag,
		const rectangleObjectType *pMovingObjectIn, double robotTurnDelayIn, double robotCubeDelayIn, 
		cubeStateType **pCubeOut, robotPathType *pPathOut);

	void updateScore(double secondsPassedIn);
	double updateScaleSwitchScore(double secondsIn, int vaultForceBlockCountIn, int vaultBoostBlockCountIn, int balanceBlockDifferenceIn,
		vaultButtonStateType forceVaultButtonIn, vaultButtonStateType boostVaultButtonIn,
		int vaultBlockSelectionIn, ownerShipType newOnerShipIn, ownerShipType *pOwnerShipInOut);

	void logAction(actionTypeType actionIn, double timeIn, int robotIndexIn, int indexIn, bool successFlagIn);

	//detect collision of two objects
	bool collisionDectection(const rectangleObjectType *pStillObjectIn, const rectangleObjectType *pMovingObjectIn, 
		coordinateType endPointIn);

	bool pointInRectangle(coordinateType aIn, coordinateType bIn, coordinateType cIn, coordinateType dIn, double pointXIn, double pointYIn) const;

	inline bool pointInObject(const rectangleObjectType *pObjectIn, double pointXIn, double pointYIn) const
	{
		double leftX, topY, bottomY, rightX;

		leftX = pObjectIn->center.x - pObjectIn->sizeX / 2;
		rightX = pObjectIn->center.x + pObjectIn->sizeX / 2;
		topY = pObjectIn->center.y + pObjectIn->sizeY / 2;
		bottomY = pObjectIn->center.y - pObjectIn->sizeY / 2;

		if ((pointXIn >= leftX) && (pointXIn <= rightX) && (pointYIn >= bottomY) && (pointYIn <= topY)) {
			return true;
		}
		else {
			return false;
		}
	}

	bool tryPickOneCube(coordinateType robotPosIn, coordinateType cubePosIn, bool cubeAvailableFlagIn);

	robotMoveZoneType getObjectZone(const rectangleObjectType *pObjectIn);
	bool objectInZone(const rectangleObjectType *pZoneIn, const rectangleObjectType *pObjectIn);

	robotMoveZoneType getPointZone(coordinateType pointIn)
	{
		rectangleObjectType object;
		object.center = pointIn;
		object.sizeX = 0;
		object.sizeY = 0;

		return getObjectZone(&object);
	}

	bool foundPathWithinZone(const rectangleObjectType *pRobotIn, coordinateType targetIn,
		const zoneType *pzoneIn, robotPathType *pPathInOut);

	double estimatePathDistance(coordinateType startIn, robotMoveZoneType startZoneIn,
		coordinateType targetIn, robotMoveZoneType targetZoneIn, int pathIdIn);

	void sortZoneConnections(coordinateType startIn, robotMoveZoneType startZoneIn,
		coordinateType targetIn, robotMoveZoneType targetZoneIn, zonePathType *pPathListOut);
};

