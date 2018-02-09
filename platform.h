#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "robot.h"

const int MAX_CUBES = 2000;

typedef enum allianceType {
	ALLIANCE_RED,
	ALLIANCE_BLUE
}allianceType;

typedef enum structuresType {
	RED_EXCHANGE_ZONE,
	BLUE_EXCHANGE_ZONE,
	RED_POWERCUBE_ZONE,
	BLUE_POWERCUB_ZONE,
	RED_SWITCH_ZONE,
	BLUE_SWITCH_ZONE,
	SCALE_ZONE,
	NUM_STILL_STRUCTURE
}structuresType;


typedef struct platformLayoutType {
	float southWall;
	float northWall;
	float eastWall;
	float westWall;
	float redAutoLine;
	float blueAutoLine;

	rectangleObjectType structures[NUM_STILL_STRUCTURE];

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
}platformLayoutType;

//the indexes to all cubes in the cube array
typedef enum cubeIndexType {
	CUBE_BY_RED_SWITCH = 0,
	CUBE_BY_BLUE_SWITCH = 6,
	CUBE_BY_RED_POWER_ZONE = 12,
	CUBE_BY_BLUE_POWER_ZONE = 22,
	CUBE_BY_RED_EXCHANGE_ZONE = 32,
	CUBE_BY_BLUE_EXCHANGE_ZONE = 1064,
	CUBE_LAST = MAX_CUBES - 1
}cubeIndexType;


class platform
{
private:
	platformStateType m_state;
	float m_timeInSec; 
	int m_redScore;
	int m_blueScore;
	int m_liftRedRobotIndex;
	int m_liftBlueRobotIndex;
	FILE *m_pLogFIle;

	platformLayoutType m_platformStructure;

	cubeStateType m_cubes[MAX_CUBES];
	//6 cubes by each switch
	//10 cubes at each power cube zone
	//unlimited cubes at exchange zone
	//constructor will put cube to the start position;

	robot m_redRobots[NUMBER_OF_ROBOTS];
	robot m_blueRobots[NUMBER_OF_ROBOTS];

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

	void setState(const platformStateType *pStateIn) { memcpy(&m_state, pStateIn,  sizeof(m_state)); }
	void setTime(float timeIn) { m_timeInSec = timeIn; }
	void setRedScore(int redScoreIn) { m_redScore = redScoreIn; }
	void setBlueScore(int blueScoreIn) { m_blueScore = blueScoreIn; }
	void setLogFIle(FILE *pFileIn) { m_pLogFIle = pFileIn; }
	void setCubes(const cubeStateType *pCubesIn) {
		memcpy(m_cubes, pCubesIn, sizeof(cubeStateType)*MAX_CUBES);
	}
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
		return srcIn;
	}

	void configRedRobots(const robotConfigurationType config1In[NUMBER_OF_ROBOTS])
	{
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
			m_redRobots[i].setConfiguration(config1In);
		}
	}
	void configBlueRobots(const robotConfigurationType config1In[NUMBER_OF_ROBOTS])
	{
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
			m_redRobots[i].setConfiguration(config1In);
		}
	}

	int takeAction(actionTypeType actionIn, float timeIn, int robotIndexIn, int indexIn);

	int isGameTimeOver(void);
	void getFinalScore(int *pRedScoreOut, int *pBlueScoreOut);
	void logFinalScore(void);
	bool isRobotLifted(allianceType allianceIn, int robotIdxIn);

	//find a path to go around all static objects
	bool findAvailablePath(const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn, 
		bool isTargetACubeIn, robotPathType *pPathOut);
	//Note: cube will not block robot because it could be pushed out

	void findTheClosestCube(const rectangleObjectType *pMovingObjectIn, allianceType allianceIn, cubeStateType *pCubeOut, robotPathType *pPathOut);

protected:
	float findOneCube(float shortestPathIn, int startSearchIdxIn, int endSearchIdxIn, bool isAllCubeSameFlag,
		const rectangleObjectType *pMovingObjectIn, cubeStateType *pCubeOut, robotPathType *pPathOut);

	void updateScore(float secondsIn);
	int updateScaleSwitchScore(float secondsIn, int vaultForceBlockCountIn, int vaultBoostBlockCountIn, int balanceBlockDifferenceIn,
		vaultButtonStateType forceVaultButtonIn, vaultButtonStateType boostVaultButtonIn,
		int vaultBlockSelectionIn, ownerShipType newOnerShipIn, ownerShipType *pOwnerShipInOut);

	void logAction(actionTypeType actionIn, float timeIn, int robotIndexIn, int indexIn);

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
};

