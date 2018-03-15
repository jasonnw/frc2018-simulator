
//#include "stdafx.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "platform.h"
#include "robot_blue0.h"
#include "robot_blue1.h"
#include "robot_blue2.h"

#define ROUNDING_METHOD(x) ((int)floor((x)))

platform::platform()
{
	memset(&m_state, 0, sizeof(m_state));
	memset(&m_platformStructure, 0, sizeof(m_platformStructure));
	m_timeInSec = 0;
	m_lastScoreUpdateTime = 0;
	m_redScore = 0;
	m_blueScore = 0;

	m_redRank = 0;
	m_blueRank = 0;

	m_pLogFIle = NULL;
	m_liftRedRobotIndex = INVALID_IDX;
	m_liftBlueRobotIndex = INVALID_IDX;
	m_debugCounter = 0;
	m_isDisplayPlatform = false;

	if (NUMBER_OF_ROBOTS != 3) {
		printf("ERROR, The number of robots are not expected");
		return;
	}

	m_pRedRobots[0] = new robot;
	m_pRedRobots[1] = new robot;
	m_pRedRobots[2] = new robot;

	m_pBlueRobots[0] = new robotBlue0;
	m_pBlueRobots[1] = new robotBlue1;
	m_pBlueRobots[2] = new robotBlue2;

	if ((m_pRedRobots[0] == NULL) ||
		(m_pRedRobots[1] == NULL) ||
		(m_pRedRobots[2] == NULL) ||
		(m_pBlueRobots[0] == NULL) ||
		(m_pBlueRobots[1] == NULL) ||
		(m_pBlueRobots[2] == NULL)) {
		printf("ERROR, allocate robots failed\n");
		return;
	}

	//give each robot access with the platform
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_pRedRobots[i]->setPlatformAndCube(this, i+ CUBE_ON_RED_ROBOTS);
		m_pBlueRobots[i]->setPlatformAndCube(this, i + CUBE_ON_BLUE_ROBOTS);
	}

	//field layout, in unit of inch,
	//red alliance is on the left side, blue is on the right side
	//the whole field is (288*2 + 72) x (264 + 48*2)
	m_platformStructure.blueAutoLine = (288 * 2 + 72) - 10 * 12;

	m_platformStructure.blueExchangeZone.objectId = 1;
	m_platformStructure.blueExchangeZone.center.x = (288 * 2 + 72) - 36;
	m_platformStructure.blueExchangeZone.center.y = (264 + 48 * 2) / 2 - 12 - 24;
	m_platformStructure.blueExchangeZone.sizeX = 36;
	m_platformStructure.blueExchangeZone.sizeY = 48;
	m_platformStructure.blueExchangeZone.color = { 100, 10, 10 };

	m_platformStructure.blueLiftZone.objectId = 2;
	m_platformStructure.blueLiftZone.center.x = ((288 * 2 + 72) - 261.74) - 20;
	m_platformStructure.blueLiftZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.blueLiftZone.sizeX = ((288 * 2 + 72)/2 - 261.74);
	m_platformStructure.blueLiftZone.sizeY = 9 * 12;
	m_platformStructure.blueLiftZone.color = { 100, 0, 0 };

	m_platformStructure.bluePlatformZone.objectId = 3;
	m_platformStructure.bluePlatformZone.center.x = (288 * 2 + 72) - 196;
	m_platformStructure.bluePlatformZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.bluePlatformZone.sizeX = (288 * 2 + 72) / 2 - 196;
	m_platformStructure.bluePlatformZone.sizeY = 9 * 12;
	m_platformStructure.bluePlatformZone.color = { 100, 10, 10 };

	m_platformStructure.bluePowerCubeZone.objectId = 4;
	m_platformStructure.bluePowerCubeZone.center.x = (288 * 2 + 72) - 98 - 42 / 2;
	m_platformStructure.bluePowerCubeZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.bluePowerCubeZone.sizeX = 42;
	m_platformStructure.bluePowerCubeZone.sizeY = 45;
	m_platformStructure.bluePowerCubeZone.color = { 100, 0, 0 };

	m_platformStructure.blueSwitchNorthPlate.objectId = 5;
	m_platformStructure.blueSwitchNorthPlate.center.x = (288 * 2 + 72) - 140 - 56 / 2;
	m_platformStructure.blueSwitchNorthPlate.center.y = (264 + 48 * 2) / 2 + 6 * 12;
	m_platformStructure.blueSwitchNorthPlate.sizeX = 4 * 12;
	m_platformStructure.blueSwitchNorthPlate.sizeY = 3 * 12;
	m_platformStructure.blueSwitchNorthPlate.color = { 100, 10, 10 };

	m_platformStructure.blueSwitchSouthPlate.objectId = 6;
	m_platformStructure.blueSwitchSouthPlate.center.x = (288 * 2 + 72) - 140 - 56 / 2;
	m_platformStructure.blueSwitchSouthPlate.center.y = (264 + 48 * 2) / 2 - 6 * 12;
	m_platformStructure.blueSwitchSouthPlate.sizeX = 4 * 12;
	m_platformStructure.blueSwitchSouthPlate.sizeY = 3 * 12;
	m_platformStructure.blueSwitchSouthPlate.color = { 100, 0, 0 };

	if (BLUE_NORTH_SWITCH_FLAG) {
		m_platformStructure.blueSwitchNorthPlate.color = { 120, 10, 10 };
		m_platformStructure.blueSwitchSouthPlate.color = { 10, 10, 120 };
	}
	else {
		m_platformStructure.blueSwitchNorthPlate.color = { 10, 10, 120 };
		m_platformStructure.blueSwitchSouthPlate.color = { 120, 10, 10 };
	}

	m_platformStructure.eastWall = (288 * 2 + 72);
	m_platformStructure.northWall = (264 + 48 * 2);
	m_platformStructure.redAutoLine = 10 * 12;
	
	m_platformStructure.redExchangeZone.objectId = 7;
	m_platformStructure.redExchangeZone.center.x = 36;
	m_platformStructure.redExchangeZone.center.y = (264 + 48 * 2) / 2 + 12 + 24;
	m_platformStructure.redExchangeZone.sizeX = 36;
	m_platformStructure.redExchangeZone.sizeY = 48;
	m_platformStructure.redExchangeZone.color = { 10, 10, 100 };

	m_platformStructure.redLiftZone.objectId = 8;
	m_platformStructure.redLiftZone.center.x = 261.74 + 20;
	m_platformStructure.redLiftZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.redLiftZone.sizeX = ((288 * 2 + 72) / 2 - 261.74);
	m_platformStructure.redLiftZone.sizeY = 9 * 12;
	m_platformStructure.redLiftZone.color = { 0, 0, 100 };

	m_platformStructure.redPlatformZone.objectId = 9;
	m_platformStructure.redPlatformZone.center.x = 196;
	m_platformStructure.redPlatformZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.redPlatformZone.sizeX = (288 * 2 + 72) / 2 - 196;
	m_platformStructure.redPlatformZone.sizeY = 9 * 12;
	m_platformStructure.redPlatformZone.color = { 10, 10, 100 };

	m_platformStructure.redPowerCubeZone.objectId = 10;
	m_platformStructure.redPowerCubeZone.center.x = 98 + 42 / 2;
	m_platformStructure.redPowerCubeZone.center.y = (264 + 48 * 2) / 2;
	m_platformStructure.redPowerCubeZone.sizeX = 42;
	m_platformStructure.redPowerCubeZone.sizeY = 45;
	m_platformStructure.redPowerCubeZone.color = { 0, 0, 100 };


	m_platformStructure.redSwitchNorthPlate.objectId = 11;
	m_platformStructure.redSwitchNorthPlate.center.x = 140 + 56 / 2;
	m_platformStructure.redSwitchNorthPlate.center.y = (264 + 48 * 2) / 2 + 6 * 12;
	m_platformStructure.redSwitchNorthPlate.sizeX = 4 * 12;
	m_platformStructure.redSwitchNorthPlate.sizeY = 3 * 12;

	m_platformStructure.redSwitchSouthPlate.objectId = 12;
	m_platformStructure.redSwitchSouthPlate.center.x = 140 + 56 / 2;
	m_platformStructure.redSwitchSouthPlate.center.y = (264 + 48 * 2) / 2 - 6 * 12;
	m_platformStructure.redSwitchSouthPlate.sizeX = 4 * 12;
	m_platformStructure.redSwitchSouthPlate.sizeY = 3 * 12;

	if (RED_NORTH_SWITCH_FLAG) {
		m_platformStructure.redSwitchNorthPlate.color = { 10, 10, 120 };
		m_platformStructure.redSwitchSouthPlate.color = { 120, 10, 10 };
	}
	else {
		m_platformStructure.redSwitchNorthPlate.color = { 120, 10, 10 };
		m_platformStructure.redSwitchSouthPlate.color = { 10, 10, 120 };
	}

	m_platformStructure.scaleNorthPlate.objectId = 13;
	m_platformStructure.scaleNorthPlate.center.x = (288 * 2 + 72) / 2;
	m_platformStructure.scaleNorthPlate.center.y = (264 + 48 * 2) / 2 + (15 * 12 / 2);
	m_platformStructure.scaleNorthPlate.sizeX = 4 * 12;
	m_platformStructure.scaleNorthPlate.sizeY = 3 * 12;

	m_platformStructure.scaleSouthPlate.objectId = 14;
	m_platformStructure.scaleSouthPlate.center.x = (288 * 2 + 72) / 2;
	m_platformStructure.scaleSouthPlate.center.y = (264 + 48 * 2) / 2 - (15 * 12 / 2);
	m_platformStructure.scaleSouthPlate.sizeX = 4 * 12;
	m_platformStructure.scaleSouthPlate.sizeY = 3 * 12;

	if (RED_NORTH_SCALE_FLAG) {
		m_platformStructure.scaleNorthPlate.color = { 10, 10, 100 };
		m_platformStructure.scaleSouthPlate.color = { 100, 10, 10 };
	}
	else {
		m_platformStructure.scaleNorthPlate.color = { 100, 10, 10 };
		m_platformStructure.scaleSouthPlate.color = { 10, 10, 10 };
	}

	m_platformStructure.southWall = 0;
	m_platformStructure.westWall = 0;

	m_platformStructure.structures[RED_SWITCH_ZONE].objectId = 15;
	m_platformStructure.structures[RED_SWITCH_ZONE].center.x = 140 + 56 / 2;
	m_platformStructure.structures[RED_SWITCH_ZONE].center.y = (264 + 48 * 2) / 2;
	m_platformStructure.structures[RED_SWITCH_ZONE].sizeX = 4 * 12 + 8;
	m_platformStructure.structures[RED_SWITCH_ZONE].sizeY = 12 * 12 + 9.5;
	m_platformStructure.structures[RED_SWITCH_ZONE].color = { 10, 10, 100 };

	m_platformStructure.structures[BLUE_SWITCH_ZONE].objectId = 16;
	m_platformStructure.structures[BLUE_SWITCH_ZONE].center.x = (288 * 2 + 72) - 140 - 56 / 2;
	m_platformStructure.structures[BLUE_SWITCH_ZONE].center.y = (264 + 48 * 2) / 2;
	m_platformStructure.structures[BLUE_SWITCH_ZONE].sizeX = 4 * 12 + 8;
	m_platformStructure.structures[BLUE_SWITCH_ZONE].sizeY = 12 * 12 + 9.5;
	m_platformStructure.structures[BLUE_SWITCH_ZONE].color = { 100, 10, 10 };

	m_platformStructure.structures[SCALE_ZONE].objectId = 17;
	m_platformStructure.structures[SCALE_ZONE].center.x = (288 * 2 + 72) / 2;
	m_platformStructure.structures[SCALE_ZONE].center.y = (264 + 48 * 2) / 2;
	m_platformStructure.structures[SCALE_ZONE].sizeX = 4 * 12;
	m_platformStructure.structures[SCALE_ZONE].sizeY = 15 * 12;
	m_platformStructure.structures[SCALE_ZONE].color = { 100, 100, 100 };


	for (int i = 0; i < NUM_OF_ZONES; i++) {
		for (int j = 0; j < NUM_OF_ZONES; j++) {
			m_platformStructure.zones[i].connectionPoints[j] = { 0, 0 };
		}
	}

	//LEFT_OF_RED_SWITCH
	m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x = (
		    m_platformStructure.structures[RED_SWITCH_ZONE].center.x -
			m_platformStructure.structures[RED_SWITCH_ZONE].sizeX / 2 - 20 +
			m_platformStructure.westWall) / 2;

	m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.y =
		(m_platformStructure.southWall + m_platformStructure.northWall) / 2;

	m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeX =
		(m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x - m_platformStructure.westWall) * 2;
	m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeY = m_platformStructure.structures[SCALE_ZONE].sizeY;

	m_platformStructure.zones[LEFT_OF_RED_SWITCH].connectionPoints[BOTTOM_CORRIDOR] =
	{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x, 
		m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.y - m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeY / 2 - 24};
	m_platformStructure.zones[LEFT_OF_RED_SWITCH].connectionPoints[TOP_CORRIDOR] =
	{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x,
		m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.y + m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeY / 2 + 24};

	m_platformStructure.zones[LEFT_OF_RED_SWITCH].numberOfWorkaroundPoints = 4;
	m_platformStructure.zones[LEFT_OF_RED_SWITCH].workaroundPoints[0] =
	{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x + m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeX / 2 - 18,
		m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.y - m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeY / 4 };
	m_platformStructure.zones[LEFT_OF_RED_SWITCH].workaroundPoints[1] =
	{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x + m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeX / 2 - 18,
		m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.y + m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeY / 4 };

	m_platformStructure.zones[LEFT_OF_RED_SWITCH].workaroundPoints[2] =
	{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x - m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeX / 2 + 18,
		m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.y - m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeY / 4 };
	m_platformStructure.zones[LEFT_OF_RED_SWITCH].workaroundPoints[3] =
	{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x - m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeX / 2 + 18,
		m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.y + m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.sizeY / 4 };


	//RIGHT_OF_RED_SWITCH
	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x =
		(m_platformStructure.structures[RED_SWITCH_ZONE].center.x + m_platformStructure.structures[RED_SWITCH_ZONE].sizeX / 2 +
		 m_platformStructure.structures[SCALE_ZONE].center.x - m_platformStructure.structures[SCALE_ZONE].sizeX / 2) / 2;
	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.y =
		(m_platformStructure.southWall + m_platformStructure.northWall) / 2;

	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeX =
		m_platformStructure.structures[SCALE_ZONE].center.x - m_platformStructure.structures[RED_SWITCH_ZONE].center.x -
		m_platformStructure.structures[RED_SWITCH_ZONE].sizeX/2 - 18;
	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeY = m_platformStructure.structures[SCALE_ZONE].sizeY;

	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].connectionPoints[BOTTOM_CORRIDOR] =
	{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x, 
		m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.y - m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeY / 2 - 24};
	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].connectionPoints[TOP_CORRIDOR] =
	{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x, 
		m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.y + m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeY / 2 + 24};

	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].numberOfWorkaroundPoints = 4;
	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].workaroundPoints[0] =
	{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x + m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeX / 2 - 18,
		m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.y - m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeY / 4 };
	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].workaroundPoints[1] =
	{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x + m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeX / 2 - 18,
		m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.y + m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeY / 4 };

	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].workaroundPoints[2] =
	{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x - m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeX / 2 + 18,
		m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.y - m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeY / 4 };
	m_platformStructure.zones[RIGHT_OF_RED_SWITCH].workaroundPoints[3] =
	{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x - m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeX / 2 + 18,
		m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.y + m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.sizeY / 4 };


	//RIGHT_OF_SCALE
	m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x = 
		(m_platformStructure.structures[BLUE_SWITCH_ZONE].center.x - m_platformStructure.structures[BLUE_SWITCH_ZONE].sizeX / 2 +
		 m_platformStructure.structures[SCALE_ZONE].center.x + m_platformStructure.structures[SCALE_ZONE].sizeX / 2) / 2;
	m_platformStructure.zones[RIGHT_OF_SCALE].area.center.y =
		(m_platformStructure.southWall + m_platformStructure.northWall) / 2;

	m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeX =
		m_platformStructure.structures[BLUE_SWITCH_ZONE].center.x - m_platformStructure.structures[SCALE_ZONE].center.x -
		m_platformStructure.structures[BLUE_SWITCH_ZONE].sizeX / 2 - 18;
	m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeY = m_platformStructure.structures[SCALE_ZONE].sizeY;

	m_platformStructure.zones[RIGHT_OF_SCALE].connectionPoints[BOTTOM_CORRIDOR] =
	{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x, 
		m_platformStructure.zones[RIGHT_OF_SCALE].area.center.y - m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeY / 2 - 24};
	m_platformStructure.zones[RIGHT_OF_SCALE].connectionPoints[TOP_CORRIDOR] =
	{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x, 
		m_platformStructure.zones[RIGHT_OF_SCALE].area.center.y + m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeY / 2 + 24};

	m_platformStructure.zones[RIGHT_OF_SCALE].numberOfWorkaroundPoints = 4;
	m_platformStructure.zones[RIGHT_OF_SCALE].workaroundPoints[0] =
	{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x + m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeX/2 - 18,
		m_platformStructure.zones[RIGHT_OF_SCALE].area.center.y - m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeY / 4 };
	m_platformStructure.zones[RIGHT_OF_SCALE].workaroundPoints[1] =
	{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x + m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeX / 2 - 18,
		m_platformStructure.zones[RIGHT_OF_SCALE].area.center.y + m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeY / 4 };

	m_platformStructure.zones[RIGHT_OF_SCALE].workaroundPoints[2] =
	{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x - m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeX / 2 + 18,
		m_platformStructure.zones[RIGHT_OF_SCALE].area.center.y - m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeY / 4 };
	m_platformStructure.zones[RIGHT_OF_SCALE].workaroundPoints[3] =
	{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x - m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeX / 2 + 18,
		m_platformStructure.zones[RIGHT_OF_SCALE].area.center.y + m_platformStructure.zones[RIGHT_OF_SCALE].area.sizeY / 4 };

	//RIGHT_OF_BLUE_SWITCH
	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.x = (20 + //cube pile size
		 m_platformStructure.eastWall + 
		 m_platformStructure.structures[BLUE_SWITCH_ZONE].center.x +
		 m_platformStructure.structures[BLUE_SWITCH_ZONE].sizeX / 2) / 2;

	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.y =
		(m_platformStructure.southWall + m_platformStructure.northWall) / 2;

	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.sizeX =
		(m_platformStructure.eastWall - m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.x) * 2;
	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.sizeY = m_platformStructure.structures[SCALE_ZONE].sizeY;

	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].connectionPoints[BOTTOM_CORRIDOR] = 
		{ m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.x, 
		m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.y - m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.sizeY / 2 - 24};
	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].connectionPoints[TOP_CORRIDOR] =
		{ m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.x, 
		m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.y + m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.sizeY / 2 + 24};

	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].numberOfWorkaroundPoints = 4;
	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].workaroundPoints[0] =
	{ m_platformStructure.eastWall - 18, 
	  m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.y - m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.sizeY /4};
	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].workaroundPoints[1] =
	{ m_platformStructure.eastWall - 18,
		m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.y + m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.sizeY / 4 };

	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].workaroundPoints[2] =
	{ m_platformStructure.eastWall - m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.sizeX  + 18,
		m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.y - m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.sizeY / 4 };
	m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].workaroundPoints[3] =
	{ m_platformStructure.eastWall - m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.sizeX + 18,
		m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.y + m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.sizeY / 4 };

	//bottom corridor
	m_platformStructure.zones[BOTTOM_CORRIDOR].area.center.x =
		(m_platformStructure.eastWall - m_platformStructure.westWall) / 2;
	m_platformStructure.zones[BOTTOM_CORRIDOR].area.center.y =
		(m_platformStructure.structures[SCALE_ZONE].center.y -
			m_platformStructure.structures[SCALE_ZONE].sizeY / 2 +
			m_platformStructure.southWall) / 2;

	m_platformStructure.zones[BOTTOM_CORRIDOR].area.sizeX =
		(m_platformStructure.eastWall - m_platformStructure.westWall);

	m_platformStructure.zones[BOTTOM_CORRIDOR].area.sizeY =
		(m_platformStructure.zones[BOTTOM_CORRIDOR].area.center.y - m_platformStructure.southWall) * 2;


	m_platformStructure.zones[BOTTOM_CORRIDOR].connectionPoints[LEFT_OF_RED_SWITCH] = 
		{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x, m_platformStructure.zones[BOTTOM_CORRIDOR].area.sizeY - 24};
	m_platformStructure.zones[BOTTOM_CORRIDOR].connectionPoints[RIGHT_OF_RED_SWITCH] =
		{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x, m_platformStructure.zones[BOTTOM_CORRIDOR].area.sizeY - 24};
	m_platformStructure.zones[BOTTOM_CORRIDOR].connectionPoints[RIGHT_OF_SCALE] =
		{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x, m_platformStructure.zones[BOTTOM_CORRIDOR].area.sizeY - 24};
	m_platformStructure.zones[BOTTOM_CORRIDOR].connectionPoints[RIGHT_OF_BLUE_SWITCH] =
		{ m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.x, m_platformStructure.zones[BOTTOM_CORRIDOR].area.sizeY - 24};

	m_platformStructure.zones[BOTTOM_CORRIDOR].numberOfWorkaroundPoints = 8;
	m_platformStructure.zones[BOTTOM_CORRIDOR].workaroundPoints[0] =
		{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x, 18 };
	m_platformStructure.zones[BOTTOM_CORRIDOR].workaroundPoints[1] =
		{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x, 18 };
	m_platformStructure.zones[BOTTOM_CORRIDOR].workaroundPoints[2] =
		{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x, 18 };
	m_platformStructure.zones[BOTTOM_CORRIDOR].workaroundPoints[3] =
		{ m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.x, 18 };

	m_platformStructure.zones[BOTTOM_CORRIDOR].workaroundPoints[4] =
		{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x, m_platformStructure.zones[BOTTOM_CORRIDOR].area.sizeY };
	m_platformStructure.zones[BOTTOM_CORRIDOR].workaroundPoints[5] =
		{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x, m_platformStructure.zones[BOTTOM_CORRIDOR].area.sizeY };
	m_platformStructure.zones[BOTTOM_CORRIDOR].workaroundPoints[6] =
		{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x, m_platformStructure.zones[BOTTOM_CORRIDOR].area.sizeY };
	m_platformStructure.zones[BOTTOM_CORRIDOR].workaroundPoints[7] =
		{ m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.x, m_platformStructure.zones[BOTTOM_CORRIDOR].area.sizeY };

	//top corridor
	m_platformStructure.zones[TOP_CORRIDOR].area.center.x =
		(m_platformStructure.eastWall - m_platformStructure.westWall) / 2;
	m_platformStructure.zones[TOP_CORRIDOR].area.center.y =
		(m_platformStructure.northWall + 
			m_platformStructure.structures[SCALE_ZONE].center.y +
			m_platformStructure.structures[SCALE_ZONE].sizeY / 2) / 2;

	m_platformStructure.zones[TOP_CORRIDOR].area.sizeX =
		(m_platformStructure.eastWall - m_platformStructure.westWall);

	m_platformStructure.zones[TOP_CORRIDOR].area.sizeY =
		(m_platformStructure.northWall - m_platformStructure.zones[TOP_CORRIDOR].area.center.y) * 2;

	m_platformStructure.zones[TOP_CORRIDOR].connectionPoints[LEFT_OF_RED_SWITCH] =
		{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x, 
		m_platformStructure.northWall - m_platformStructure.zones[TOP_CORRIDOR].area.sizeY + 24};
	m_platformStructure.zones[TOP_CORRIDOR].connectionPoints[RIGHT_OF_RED_SWITCH] =
		{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x, 
		m_platformStructure.northWall - m_platformStructure.zones[TOP_CORRIDOR].area.sizeY + 24};
	m_platformStructure.zones[TOP_CORRIDOR].connectionPoints[RIGHT_OF_SCALE] =
		{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x, 
		m_platformStructure.northWall - m_platformStructure.zones[TOP_CORRIDOR].area.sizeY + 24};
	m_platformStructure.zones[TOP_CORRIDOR].connectionPoints[RIGHT_OF_BLUE_SWITCH] =
		{ m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.x, 
		m_platformStructure.northWall - m_platformStructure.zones[TOP_CORRIDOR].area.sizeY + 24};

	m_platformStructure.zones[TOP_CORRIDOR].numberOfWorkaroundPoints = 8;
	m_platformStructure.zones[TOP_CORRIDOR].workaroundPoints[0] =
		{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x, m_platformStructure.northWall - 18 };
	m_platformStructure.zones[TOP_CORRIDOR].workaroundPoints[1] =
		{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x, m_platformStructure.northWall - 18 };
	m_platformStructure.zones[TOP_CORRIDOR].workaroundPoints[2] =
		{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x, m_platformStructure.northWall - 18 };
	m_platformStructure.zones[TOP_CORRIDOR].workaroundPoints[3] =
		{ m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.x, m_platformStructure.northWall - 18 };

	m_platformStructure.zones[TOP_CORRIDOR].workaroundPoints[4] =
		{ m_platformStructure.zones[LEFT_OF_RED_SWITCH].area.center.x,
			m_platformStructure.northWall - m_platformStructure.zones[TOP_CORRIDOR].area.sizeY };
	m_platformStructure.zones[TOP_CORRIDOR].workaroundPoints[5] =
		{ m_platformStructure.zones[RIGHT_OF_RED_SWITCH].area.center.x,
			m_platformStructure.northWall - m_platformStructure.zones[TOP_CORRIDOR].area.sizeY };
	m_platformStructure.zones[TOP_CORRIDOR].workaroundPoints[6] =
		{ m_platformStructure.zones[RIGHT_OF_SCALE].area.center.x,
			m_platformStructure.northWall - m_platformStructure.zones[TOP_CORRIDOR].area.sizeY };
	m_platformStructure.zones[TOP_CORRIDOR].workaroundPoints[7] =
		{ m_platformStructure.zones[RIGHT_OF_BLUE_SWITCH].area.center.x,
			m_platformStructure.northWall - m_platformStructure.zones[TOP_CORRIDOR].area.sizeY };


	for (int i = CUBE_BY_RED_SWITCH; i < CUBE_BY_BLUE_SWITCH; i++) {
		m_cubes[i].index = i;
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position.x = 196 + 6 + 10;
		m_cubes[i].position.y = (double) ((264 + 48 * 2) / 2 - (12 * 12 + 9.5) / 2 + (i- CUBE_BY_RED_SWITCH) * 30);
	}
	for (int i = CUBE_BY_BLUE_SWITCH; i < CUBE_BY_RED_POWER_ZONE; i++) {
		m_cubes[i].index = i;
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position.x = (288 * 2 + 72) - 196 - 6 - 10;
		m_cubes[i].position.y = (double)((264 + 48 * 2) / 2 - (12 * 12 + 9.5) / 2 + (i- CUBE_BY_BLUE_SWITCH) * 30);
	}
	for (int i = CUBE_BY_RED_POWER_ZONE; i < CUBE_BY_BLUE_POWER_ZONE; i++) {
		m_cubes[i].index = i;
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position = m_platformStructure.redPowerCubeZone.center;
	}
	for (int i = CUBE_BY_BLUE_POWER_ZONE; i < CUBE_BY_RED_EXCHANGE_ZONE; i++) {
		m_cubes[i].index = i;
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position = m_platformStructure.bluePowerCubeZone.center;
	}
	for (int i = CUBE_BY_RED_EXCHANGE_ZONE; i < CUBE_BY_BLUE_EXCHANGE_ZONE; i++) {
		m_cubes[i].index = i;
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position = m_platformStructure.redExchangeZone.center;
	}
	for (int i = CUBE_BY_BLUE_EXCHANGE_ZONE; i < CUBE_LAST; i++) {
		m_cubes[i].index = i;
		m_cubes[i].availbleFlag = true;
		m_cubes[i].position = m_platformStructure.blueExchangeZone.center;
	}

	//Note: center is not the actual object center. It is the position for the robot to arrive.

	memset(m_zone2ZonePath, 0, sizeof(m_zone2ZonePath));
	
	//TOP_CORRIDOR
	m_zone2ZonePath[TOP_CORRIDOR][BOTTOM_CORRIDOR] = {
		{
			{ LEFT_OF_RED_SWITCH , INVALID_ZONE },
			{ RIGHT_OF_RED_SWITCH , INVALID_ZONE },
			{ RIGHT_OF_SCALE , INVALID_ZONE },
			{ RIGHT_OF_BLUE_SWITCH , INVALID_ZONE }
		},
		4
	};
	m_zone2ZonePath[TOP_CORRIDOR][LEFT_OF_RED_SWITCH] = {
		{
			{ INVALID_ZONE , INVALID_ZONE },
			{ RIGHT_OF_RED_SWITCH , BOTTOM_CORRIDOR },
			{ RIGHT_OF_SCALE , BOTTOM_CORRIDOR },
			{ RIGHT_OF_BLUE_SWITCH , BOTTOM_CORRIDOR }
		},
		4
	};
	m_zone2ZonePath[TOP_CORRIDOR][RIGHT_OF_RED_SWITCH] = {
		{
			{ LEFT_OF_RED_SWITCH , BOTTOM_CORRIDOR },
			{ INVALID_ZONE , INVALID_ZONE },
			{ RIGHT_OF_SCALE , BOTTOM_CORRIDOR },
			{ RIGHT_OF_BLUE_SWITCH , BOTTOM_CORRIDOR }
		},
		4
	};
	m_zone2ZonePath[TOP_CORRIDOR][RIGHT_OF_SCALE] = {
		{
			{ LEFT_OF_RED_SWITCH , BOTTOM_CORRIDOR },
			{ RIGHT_OF_RED_SWITCH , BOTTOM_CORRIDOR },
			{ INVALID_ZONE , INVALID_ZONE },
			{ RIGHT_OF_BLUE_SWITCH , BOTTOM_CORRIDOR }
		},
		4
	};
	m_zone2ZonePath[TOP_CORRIDOR][RIGHT_OF_BLUE_SWITCH] = {
		{
			{ LEFT_OF_RED_SWITCH , BOTTOM_CORRIDOR },
			{ RIGHT_OF_RED_SWITCH , BOTTOM_CORRIDOR },
			{ RIGHT_OF_SCALE , BOTTOM_CORRIDOR },
			{ INVALID_ZONE , INVALID_ZONE }
		},
		4
	};

	//BOTTOM_CORRIDOR
	m_zone2ZonePath[BOTTOM_CORRIDOR][TOP_CORRIDOR] = {
		{
			{ LEFT_OF_RED_SWITCH , INVALID_ZONE },
			{ RIGHT_OF_RED_SWITCH , INVALID_ZONE },
			{ RIGHT_OF_SCALE , INVALID_ZONE },
			{ RIGHT_OF_BLUE_SWITCH , INVALID_ZONE }
		},
		4
	};
	m_zone2ZonePath[BOTTOM_CORRIDOR][LEFT_OF_RED_SWITCH] = {
		{
			{ INVALID_ZONE , INVALID_ZONE },
			{ RIGHT_OF_RED_SWITCH , TOP_CORRIDOR },
			{ RIGHT_OF_SCALE , TOP_CORRIDOR },
			{ RIGHT_OF_BLUE_SWITCH , TOP_CORRIDOR }
		},
		4
	};
	m_zone2ZonePath[BOTTOM_CORRIDOR][RIGHT_OF_RED_SWITCH] = {
		{
			{ LEFT_OF_RED_SWITCH , TOP_CORRIDOR },
			{ INVALID_ZONE , INVALID_ZONE },
			{ RIGHT_OF_SCALE , TOP_CORRIDOR },
			{ RIGHT_OF_BLUE_SWITCH , TOP_CORRIDOR }
		},
		4
	};
	m_zone2ZonePath[BOTTOM_CORRIDOR][RIGHT_OF_SCALE] = {
		{
			{ LEFT_OF_RED_SWITCH , TOP_CORRIDOR },
			{ RIGHT_OF_RED_SWITCH , TOP_CORRIDOR },
			{ INVALID_ZONE , INVALID_ZONE },
			{ RIGHT_OF_BLUE_SWITCH , TOP_CORRIDOR }
		},
		4
	};
	m_zone2ZonePath[BOTTOM_CORRIDOR][RIGHT_OF_BLUE_SWITCH] = {
		{
			{ LEFT_OF_RED_SWITCH , TOP_CORRIDOR },
			{ RIGHT_OF_RED_SWITCH , TOP_CORRIDOR },
			{ RIGHT_OF_SCALE , TOP_CORRIDOR },
			{ INVALID_ZONE , INVALID_ZONE }
		},
		4
	};

	//LEFT_OF_RED_SWITCH
	m_zone2ZonePath[LEFT_OF_RED_SWITCH][TOP_CORRIDOR] = {
		{
			{ INVALID_ZONE , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , RIGHT_OF_RED_SWITCH },
			{ BOTTOM_CORRIDOR , RIGHT_OF_SCALE },
			{ BOTTOM_CORRIDOR , RIGHT_OF_BLUE_SWITCH }
		},
		4
	};
	m_zone2ZonePath[LEFT_OF_RED_SWITCH][BOTTOM_CORRIDOR] = {
		{
			{ INVALID_ZONE , INVALID_ZONE },
			{ TOP_CORRIDOR , RIGHT_OF_RED_SWITCH },
			{ TOP_CORRIDOR , RIGHT_OF_SCALE },
			{ TOP_CORRIDOR , RIGHT_OF_BLUE_SWITCH }
		},
		4
	};
	m_zone2ZonePath[LEFT_OF_RED_SWITCH][RIGHT_OF_RED_SWITCH] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	m_zone2ZonePath[LEFT_OF_RED_SWITCH][RIGHT_OF_SCALE] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	m_zone2ZonePath[LEFT_OF_RED_SWITCH][RIGHT_OF_BLUE_SWITCH] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	//RIGHT_OF_RED_SWITCH
	m_zone2ZonePath[RIGHT_OF_RED_SWITCH][TOP_CORRIDOR] = {
		{
			{ INVALID_ZONE , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , LEFT_OF_RED_SWITCH },
			{ BOTTOM_CORRIDOR , RIGHT_OF_SCALE },
			{ BOTTOM_CORRIDOR , RIGHT_OF_BLUE_SWITCH }
		},
		4
	};
	m_zone2ZonePath[RIGHT_OF_RED_SWITCH][BOTTOM_CORRIDOR] = {
		{
			{ INVALID_ZONE , INVALID_ZONE },
			{ TOP_CORRIDOR , LEFT_OF_RED_SWITCH },
			{ TOP_CORRIDOR , RIGHT_OF_SCALE },
			{ TOP_CORRIDOR , RIGHT_OF_BLUE_SWITCH }
		},
		4
	};
	m_zone2ZonePath[RIGHT_OF_RED_SWITCH][LEFT_OF_RED_SWITCH] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	m_zone2ZonePath[RIGHT_OF_RED_SWITCH][RIGHT_OF_SCALE] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	m_zone2ZonePath[RIGHT_OF_RED_SWITCH][RIGHT_OF_BLUE_SWITCH] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	//RIGHT_OF_SCALE
	m_zone2ZonePath[RIGHT_OF_SCALE][TOP_CORRIDOR] = {
		{
			{ INVALID_ZONE , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , RIGHT_OF_RED_SWITCH },
			{ BOTTOM_CORRIDOR , LEFT_OF_RED_SWITCH },
			{ BOTTOM_CORRIDOR , RIGHT_OF_BLUE_SWITCH }
		},
		4
	};
	m_zone2ZonePath[RIGHT_OF_SCALE][BOTTOM_CORRIDOR] = {
		{
			{ INVALID_ZONE , INVALID_ZONE },
			{ TOP_CORRIDOR , RIGHT_OF_RED_SWITCH },
			{ TOP_CORRIDOR , LEFT_OF_RED_SWITCH },
			{ TOP_CORRIDOR , RIGHT_OF_BLUE_SWITCH }
		},
		4
	};
	m_zone2ZonePath[RIGHT_OF_SCALE][RIGHT_OF_RED_SWITCH] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	m_zone2ZonePath[RIGHT_OF_SCALE][LEFT_OF_RED_SWITCH] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	m_zone2ZonePath[RIGHT_OF_SCALE][RIGHT_OF_BLUE_SWITCH] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	//RIGHT_OF_BLUE_SWITCH
	m_zone2ZonePath[RIGHT_OF_BLUE_SWITCH][TOP_CORRIDOR] = {
		{
			{ INVALID_ZONE , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , RIGHT_OF_RED_SWITCH },
			{ BOTTOM_CORRIDOR , LEFT_OF_RED_SWITCH },
			{ BOTTOM_CORRIDOR , RIGHT_OF_SCALE }
		},
		4
	};
	m_zone2ZonePath[RIGHT_OF_BLUE_SWITCH][BOTTOM_CORRIDOR] = {
		{
			{ INVALID_ZONE , INVALID_ZONE },
			{ TOP_CORRIDOR , RIGHT_OF_RED_SWITCH },
			{ TOP_CORRIDOR , LEFT_OF_RED_SWITCH },
			{ TOP_CORRIDOR , RIGHT_OF_SCALE }
		},
		4
	};
	m_zone2ZonePath[RIGHT_OF_BLUE_SWITCH][RIGHT_OF_RED_SWITCH] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	m_zone2ZonePath[RIGHT_OF_BLUE_SWITCH][LEFT_OF_RED_SWITCH] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
	m_zone2ZonePath[RIGHT_OF_BLUE_SWITCH][RIGHT_OF_SCALE] = {
		{
			{ TOP_CORRIDOR , INVALID_ZONE },
			{ BOTTOM_CORRIDOR , INVALID_ZONE },
		},
		2
	};
}



platform::~platform()
{
	if ((m_pRedRobots[0] == NULL) ||
		(m_pRedRobots[1] == NULL) ||
		(m_pRedRobots[2] == NULL) ||
		(m_pBlueRobots[0] == NULL) ||
		(m_pBlueRobots[1] == NULL) ||
		(m_pBlueRobots[2] == NULL)) {
		printf("ERROR, allocate robots failed\n");
		return;
	}
	else {
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
			delete m_pRedRobots[i];
			delete m_pBlueRobots[i];
		}
	}

}
//Robot object ID is from 18 to 23, should convert these IDs to an enum, JWJW
void platform::configRedRobots(const robotConfigurationType config1In[NUMBER_OF_ROBOTS])
{
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_pRedRobots[i]->setConfiguration(&config1In[i], this, i);
		m_pRedRobots[i]->setAllianceType(ALLIANCE_RED);
	}
	//set robot initial positions
	m_pRedRobots[0]->setPosition(config1In[0].sizeX / 2, config1In[0].sizeY * 2, 18);
	m_pRedRobots[1]->setPosition(config1In[1].sizeX / 2, (264 + 48 * 2) / 2, 19);
	m_pRedRobots[2]->setPosition(config1In[0].sizeX / 2, (264 + 48 * 2) - config1In[0].sizeY * 2, 20);
}

void platform::configBlueRobots(const robotConfigurationType config1In[NUMBER_OF_ROBOTS])
{
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_pBlueRobots[i]->setConfiguration(&config1In[i], this, i);
		m_pBlueRobots[i]->setAllianceType(ALLIANCE_BLUE);
	}
	//set robot initial positions
	m_pBlueRobots[0]->setPosition((288 * 2 + 72) - config1In[0].sizeX / 2, config1In[0].sizeY * 2, 21);
	m_pBlueRobots[1]->setPosition((288 * 2 + 72) - config1In[1].sizeX / 2, (264 + 48 * 2) / 2, 22);
	m_pBlueRobots[2]->setPosition((288 * 2 + 72) - config1In[0].sizeX / 2, (264 + 48 * 2) - config1In[0].sizeY * 2, 23);
}

void platform::getFinalScore(int *pRedScoreOut, int *pBlueScoreOut)
{
	if (CLIMB_END_TIME > m_timeInSec) {
		updateScore(CLIMB_END_TIME - m_lastScoreUpdateTime);
	}

	*pRedScoreOut = (int) floor(getRedScore() + 0.5);
	*pBlueScoreOut = (int) floor(getBlueScore() + 0.5);

	if (*pRedScoreOut > *pBlueScoreOut) {
		m_redRank += 2;
	}
	else if (*pRedScoreOut == *pBlueScoreOut) {
		m_redRank++;
		m_blueRank++;
	}
	else {
		m_blueRank += 2;
	}

	return;
}

bool platform::isRobotLifted(allianceType allianceIn, int robotIdxIn)
{
	if (allianceIn == ALLIANCE_RED) {
		return m_state.redLiftFlag[robotIdxIn];
	}
	else {
		return m_state.blueLiftFlag[robotIdxIn];
	}
}

bool platform::hasPendingAction(int robotIndexIn, allianceType allianceIn)
{
	robot **pRobots;
	const pendingActionType *pPlannedAction;

	if (allianceIn == ALLIANCE_RED) {
		pRobots = m_pRedRobots;
	}
	else {
		pRobots = m_pBlueRobots;
	}

	pPlannedAction = pRobots[robotIndexIn]->getPlannedAction();
	if (pPlannedAction->actionType != INVALID_ACTION) {
		return true;
	}
	else {
		return false;
	}
}

void platform::forceRobotAction(const pendingActionType *pPlannedActionIn, coordinateType startPosIn, int cubeIdxIn,
	allianceType allianceIn, int robotIdxIn, int indexIn)
{
	static int debugCounter = 0;
	robot **pRobots;
	coordinateType currentPos = getRobotPos(allianceIn, robotIdxIn);
	bool currentHasCubeFlag = getRobotHasCubeFlag(allianceIn, robotIdxIn);

	if (currentHasCubeFlag) {
		if (cubeIdxIn == INVALID_IDX) {
			printf("ERROR, robot %d should not have a cube\n", robotIdxIn);
		}
	}
	else {
		if (cubeIdxIn != INVALID_IDX) {
			printf("ERROR, robot %d should have a cube\n", robotIdxIn);
		}
	}
	//Note: Because of different execution order between game platform and display platform
	//      one robot may pick up different cube on different platform. But, has cube flag should be the same.

	if ((abs(currentPos.x - startPosIn.x) >= 5) || (abs(currentPos.y - startPosIn.y) >= 5)) {
		printf("ERROR, robot %d position out of sync\n", robotIdxIn);
	}

	if (allianceIn == ALLIANCE_RED) {
		pRobots = m_pRedRobots;
	}
	else {
		pRobots = m_pBlueRobots;
	}

	pRobots[robotIdxIn]->forceAction(pPlannedActionIn, startPosIn, m_timeInSec, indexIn);
	debugCounter++;
}

coordinateType platform::getRobotPos(allianceType allianceIn, int robotIdxIn) const
{
	const robot *const* pRobots;
	const rectangleObjectType *pPos;

	if (allianceIn == ALLIANCE_RED) {
		pRobots = m_pRedRobots;
	}
	else {
		pRobots = m_pBlueRobots;
	}
	pPos = pRobots[robotIdxIn]->getPosition();
	return pPos->center;
}

bool platform::getRobotHasCubeFlag(allianceType allianceIn, int robotIdxIn) const
{
	const robot *const *pRobots;

	if (allianceIn == ALLIANCE_RED) {
		pRobots = m_pRedRobots;
	}
	else {
		pRobots = m_pBlueRobots;
	}
	return pRobots[robotIdxIn]->hasCube();
}

int platform::getRobotCubeIdx(allianceType allianceIn, int robotIdxIn) const
{
	const robot *const *pRobots;

	if (allianceIn == ALLIANCE_RED) {
		pRobots = m_pRedRobots;
	}
	else {
		pRobots = m_pBlueRobots;
	}
	return pRobots[robotIdxIn]->getCubeIdx();
}

const pendingActionType *platform::getRobotAction(allianceType allianceIn, int robotIdxIn) const
{
	const robot *const* pRobots;

	if (allianceIn == ALLIANCE_RED) {
		pRobots = m_pRedRobots;
	}
	else {
		pRobots = m_pBlueRobots;
	}
	return pRobots[robotIdxIn]->getPlannedAction();
}

int platform::setRobotAction(searchActionType *pActionListInOut, allianceType allianceIn, int indexIn)
{
	int rvalue;
	robot *const * pRobots;
	int robotIdx = pActionListInOut->robotIndex;

	if (allianceIn == ALLIANCE_RED) {
		pRobots = m_pRedRobots;
	}
	else {
		pRobots = m_pBlueRobots;
	}

	rvalue = pRobots[robotIdx]->takeAction(pActionListInOut->actionType, pActionListInOut->actionDonePos, m_timeInSec, indexIn);
	//update the projected start and finished time
	pActionListInOut->startTime = m_timeInSec;
	pActionListInOut->projectedFinishTime = pRobots[robotIdx]->getPlannedActionFinishTime();
	//Note: pActionListInOut only has estimated start and stop time. After moves of other robots, the
	//      start and finish time may change.

	return rvalue;
}

bool platform::hasPendingActions(void)
{
	bool hasPendingActionFlag = false;

	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		if (hasPendingAction(i, ALLIANCE_RED)) {
			hasPendingActionFlag = true;
			break;
		}
		if (hasPendingAction(i, ALLIANCE_BLUE)) {
			hasPendingActionFlag = true;
			break;
		}
	}

	return hasPendingActionFlag;
}

double platform::getEarliestStopTime(void)
{
	double earliestFinishTime;

	earliestFinishTime = CLIMB_END_TIME + 1;
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		if (hasPendingAction(i, ALLIANCE_RED)) {
			if (earliestFinishTime > m_pRedRobots[i]->getNextStopTime()) {
				earliestFinishTime = m_pRedRobots[i]->getNextStopTime();
			}
		}
		if (hasPendingAction(i, ALLIANCE_BLUE)) {
			if (earliestFinishTime > m_pBlueRobots[i]->getNextStopTime()) {
				earliestFinishTime = m_pBlueRobots[i]->getNextStopTime();
			}
		}
	}
	return earliestFinishTime;
}

int platform::commitAction(double nextTimeIn, int indexIn, allianceType activeAllianceIn)
{
	const rectangleObjectType *pRobotPos;
	int updateActionResult = 0;
	actionResultType actionResult;
	double earliestFinishTime;
	const pendingActionType *pPlannetAction;
	actionTypeType actionType;
	allianceType passiveAlliance;
	bool actionCommitFlag;

	if (m_timeInSec >= CLIMB_END_TIME) {
		return updateActionResult; //game is over, do nothing
		//Note: assume that all robots are trying to climb at the last 30 sec.
	}

	passiveAlliance = INVALID_ALLIANCE;
	if (activeAllianceIn == ALLIANCE_RED) {
		passiveAlliance = ALLIANCE_BLUE;
	}
	else if (activeAllianceIn == ALLIANCE_BLUE) {
		passiveAlliance = ALLIANCE_RED;
	}

	earliestFinishTime = nextTimeIn;
	if (earliestFinishTime >= CLIMB_END_TIME) {
		//skip the action and only update the game score
		updateScore(CLIMB_END_TIME - m_lastScoreUpdateTime);
		return updateActionResult;
	}

	actionCommitFlag = false;

	//run action of each robot
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		pPlannetAction = m_pRedRobots[i]->getPlannedAction();
		actionType = pPlannetAction->actionType;
		if (actionType != INVALID_ACTION) {
			actionResult = m_pRedRobots[i]->moveToNextTime(earliestFinishTime);

			pRobotPos = m_pRedRobots[i]->getPosition();
			if ((pRobotPos->center.x + pRobotPos->sizeX / 2 >= m_platformStructure.redAutoLine)
				&& (earliestFinishTime <= AUTONOMOUS_END_TIME)
				&& (!m_state.redCrossAutoFlag[i])) {

				m_state.redCrossAutoFlag[i] = true;
				m_redScore += 5;
				m_state.redAutonomousLineCount++;
			}

			switch(actionResult) {
			case ACTION_TIME_OUT:
				//give up the action and do nothing
				logAction(actionType, earliestFinishTime, i, indexIn, false);
				break;
			case ACTION_DONE:
				updateActionResult = updateOneAction(actionType, earliestFinishTime, i, ALLIANCE_RED, indexIn);
				actionCommitFlag = true;
				//action done log is printed inside updateOneAction()
				break;
			case ACTION_IN_PROGRESS:
				//check if action passed due time
				if (m_pRedRobots[i]->getPlannedActionFinishTime() <= earliestFinishTime) {
					logAction(actionType, earliestFinishTime, i, indexIn, false);
					updateActionResult = -1;
				}
				break;
			case ACTION_FAILED:
			case ACTION_START_ERROR:
			default:
				logAction(actionType, earliestFinishTime, i, indexIn, false);
				updateActionResult = -1;
				break;
			}

			if (updateActionResult != 0) {
				if (passiveAlliance == ALLIANCE_RED) {
					printf("ERROR, passive red alliance error is not expected\n");
				}
				return updateActionResult;
			}
		}

		pPlannetAction = m_pBlueRobots[i]->getPlannedAction();
		actionType = pPlannetAction->actionType;
		if (actionType != INVALID_ACTION) {
			actionResult = m_pBlueRobots[i]->moveToNextTime(earliestFinishTime);

			pRobotPos = m_pBlueRobots[i]->getPosition();
			if ((pRobotPos->center.x - pRobotPos->sizeX / 2 <= m_platformStructure.blueAutoLine)
				&& (earliestFinishTime <= AUTONOMOUS_END_TIME)
				&& (!m_state.blueCrossAutoFlag[i])) {

				m_state.blueCrossAutoFlag[i] = true;
				m_blueScore += 5;
				m_state.blueAutonomousLineCount++;
			}

			switch (actionResult) {
			case ACTION_TIME_OUT:
				//give up the action and do nothing
				logAction(actionType, earliestFinishTime, i, indexIn, false);
				break;
			case ACTION_DONE:
				updateActionResult = updateOneAction(actionType, earliestFinishTime, i, ALLIANCE_BLUE, indexIn);
				actionCommitFlag = true;
				//action done log is printed inside updateOneAction()
				break;
			case ACTION_IN_PROGRESS:
				//check if action passed due time
				if (m_pBlueRobots[i]->getPlannedActionFinishTime() <= earliestFinishTime) {
					logAction(actionType, earliestFinishTime, i, indexIn, false);
					updateActionResult = -1;
				}
				break;
			case ACTION_FAILED:
			case ACTION_START_ERROR:
			default:
				logAction(actionType, earliestFinishTime, i, indexIn, false);
				updateActionResult = -1;
				break;
			}

			if (updateActionResult != 0) {
				if (passiveAlliance == ALLIANCE_BLUE) {
					printf("ERROR, passive blue alliance error is not expected\n");
				}
				return updateActionResult;
			}
		}
	}

	if (!actionCommitFlag) {
		//just update the score without action
		updateOneAction(RED_ACTION_NONE, earliestFinishTime, 0, ALLIANCE_RED, indexIn);
	}

	//success
	m_timeInSec = earliestFinishTime;
	m_debugCounter++;
	return updateActionResult;
}

int platform::updateOneAction(actionTypeType actionIn, double timeIn, int robotIndexIn, allianceType allianceIn, int indexIn)
{
	int liftRebotCount = 0;

	switch (actionIn) {
	case CUBE_RED_OFFENSE_SWITCH:
		m_state.switchRed_RedBlockCount++;
		if (!m_isDisplayPlatform) {
			m_redScore += CUBE_REWARD_SCORE;
		}
		break;
	case CUBE_RED_DEFENSE_SWITCH:
		m_state.switchBlue_RedBlockCount++;
		if (!m_isDisplayPlatform) {
			m_redScore += CUBE_REWARD_SCORE;
		}
		break;
	case CUBE_RED_SCALE:
		m_state.scaleRedBlockCount++;
		if (!m_isDisplayPlatform) {
			m_redScore += CUBE_REWARD_SCORE;
		}
		break;
	case CUBE_RED_FORCE_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.forceRedBlockCount++;
		if (m_state.forceRedBlockCount > 3) {
			m_state.forceRedBlockCount = 3;
		}
		else {
			m_redScore += 5;
		}
		break;
	case CUBE_RED_BOOST_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.boostRedBlockCount++;
		if (m_state.boostRedBlockCount > 3) {
			m_state.boostRedBlockCount = 3;
		}
		else {
			m_redScore += 5;
		}
		break;
	case CUBE_RED_LIFT_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.liftRedBlockCount++;
		if (m_state.liftRedBlockCount > 3) {
			m_state.liftRedBlockCount = 3;
		}
		else {
			m_redScore += 5;
		}

		if ((m_state.liftRedBlockCount == 3) &&
			(liftRebotCount < NUMBER_OF_ROBOTS)) {

			if (m_state.redLiftButton == BUTTON_NOT_PUSH) {
				m_state.redLiftButton = BUTTON_PUSH;
				m_state.liftRedButtonPushBlockCount = m_state.liftRedBlockCount;
			}
		}

		break;
	case PUSH_RED_FORCE_BUTTON:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		if (m_state.forceRedBlockCount <= 2) {
			return -1;
		}
		if (m_state.redForceButton == BUTTON_NOT_PUSH) {
			m_state.redForceButton = BUTTON_PUSH;
			m_state.redForceButtonTime = 0;
			m_state.forceRedButtonPushBlockCount = m_state.forceRedBlockCount;
		}
		else {
			return -1;
		}
		break;
	case PUSH_RED_BOOST_BUTTON:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		if (m_state.boostRedBlockCount <= 2) {
			return -1;
		}
		if (m_state.redBoostButton == BUTTON_NOT_PUSH) {
			m_state.redBoostButton = BUTTON_PUSH;
			m_state.redBoostButtonTime = 0;
			m_state.boostRedButtonPushBlockCount = m_state.boostRedBlockCount;
		}
		else {
			return -1;
		}
	break;
	case LIFT_ONE_RED_ROBOT:
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
			liftRebotCount += (m_state.redLiftFlag[i] == true);

		//lift button plus lift robots are enough
		if ((liftRebotCount == NUMBER_OF_ROBOTS - 1) &&
			(m_state.redLiftButton != BUTTON_NOT_PUSH)) {
			return -1;
		}

		if ((m_state.redLiftFlag[robotIndexIn]) ||
			(timeIn < COMPETITION_END_TIME)) {
			return -1;
		}
		m_liftRedRobotIndex = robotIndexIn;
		break;
		////////////////////
	case CUBE_BLUE_OFFENSE_SWITCH:
		m_state.switchBlue_BlueBlockCount++;
		if (!m_isDisplayPlatform) {
			m_redScore += CUBE_REWARD_SCORE;
		}
		break;
	case CUBE_BLUE_DEFENSE_SWITCH:
		m_state.switchRed_BlueBlockCount++;
		if (!m_isDisplayPlatform) {
			m_redScore += CUBE_REWARD_SCORE;
		}
		break;
	case CUBE_BLUE_SCALE:
		m_state.scaleBlueBlockCount++;
		if (!m_isDisplayPlatform) {
			m_redScore += CUBE_REWARD_SCORE;
		}
		break;
	case CUBE_BLUE_FORCE_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.forceBlueBlockCount++;
		if (m_state.forceBlueBlockCount > 3) {
			m_state.forceBlueBlockCount = 3;
		}
		else {
			m_blueScore += 5;
		}
		break;
	case CUBE_BLUE_BOOST_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.boostBlueBlockCount++;
		if (m_state.boostBlueBlockCount > 3) {
			m_state.boostBlueBlockCount = 3;
		}
		else {
			m_blueScore += 5;
		}
		break;
	case CUBE_BLUE_LIFT_VAULT:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		m_state.liftBlueBlockCount++;
		if (m_state.liftBlueBlockCount > 3) {
			m_state.liftBlueBlockCount = 3;
		}
		else {
			m_blueScore += 5;
		}

		if ((m_state.liftBlueBlockCount >= 3) &&
			(liftRebotCount < NUMBER_OF_ROBOTS)) {

			if (m_state.blueLiftButton == BUTTON_NOT_PUSH) {
				m_state.blueLiftButton = BUTTON_PUSH;
				m_state.liftBlueButtonPushBlockCount = m_state.liftBlueBlockCount;
			}
		}
		break;
	case PUSH_BLUE_FORCE_BUTTON:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		if (m_state.forceBlueBlockCount <= 2) {
			return -1;
		}
		if (m_state.blueForceButton == BUTTON_NOT_PUSH) {
			m_state.blueForceButton = BUTTON_PUSH;
			m_state.blueForceButtonTime = 0;
			m_state.forceBlueButtonPushBlockCount = m_state.forceBlueBlockCount;
		}
		else {
			return -1;
		}
		break;
	case PUSH_BLUE_BOOST_BUTTON:
		if (timeIn <= AUTONOMOUS_END_TIME) {
			return -1;
		}
		if (m_state.boostBlueBlockCount <= 2) {
			return -1;
		}
		if (m_state.blueBoostButton == BUTTON_NOT_PUSH) {
			m_state.blueBoostButton = BUTTON_PUSH;
			m_state.blueBoostButtonTime = 0;
			m_state.boostBlueButtonPushBlockCount = m_state.boostBlueBlockCount;
		}
		else {
			return -1;
		}
		break;
	case LIFT_ONE_BLUE_ROBOT:
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
			liftRebotCount += (m_state.blueLiftFlag[i] == true);

		if ((liftRebotCount == NUMBER_OF_ROBOTS - 1) &&
			(m_state.blueLiftButton != BUTTON_NOT_PUSH)) {
			return -1;
		}

		if ((m_state.blueLiftFlag[robotIndexIn]) ||
		    (timeIn < COMPETITION_END_TIME)) {
			return -1;
		}
		m_liftBlueRobotIndex = robotIndexIn;
		break;
	case RED_ACTION_NONE:
	case BLUE_ACTION_NONE:
	case RED_ROBOT_GOTO_POS:
	case BLUE_ROBOT_GOTO_POS:
		break; //no action, just time pass
	default:
		printf("ERROR: invalid action %d\n", actionIn);
		break;
	}

	if (!m_state.autonomousRankingDoneFlag && m_lastScoreUpdateTime >= AUTONOMOUS_END_TIME) {
		if ((m_state.redAutonomousLineCount >= 3) && (m_state.switchRedOwner == ALLIANCE_RED)) {
			m_redRank++;
		}
		if ((m_state.blueAutonomousLineCount >= 3) && (m_state.switchBlueOwner == ALLIANCE_BLUE)) {
			m_blueRank++;
		}

		m_state.autonomousRankingDoneFlag = true;
	}

	if (timeIn < m_timeInSec) {
		printf("Error, time reverse, from %3.2f to %3.2f\n", m_timeInSec, timeIn);
	}
	updateScore(timeIn - m_lastScoreUpdateTime);

	if ((actionIn != RED_ACTION_NONE) && (actionIn != BLUE_ACTION_NONE)) {
		logAction(actionIn, timeIn, robotIndexIn, indexIn, true);
	}
	return 0;
}

int platform::isGameTimeOver(void)
{
	if (m_timeInSec >= CLIMB_END_TIME) {
		return 1;
	}
	else {
		return 0;
	}
}

double platform::updateScaleSwitchScore(double secondsIn, int vaultForceBlockCountIn, int vaultBoostBlockCountIn, int balanceBlockDifferenceIn,
	vaultButtonStateType forceVaultButtonIn, vaultButtonStateType boostVaultButtonIn,
	int vaultBlockSelectionIn, ownerShipType newOnershipIn, ownerShipType *pOwnershipInOut)
{
	double scores = 0;
	bool ownershipChangeFlag;

	ownershipChangeFlag = false;
	if ((balanceBlockDifferenceIn >= MIN_BLOCK_DIFFERENCE_TO_SCORE) ||
		((forceVaultButtonIn >= BUTTON_PUSH_0SEC) && (forceVaultButtonIn <= BUTTON_PUSH_9SEC) &&
		((vaultForceBlockCountIn == vaultBlockSelectionIn) || (vaultForceBlockCountIn == 3)))) {

		//get one more point for balance change
		if (balanceBlockDifferenceIn >= MIN_BLOCK_DIFFERENCE_TO_SCORE) {
			if (*pOwnershipInOut != newOnershipIn) {
				*pOwnershipInOut = newOnershipIn;
				ownershipChangeFlag = true;
				scores++;
				if (m_timeInSec + secondsIn < AUTONOMOUS_END_TIME) {
					scores++; //in autonomous session, ownership change worth 2 points
				}
			}
		}

		if ((*pOwnershipInOut == newOnershipIn) &&
			(ownershipChangeFlag == false)) { //ownership change happens before

			scores += secondsIn;
			if (m_timeInSec + secondsIn < AUTONOMOUS_END_TIME) {
				scores += secondsIn;
			}
			else if (m_timeInSec < AUTONOMOUS_END_TIME) {
				scores += AUTONOMOUS_END_TIME - m_timeInSec;
			}
			//Note: because button push is not allowed in autonomous session,
			//      the logic above doesn't distinguish the ownership caused by cubes
			//      or force button.
		}
		else if ((forceVaultButtonIn >= BUTTON_PUSH_0SEC) && (forceVaultButtonIn <= BUTTON_PUSH_9SEC) &&
			     ((vaultForceBlockCountIn == vaultBlockSelectionIn) || (vaultForceBlockCountIn == 3))) {
			//it must be force button on
			//only get scores within force button 10 seconds
			if (forceVaultButtonIn + secondsIn <= BUTTON_PUSH_OVER_10SEC) {
				scores += secondsIn;
			}
			else {
				scores += BUTTON_PUSH_OVER_10SEC - forceVaultButtonIn;
			}
		}

		if ((boostVaultButtonIn >= BUTTON_PUSH_0SEC) && (boostVaultButtonIn < BUTTON_PUSH_OVER_10SEC) &&
			((vaultBoostBlockCountIn == vaultBlockSelectionIn) || (vaultBoostBlockCountIn == 3))) {

			//double the score
			if ((*pOwnershipInOut == newOnershipIn) && (ownershipChangeFlag == false)) {
				if (boostVaultButtonIn + secondsIn <= BUTTON_PUSH_OVER_10SEC) {
					scores += secondsIn;
				}
				else {
					scores += BUTTON_PUSH_OVER_10SEC - boostVaultButtonIn;
				}
			}
			else {
				//by force button
				if ((forceVaultButtonIn + secondsIn <= BUTTON_PUSH_OVER_10SEC) &&
					(boostVaultButtonIn + secondsIn <= BUTTON_PUSH_OVER_10SEC)) {
					scores += secondsIn;
				}
				else {
					if (forceVaultButtonIn >= boostVaultButtonIn) {
						scores += BUTTON_PUSH_OVER_10SEC - forceVaultButtonIn;
					}
					else {
						scores += BUTTON_PUSH_OVER_10SEC - boostVaultButtonIn;
					}
				}
			}
		}
	}
	else {
		if (*pOwnershipInOut == newOnershipIn) {
			*pOwnershipInOut = OWNED_BY_NONE;
		}
	}

	return scores;
}

void platform::updateScore(double secondsPassedIn)
{
	int liftRebotCount;

	vaultButtonStateType previousRedForceButton = BUTTON_NOT_PUSH;
	vaultButtonStateType previousBlueForceButton = BUTTON_NOT_PUSH;
	vaultButtonStateType previousRedBoostButton = BUTTON_NOT_PUSH;
	vaultButtonStateType previousBlueBoostButton = BUTTON_NOT_PUSH;

	//force button could be queued if the opponent is in force
	if ((m_state.blueForceButton >= BUTTON_PUSH) && (m_state.blueForceButton != BUTTON_PUSH_OVER_10SEC)) {
		//skip update of red force and boost button, wait blue force done
	}
	else {
		if ((m_state.redForceButton != BUTTON_NOT_PUSH) &&
			((m_state.redForceButton != BUTTON_PUSH_OVER_10SEC))) {

			previousRedForceButton = m_state.redForceButton;  //red force is enabled
			m_state.redForceButtonTime += secondsPassedIn;
			if (m_state.redForceButtonTime >= 1) {
				m_state.redForceButton = (vaultButtonStateType)(m_state.redForceButton + ROUNDING_METHOD(m_state.redForceButtonTime));
				m_state.redForceButtonTime -= ROUNDING_METHOD(m_state.redForceButtonTime);
				if (m_state.redForceButton > BUTTON_PUSH_OVER_10SEC) {
					m_state.redForceButton = BUTTON_PUSH_OVER_10SEC;
				}
			}
		}

		if ((m_state.redBoostButton != BUTTON_NOT_PUSH) &&
			((m_state.redBoostButton != BUTTON_PUSH_OVER_10SEC))) {

			previousRedBoostButton = m_state.redBoostButton; //red boost is enabled
			m_state.redBoostButtonTime += secondsPassedIn;
			if (m_state.redBoostButtonTime >= 1) {
				m_state.redBoostButton = (vaultButtonStateType)(m_state.redBoostButton + ROUNDING_METHOD(m_state.redBoostButtonTime));
				m_state.redBoostButtonTime -= ROUNDING_METHOD(m_state.redBoostButtonTime);
				if (m_state.redBoostButton > BUTTON_PUSH_OVER_10SEC) {
					m_state.redBoostButton = BUTTON_PUSH_OVER_10SEC;
				}
			}
		}
	}
	if ((m_state.redForceButton >= BUTTON_PUSH) && (m_state.redForceButton != BUTTON_PUSH_OVER_10SEC)) {
		//skip update of blue force and boost button, wait red force button done
	}
	else {
		if ((m_state.blueForceButton != BUTTON_NOT_PUSH) &&
			((m_state.blueForceButton != BUTTON_PUSH_OVER_10SEC))) {

			previousBlueForceButton = m_state.blueForceButton;
			m_state.blueForceButtonTime += secondsPassedIn;
			if (m_state.blueForceButtonTime >= 1) {
				m_state.blueForceButton = (vaultButtonStateType)(m_state.blueForceButton + ROUNDING_METHOD(m_state.blueForceButtonTime));
				m_state.blueForceButtonTime -= ROUNDING_METHOD(m_state.blueForceButtonTime);
				if (m_state.blueForceButton > BUTTON_PUSH_OVER_10SEC) {
					m_state.blueForceButton = BUTTON_PUSH_OVER_10SEC;
				}
			}
		}

		if ((m_state.blueBoostButton != BUTTON_NOT_PUSH) &&
			((m_state.blueBoostButton != BUTTON_PUSH_OVER_10SEC))) {

			previousBlueBoostButton = m_state.blueBoostButton;
			m_state.blueBoostButtonTime += secondsPassedIn;
			if (m_state.blueBoostButtonTime >= 1) {
				m_state.blueBoostButton = (vaultButtonStateType)(m_state.blueBoostButton + ROUNDING_METHOD(m_state.blueBoostButtonTime));
				m_state.blueBoostButtonTime -= ROUNDING_METHOD(m_state.blueBoostButtonTime);
				if (m_state.blueBoostButton > BUTTON_PUSH_OVER_10SEC) {
					m_state.blueBoostButton = BUTTON_PUSH_OVER_10SEC;
				}
			}
		}
	}

	m_redScore += updateScaleSwitchScore(secondsPassedIn, m_state.forceRedButtonPushBlockCount, m_state.boostRedButtonPushBlockCount,
		m_state.scaleRedBlockCount - m_state.scaleBlueBlockCount,
		previousRedForceButton, previousRedBoostButton, 2, OWNED_BY_RED, &m_state.scaleOwner);

	m_blueScore += updateScaleSwitchScore(secondsPassedIn, m_state.forceBlueButtonPushBlockCount, m_state.boostBlueButtonPushBlockCount,
		m_state.scaleBlueBlockCount - m_state.scaleRedBlockCount,
		previousBlueForceButton, previousBlueBoostButton, 2, OWNED_BY_BLUE, &m_state.scaleOwner);

	m_redScore += updateScaleSwitchScore(secondsPassedIn, m_state.forceRedButtonPushBlockCount, m_state.boostRedButtonPushBlockCount,
		m_state.switchRed_RedBlockCount - m_state.switchRed_BlueBlockCount,
		previousRedForceButton, previousRedBoostButton, 2, OWNED_BY_RED, &m_state.switchRedOwner);

	m_blueScore += updateScaleSwitchScore(secondsPassedIn, m_state.forceBlueButtonPushBlockCount, m_state.boostBlueButtonPushBlockCount,
		m_state.switchBlue_BlueBlockCount - m_state.switchBlue_RedBlockCount,
		previousBlueForceButton, previousBlueBoostButton, 2, OWNED_BY_BLUE, &m_state.switchBlueOwner);

	if (m_timeInSec >= COMPETITION_END_TIME) {
		//lift button is only allowed at the last 30 seconds

		//red lifting
		if ((m_state.redLiftButton == BUTTON_PUSH) && (m_state.liftRedBlockCount >= 3)) {
			m_redScore += 30;
			m_state.redLiftButton = BUTTON_PUSH_OVER_10SEC;
		}
		else if(m_state.liftRedButtonPushBlockCount < 3) {
			m_state.redLiftButton = BUTTON_NOT_PUSH; //less than 3 cubes, button push ignored
		}

		if (m_liftRedRobotIndex < NUMBER_OF_ROBOTS) {
			m_state.redLiftFlag[m_liftRedRobotIndex] = true;
			m_redScore += 30;
			m_liftRedRobotIndex = INVALID_IDX;
		}

		liftRebotCount = (m_state.redLiftButton == BUTTON_PUSH_OVER_10SEC);
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
			liftRebotCount += (m_state.redLiftFlag[i] == true);

		if ((liftRebotCount == NUMBER_OF_ROBOTS) && (!m_state.allRedRobotsLiftFlag)) {
			m_redRank += 1;
			m_state.allRedRobotsLiftFlag = true;
		}

		//blue lifting
		if ((m_state.blueLiftButton == BUTTON_PUSH) && (m_state.liftBlueBlockCount >= 3)) {
			m_blueScore += 30;
			m_state.blueLiftButton = BUTTON_PUSH_OVER_10SEC;
		}
		else if (m_state.liftBlueButtonPushBlockCount < 3) {
			m_state.blueLiftButton = BUTTON_NOT_PUSH; //less than 3 cubes, button push ignored
		}

		if (m_liftBlueRobotIndex < NUMBER_OF_ROBOTS) {
			m_state.blueLiftFlag[m_liftBlueRobotIndex] = true;
			m_blueScore += 30;
			m_liftBlueRobotIndex = INVALID_IDX;
		}

		liftRebotCount = (m_state.blueLiftButton == BUTTON_PUSH_OVER_10SEC);
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++)
			liftRebotCount += (m_state.blueLiftFlag[i] == true);

		if ((liftRebotCount == NUMBER_OF_ROBOTS) && (!m_state.allBlueRobotsLiftFlag)) {
			m_blueRank += 1;
			m_state.allBlueRobotsLiftFlag = true;
		}
	}
	else {
		//otherwise, the push signal is ignored and reset the button state
		m_state.redLiftButton = BUTTON_NOT_PUSH;
		m_state.blueLiftButton = BUTTON_NOT_PUSH;
	}

	m_lastScoreUpdateTime += secondsPassedIn;
}

void platform::logAction(actionTypeType actionIn, double timeIn, int robotIndexIn, int indexIn, bool successFlagIn)
{
	if (m_pLogFIle == NULL) {
		return;
	}
	switch (actionIn) {
	case CUBE_RED_OFFENSE_SWITCH:
	case CUBE_RED_DEFENSE_SWITCH:
	case CUBE_RED_SCALE:
	case CUBE_RED_FORCE_VAULT:
	case CUBE_RED_BOOST_VAULT:
	case CUBE_RED_LIFT_VAULT:
	case PUSH_RED_FORCE_BUTTON:
	case PUSH_RED_BOOST_BUTTON:
	case LIFT_ONE_RED_ROBOT:
	case RED_ACTION_NONE:
	case RED_ROBOT_GOTO_POS:
		fprintf(m_pLogFIle, "%d. Time %3.2f (sec), Red alliance robot[%d]: ", indexIn, timeIn, robotIndexIn);
		break;
	case CUBE_BLUE_OFFENSE_SWITCH:
	case CUBE_BLUE_DEFENSE_SWITCH:
	case CUBE_BLUE_SCALE:
	case CUBE_BLUE_FORCE_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
	case CUBE_BLUE_LIFT_VAULT:
	case PUSH_BLUE_FORCE_BUTTON:
	case PUSH_BLUE_BOOST_BUTTON:
	case LIFT_ONE_BLUE_ROBOT:
	case BLUE_ACTION_NONE:
	case BLUE_ROBOT_GOTO_POS:
		fprintf(m_pLogFIle, "%d. Time %3.2f (sec), Blue alliance robot[%d]: ", indexIn, timeIn, robotIndexIn);
		break;
	default:
		fprintf(m_pLogFIle, "%d. Time %3.2f (sec), unknown alliance (ERROR)", indexIn, timeIn);
		break;
	}
	
	switch (actionIn) {
	case CUBE_RED_OFFENSE_SWITCH:
		fprintf(m_pLogFIle, "put a cube on red side switch");
		break;
	case CUBE_BLUE_OFFENSE_SWITCH:
		fprintf(m_pLogFIle, "put a cube on blue side switch");
		break;
	case CUBE_RED_DEFENSE_SWITCH:
		fprintf(m_pLogFIle, "put a cube on blue side switch");
		break;
	case CUBE_BLUE_DEFENSE_SWITCH:
		fprintf(m_pLogFIle, "put a cube on red side switch");
		break;
	case CUBE_RED_SCALE:
	case CUBE_BLUE_SCALE:
		fprintf(m_pLogFIle, "put a cube on the scale");
		break;
	case CUBE_RED_FORCE_VAULT:
	case CUBE_BLUE_FORCE_VAULT:
		fprintf(m_pLogFIle, "add a cube to force vault");
		break;
	case CUBE_RED_BOOST_VAULT:
	case CUBE_BLUE_BOOST_VAULT:
		fprintf(m_pLogFIle, "add a cube to boost vault");
		break;
	case CUBE_RED_LIFT_VAULT:
	case CUBE_BLUE_LIFT_VAULT:
		fprintf(m_pLogFIle, "add a cube to lift vault");
		break;
	case PUSH_RED_FORCE_BUTTON:
	case PUSH_BLUE_FORCE_BUTTON:
		fprintf(m_pLogFIle, "push force vault button");
		break;
	case PUSH_RED_BOOST_BUTTON:
	case PUSH_BLUE_BOOST_BUTTON:
		fprintf(m_pLogFIle, "push boost vault button");
		break;
	case LIFT_ONE_RED_ROBOT:
	case LIFT_ONE_BLUE_ROBOT:
		fprintf(m_pLogFIle, "lift robot on platform");
		break;
	case RED_ACTION_NONE:
	case BLUE_ACTION_NONE:
	case RED_ROBOT_GOTO_POS:
	case BLUE_ROBOT_GOTO_POS:
		fprintf(m_pLogFIle, "no action");
		break;
	default:
		fprintf(m_pLogFIle, "unknown action (ERROR)");
		break;
	}

	if (successFlagIn) {
		fprintf(m_pLogFIle, ", red score %d, blue score %d\n", (int) getRedScore(), (int) getBlueScore());
	}
	else {
		fprintf(m_pLogFIle, " (failed)\n");
	}
}

void platform::logFinalRanking(void)
{
	if (m_pLogFIle == NULL) {
		return;
	}
	fprintf(m_pLogFIle, "================= The final ranking is (blue ranking %d, red ranking %d) ================\n",
		 m_blueRank, m_redRank);
	fflush(m_pLogFIle);
}

double platform::findOneCube(double shortestPathIn, int startSearchIdxIn, int endSearchIdxIn, bool isAllCubeSameFlag,
	const rectangleObjectType *pMovingObjectIn, double robotTurnDelayIn, double robotCubeDelayIn,
	double xOffsetIn, double yOffsetIn,	cubeStateType **pCubeOut, robotPathType *pPathOut)
{
	double shortestPath = shortestPathIn;
	robotPathType nextPath;
	coordinateType cubePos;
	bool findACubeFlag;

	//search for cubs by the switch
	for (int i = startSearchIdxIn; i < endSearchIdxIn; i++) {
		if (!m_cubes[i].availbleFlag) {
			continue;
		}
		cubePos = m_cubes[i].position;
		nextPath.numberOfTurns = 0;
		findACubeFlag = false;

		if (findAvailablePath(pMovingObjectIn, m_cubes[i].position, true, robotTurnDelayIn, &nextPath)) {
			findACubeFlag = true;
		}

		if ((isAllCubeSameFlag) && (findACubeFlag == false)) {
			//try other offset
			cubePos = m_cubes[i].position;
			cubePos.x += xOffsetIn;
			nextPath.numberOfTurns = 0;
			if (findAvailablePath(pMovingObjectIn, cubePos, true, robotTurnDelayIn, &nextPath)) {
				findACubeFlag = true;
			}
		}

		if ((isAllCubeSameFlag) && (findACubeFlag == false)) {
			//try other offset
			cubePos = m_cubes[i].position;
			cubePos.y += yOffsetIn;
			nextPath.numberOfTurns = 0;
			if (findAvailablePath(pMovingObjectIn, cubePos, true, robotTurnDelayIn, &nextPath)) {
				findACubeFlag = true;
			}
		}

		if ((isAllCubeSameFlag) && (findACubeFlag == false)) {
			//try other offset
			cubePos = m_cubes[i].position;
			cubePos.y -= yOffsetIn;
			nextPath.numberOfTurns = 0;
			if (findAvailablePath(pMovingObjectIn, cubePos, true, robotTurnDelayIn, &nextPath)) {
				findACubeFlag = true;
			}
		}

		if(findACubeFlag) {
			if (nextPath.numberOfTurns != 0) {
				nextPath.turnPointDelay[nextPath.numberOfTurns - 1] += robotCubeDelayIn;
			}
			else {
				nextPath.firstTurnDelay += robotCubeDelayIn;
			}
			if (shortestPath > nextPath.totalDistance) {
				memcpy(pPathOut, &nextPath, sizeof(robotPathType));
				*pCubeOut = &m_cubes[i];
				shortestPath = nextPath.totalDistance;
			}
		}
		//It is a simple implementation, the number of turns is not counted.

		if (isAllCubeSameFlag) {
			break; //all the cubes are the same, try one of them is enough
		}
	}

	return shortestPath;
}

const cubeSearchRangeType redCubeSearchRange[] =
{	{ CUBE_BY_RED_SWITCH , CUBE_BY_RED_POWER_ZONE, false },  //cubes by switch
	{ CUBE_BY_RED_POWER_ZONE , CUBE_BY_BLUE_POWER_ZONE, true },  //CUBE BY POWER ZONE
	{ CUBE_BY_RED_EXCHANGE_ZONE , CUBE_BY_BLUE_EXCHANGE_ZONE, true }   //CUBE BY EXCHANGE ZONE
};

const cubeSearchRangeType blueCubeSearchRange[] =
{	{ CUBE_BY_RED_SWITCH , CUBE_BY_RED_POWER_ZONE, false },  //cubes by switch
	{ CUBE_BY_BLUE_POWER_ZONE , CUBE_BY_RED_EXCHANGE_ZONE, true },  //CUBE BY POWER ZONE
	{ CUBE_BY_BLUE_EXCHANGE_ZONE , CUBE_LAST, true }   //CUBE BY EXCHANGE ZONE
};

const double DISTANCE_OUT_OF_RANGE = 1000000;
bool platform::findTheClosestCube(const rectangleObjectType *pMovingObjectIn, allianceType allianceIn, 
	double robotTurnDelayIn, double robotCubeDelayIn,
	cubeStateType **pCubeOut, robotPathType *pPathOut)
{
	double shortestPath = DISTANCE_OUT_OF_RANGE;
	int cubeListSize = sizeof(redCubeSearchRange) / sizeof(cubeSearchRangeType);

	pPathOut->numberOfTurns = 0;

	//search power zone and exchange zone
	if (allianceIn == ALLIANCE_RED) {
		for (int i = 0; i < cubeListSize - 1; i++) {
			shortestPath = findOneCube(shortestPath, redCubeSearchRange[i].startIdx, redCubeSearchRange[i].endIdx, 
				redCubeSearchRange[i].allCubeSameFlag, pMovingObjectIn, robotTurnDelayIn, robotCubeDelayIn, -40, 35, pCubeOut, pPathOut);

		}
		//exchange zone
		if (m_timeInSec >= AUTONOMOUS_END_TIME) {
			shortestPath = findOneCube(shortestPath, redCubeSearchRange[cubeListSize - 1].startIdx, redCubeSearchRange[cubeListSize - 1].endIdx,
				redCubeSearchRange[cubeListSize - 1].allCubeSameFlag, pMovingObjectIn, robotTurnDelayIn, robotCubeDelayIn, 40, 35, pCubeOut, pPathOut);
		}
	}
	else {
		for (int i = 0; i < cubeListSize - 1; i++) {
			shortestPath = findOneCube(shortestPath, blueCubeSearchRange[i].startIdx, blueCubeSearchRange[i].endIdx,
				blueCubeSearchRange[i].allCubeSameFlag, pMovingObjectIn, robotTurnDelayIn, robotCubeDelayIn, 40, 35, pCubeOut, pPathOut);

		}
		//exchange zone
		if (m_timeInSec >= AUTONOMOUS_END_TIME) {
			shortestPath = findOneCube(shortestPath, blueCubeSearchRange[cubeListSize - 1].startIdx, blueCubeSearchRange[cubeListSize - 1].endIdx,
				blueCubeSearchRange[cubeListSize - 1].allCubeSameFlag, pMovingObjectIn, robotTurnDelayIn, robotCubeDelayIn, -40, 35, pCubeOut, pPathOut);
		}
	}

	if (shortestPath >= DISTANCE_OUT_OF_RANGE) {
		return false;
	}

	if (pPathOut->numberOfTurns == 0) {
		//pick up cube without moving
		pPathOut->numberOfTurns = 1;
		pPathOut->totalDistance = 0;
		pPathOut->initialSpeed = 0;
		pPathOut->turnPoints[0] = pMovingObjectIn->center;
		pPathOut->turnPointDelay[0] = 0; //no delay after turn point is arrived
		pPathOut->pickUpCubeIndex = -1;
		pPathOut->firstTurnDelay = robotCubeDelayIn;
	}
	else {
		//the last turn point is the point to pick up a cube
		pPathOut->pickUpCubeIndex = pPathOut->numberOfTurns - 1;
	}
	return true;
}

int platform::pickUpCube(coordinateType positionIn, allianceType allianceIn)
{
	if (allianceIn == ALLIANCE_RED) {
		for (int i = 0; i < sizeof(redCubeSearchRange) / sizeof(cubeSearchRangeType); i++) {
			for (int j = redCubeSearchRange[i].startIdx; j < redCubeSearchRange[i].endIdx; j++) {
				if (tryPickOneCube(positionIn, m_cubes[j].position, m_cubes[j].availbleFlag)) {
					m_cubes[j].availbleFlag = false;
					return j;
				}
			}
		}
	}
	else {
		for (int i = 0; i < sizeof(blueCubeSearchRange) / sizeof(cubeSearchRangeType); i++) {
			for (int j = blueCubeSearchRange[i].startIdx; j < blueCubeSearchRange[i].endIdx; j++) {
				if (tryPickOneCube(positionIn, m_cubes[j].position, m_cubes[j].availbleFlag)) {
					m_cubes[j].availbleFlag = false;
					return j;
				}
			}
		}
	}
	return INVALID_IDX;
}

bool  platform::tryPickOneCube(coordinateType robotPosIn, coordinateType cubePosIn, bool cubeAvailableFlagIn)
{
	double distance;

	distance = calculateDistance(robotPosIn, cubePosIn);

	if ((distance <= PICK_UP_CUBE_DISTANCE) && (cubeAvailableFlagIn)) {
		return true;
	}
	else {
		return false;
	}
}


bool platform::findAvailablePath(const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn,
	bool isTargetACubeIn, double robotTurnDelayIn, robotPathType *pPathOut)
{
	//the path selection algorithm is,
	//a) find out the robot zone and the destination point zone, then, work on following cases,
	//     1. construct candidate zone paths in order.
	//     2. for each zone path, find out every turn points. Stop on the first non-blocking path.
    //b) Fill up the delay on each turn point

	bool isSearchDoneFlag;
	bool isSearchFailedFlag;
	rectangleObjectType movingObject;
	robotMoveZoneType firstZone;
	robotMoveZoneType startZone;
	robotMoveZoneType targetZone;
	robotMoveZoneType connectionZoneIdx;
	coordinateType connectionPoint;

	//initialize output
	pPathOut->initialSpeed = 0;
	pPathOut->firstTurnDelay = 0;
	pPathOut->numberOfTurns = 0;

	//check if it is arrived 
	memcpy(&movingObject, pMovingObjectIn, sizeof(movingObject));
	if ((movingObject.center.x == endPointIn.x) && (movingObject.center.y == endPointIn.y)) {
		pPathOut->numberOfTurns = 0;
		pPathOut->totalDistance = 0;
		return true;
	}

	//else, search for zone path
	firstZone = getObjectZone(&movingObject);
	targetZone = getPointZone(endPointIn);

	if (firstZone == targetZone) {
		//in the same zone
		if (true == foundPathWithinZone(&movingObject, endPointIn, &m_platformStructure.zones[firstZone], pPathOut)) {
			if (pPathOut->numberOfTurns < 1) {
				printf("ERROR, found an empty path\n");
				return false;
			}
			for (int i = 0; i < pPathOut->numberOfTurns - 1; i++) {
				pPathOut->turnPointDelay[i] = robotTurnDelayIn;
			}
			//the caller will set the delay of the last turn point, set it to 0 for now.
			pPathOut->turnPointDelay[pPathOut->numberOfTurns - 1] = 0;
			return true;
		}
		else {
			return false;
		}
	}

	//else, not in the same zone
	sortZoneConnections(movingObject.center, firstZone, endPointIn, targetZone, &m_sortedZonePath);

	for (int i = 0; i < m_sortedZonePath.pathNUmber; i++) {
		isSearchDoneFlag = false;
		isSearchFailedFlag = false;
		movingObject.center = pMovingObjectIn->center;
		startZone = firstZone;
		pPathOut->numberOfTurns = 0;

		for (int j = 0; j < NUMBER_OF_ZONES_ON_PATH; j++) {
			//to the connect point of the same zone
			connectionZoneIdx = m_sortedZonePath.path[i].connections[j];
			if (connectionZoneIdx == INVALID_ZONE) {
				//replace it with the target zone
				connectionZoneIdx = targetZone;
				connectionPoint = m_platformStructure.zones[startZone].connectionPoints[connectionZoneIdx];
				isSearchDoneFlag = true;
			}
			else {
				connectionPoint = m_platformStructure.zones[startZone].connectionPoints[connectionZoneIdx];
			}

			if (true == foundPathWithinZone(&movingObject, connectionPoint, &m_platformStructure.zones[startZone], pPathOut)) {
				movingObject.center = connectionPoint;
				startZone = connectionZoneIdx;

				if (isSearchDoneFlag) {
					break;
				}
			}
			else {
				//search failed, give up tis path
				isSearchFailedFlag = true;
				isSearchDoneFlag = true;
				break;
			}
		}

		//search the last point
		if (!isSearchFailedFlag) {
			if (true != foundPathWithinZone(&movingObject, endPointIn, &m_platformStructure.zones[targetZone], pPathOut)) {
				isSearchFailedFlag = true;
			}
		}

		if ((!isSearchFailedFlag) && (isSearchDoneFlag)) {
			break; //success
		}
	}

	if ((!isSearchFailedFlag) && (isSearchDoneFlag)) {
		//success
		for (int i = 0; i < pPathOut->numberOfTurns - 1; i++) {
			pPathOut->turnPointDelay[i] = robotTurnDelayIn;
		}
		//the caller will set the delay of the last turn point, set it to 0 for now.
		pPathOut->turnPointDelay[pPathOut->numberOfTurns - 1] = 0;
		return true;
	}
	else {
		//failed
		return false;
	}
}

bool platform::collisionWithAllOtherObjects(const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn,
	                                        const rectangleObjectType **pCollisionObjectOut)
{
	const robotStateType *pRobotState;
	const rectangleObjectType *pRobotPosition;
	const pendingActionType *pPlannedAction;
	rectangleObjectType futurePosition;

	*pCollisionObjectOut = NULL;

	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		pRobotState =  m_pRedRobots[i]->getState();
		pRobotPosition = &pRobotState->pos;
		pPlannedAction = m_pRedRobots[i]->getPlannedAction();

		for (int j = 0; j < 2; j++) {
			if (pRobotPosition->objectId != pMovingObjectIn->objectId) {
				if (collisionDectection(pRobotPosition, pMovingObjectIn, endPointIn)) {

					if ((pMovingObjectIn->center.x == endPointIn.x) && (pMovingObjectIn->center.x == endPointIn.x)) {
						//single point collision check
						*pCollisionObjectOut = pRobotPosition;
						return true;
					}
					//else, moving path collision check
					if (false == collisionDectection(pRobotPosition, pMovingObjectIn, pMovingObjectIn->center)) {
						*pCollisionObjectOut = pRobotPosition;
						return true;
					}
					//else, collision already happens at the start position, ignore it
				}

				//future collision test
				memcpy(&futurePosition, pRobotPosition, sizeof(futurePosition));
				if ((pPlannedAction->path.pickUpCubeIndex != INVALID_IDX) && (pPlannedAction->path.pickUpCubeIndex != -1)) {

					futurePosition.center = pPlannedAction->path.turnPoints[pPlannedAction->path.pickUpCubeIndex];
					if (collisionDectection(&futurePosition, pMovingObjectIn, endPointIn)) {
						*pCollisionObjectOut = pRobotPosition;
						return true;
					}
				}
				if (pPlannedAction->path.numberOfTurns > 0) {
					futurePosition.center = pPlannedAction->path.turnPoints[pPlannedAction->path.numberOfTurns-1];
					if (collisionDectection(&futurePosition, pMovingObjectIn, endPointIn)) {
						*pCollisionObjectOut = pRobotPosition;
						return true;
					}
				}
			}

			pRobotState = m_pBlueRobots[i]->getState();
			pRobotPosition = &pRobotState->pos;
			pPlannedAction = m_pBlueRobots[i]->getPlannedAction();
		}
	}
	return false;
}

bool platform::pointInRectangle(coordinateType aIn, coordinateType bIn, coordinateType cIn, coordinateType dIn,
	double pointXIn, double pointYIn) const
{
	//Note: points A, B, C, D are clockwise
	if ((pointXIn - aIn.x) * (bIn.y - aIn.y) > (pointYIn - aIn.y) * (bIn.x - aIn.x)) {
		if ((pointXIn - dIn.x) * (cIn.y - dIn.y) > (pointYIn - dIn.y) * (cIn.x - dIn.x)) {
			return false;
		}
	}
	else {
		if ((pointXIn - dIn.x) * (cIn.y - dIn.y) < (pointYIn - dIn.y) * (cIn.x - dIn.x)) {
			return false;
		}
	}

	if ((pointXIn - aIn.x) * (dIn.y - aIn.y) > (pointYIn - aIn.y) * (dIn.x - aIn.x)) {
		if ((pointXIn - bIn.x) * (cIn.y - bIn.y) > (pointYIn - bIn.y) * (cIn.x - bIn.x)) {
			return false;
		}
	}
	else {
		if ((pointXIn - bIn.x) * (cIn.y - bIn.y) < (pointYIn - bIn.y) * (cIn.x - bIn.x)) {
			return false;
		}
	}

	return true;
}

bool platform::collisionDectection(const rectangleObjectType *pStillObjectIn, 
	const rectangleObjectType *pMovingObjectIn, coordinateType endPointIn)
{
	rectangleObjectType endObject;
	double sLeftX, sTopY, sBottomY, sRightX;
	coordinateType a, b, c, d;

	memcpy(&endObject, pMovingObjectIn, sizeof(endObject));
	endObject.center = endPointIn;

	//find the still object rectangle
	sLeftX = pStillObjectIn->center.x - pStillObjectIn->sizeX / 2;
	sRightX = pStillObjectIn->center.x + pStillObjectIn->sizeX / 2;
	sTopY = pStillObjectIn->center.y + pStillObjectIn->sizeY / 2;
	sBottomY = pStillObjectIn->center.y - pStillObjectIn->sizeY / 2;

	//test if two rectangles overlap on 4 corners
	if (pointInObject(pMovingObjectIn, sLeftX, sTopY)) {
		return true;
	}
	if (pointInObject(pMovingObjectIn, sRightX, sTopY)) {
		return true;
	}
	if (pointInObject(pMovingObjectIn, sLeftX, sBottomY)) {
		return true;
	}
	if (pointInObject(pMovingObjectIn, sRightX, sBottomY)) {
		return true;
	}

	if (pointInObject(&endObject, sLeftX, sTopY)) {
		return true;
	}
	if (pointInObject(&endObject, sRightX, sTopY)) {
		return true;
	}
	if (pointInObject(&endObject, sLeftX, sBottomY)) {
		return true;
	}
	if (pointInObject(&endObject, sRightX, sBottomY)) {
		return true;
	}

	if ((endObject.center.x - pMovingObjectIn->center.x) * (endObject.center.y - pMovingObjectIn->center.y) >= 0) {
		a.x = endObject.center.x + endObject.sizeX / 2;
		a.y = endObject.center.y - endObject.sizeY / 2;

		d.x = endObject.center.x - endObject.sizeX / 2;
		d.y = endObject.center.y + endObject.sizeY / 2;

		b.x = pMovingObjectIn->center.x + pMovingObjectIn->sizeX / 2;
		b.y = pMovingObjectIn->center.y - pMovingObjectIn->sizeY / 2;

		c.x = pMovingObjectIn->center.x - pMovingObjectIn->sizeX / 2;
		c.y = pMovingObjectIn->center.y + pMovingObjectIn->sizeY / 2;
	}
	else {
		a.x = endObject.center.x + endObject.sizeX / 2;
		a.y = endObject.center.y + endObject.sizeY / 2;

		d.x = endObject.center.x - endObject.sizeX / 2;
		d.y = endObject.center.y - endObject.sizeY / 2;

		b.x = pMovingObjectIn->center.x + pMovingObjectIn->sizeX / 2;
		b.y = pMovingObjectIn->center.y + pMovingObjectIn->sizeY / 2;

		c.x = pMovingObjectIn->center.x - pMovingObjectIn->sizeX / 2;
		c.y = pMovingObjectIn->center.y - pMovingObjectIn->sizeY / 2;
	}

	if (pointInRectangle(a, b, c, d, sLeftX, sTopY)) {
		return true;
	}
	if (pointInRectangle(a, b, c, d, sRightX, sTopY)) {
		return true;
	}
	if (pointInRectangle(a, b, c, d, sLeftX, sBottomY)) {
		return true;
	}
	if (pointInRectangle(a, b, c, d, sRightX, sBottomY)) {
		return true;
	}

	return false;
}

double platform::estimatePathDistance(coordinateType startIn, robotMoveZoneType startZoneIn, 
	coordinateType targetIn, robotMoveZoneType targetZoneIn, int pathIdIn)
{
	coordinateType connectionPoint;
	coordinateType startPoint;
	robotMoveZoneType connectionZoneIdx;
	robotMoveZoneType startZoneIdx;
	int idx;
	double distance = 0;

	if (startZoneIn == targetZoneIn) {
		//in the same zone
		return calculateDistance(startIn, targetIn);
	}

	if (pathIdIn >= m_zone2ZonePath[startZoneIn][targetZoneIn].pathNUmber) {
		printf("ERROR, invalid path index\n");
		return 0;
	}

	idx = 0;
	distance = 0;
	connectionZoneIdx = m_zone2ZonePath[startZoneIn][targetZoneIn].path[pathIdIn].connections[idx];
	startZoneIdx = startZoneIn;
	startPoint = startIn;

	while (connectionZoneIdx != INVALID_ZONE) {
		connectionPoint = m_platformStructure.zones[startZoneIdx].connectionPoints[connectionZoneIdx];
		distance += calculateDistance(connectionPoint, startPoint);

		startPoint = connectionPoint;
		startZoneIdx = connectionZoneIdx;
		idx++;
		connectionZoneIdx = m_zone2ZonePath[startZoneIn][targetZoneIn].path[pathIdIn].connections[idx];
	}

	//from the last connect zone to the target zone
	distance += calculateDistance(m_platformStructure.zones[startZoneIdx].connectionPoints[targetZoneIn], startPoint);
	distance += calculateDistance(m_platformStructure.zones[startZoneIdx].connectionPoints[targetZoneIn], targetIn);
	return distance;
}

void platform::sortZoneConnections(coordinateType startIn, robotMoveZoneType startZoneIn,
	coordinateType targetIn, robotMoveZoneType targetZoneIn, zonePathType *pPathListOut)
{
	int index;
	int pathIdx[MAXIMUM_PATH_NUMBER];
	double length;
	double distance[MAXIMUM_PATH_NUMBER];
	int pathNumber = m_zone2ZonePath[startZoneIn][targetZoneIn].pathNUmber;

	if (pathNumber < 2) {
		printf("ERROR, the smallest path number is 2\n");
	}

	for (int i = 0; i < pathNumber; i++) {
		pathIdx[i] = i;
		distance[i] = estimatePathDistance(startIn, startZoneIn, targetIn, targetZoneIn, i);
	}

	for (int i = 0; i < pathNumber; i++) {
		for (int j = 0; j < pathNumber - 1; j++) {
			if (distance[j] > distance[j + 1]) {
				index = pathIdx[j];
				pathIdx[j] = pathIdx[j + 1];
				pathIdx[j + 1] = index;

				length = distance[j];
				distance[j] = distance[j + 1];
				distance[j + 1] = length;
			}
		}
	}

	//copy to the output
	pPathListOut->pathNUmber = pathNumber;
	for (int i = 0; i < pathNumber; i++) {
		for (int j = 0; j < NUMBER_OF_ZONES_ON_PATH; j++) {
			pPathListOut->path[i].connections[j] = m_zone2ZonePath[startZoneIn][targetZoneIn].path[pathIdx[i]].connections[j];
		}
	}
}


bool platform::foundPathWithinZone(const rectangleObjectType *pRobotIn, coordinateType targetIn,
	const zoneType *pzoneIn, robotPathType *pPathInOut)
{
	rectangleObjectType startPos;
	rectangleObjectType searchPos;
	double shortestDistance;
	int workaroundPointIdx;
	bool collisionDetectedFlag;
	bool beforeWorkaroundCollision;
	bool afterWoraroundCollision;
	const rectangleObjectType *pCollisionObject;
	int turnNumber = pPathInOut->numberOfTurns;
	bool isSuccess;
	double distanceA, distanceB, distanceC;

	if (turnNumber >= MAX_TURNS_ON_PATH) {
		return false;
	}


	memcpy(&startPos, pRobotIn, sizeof(rectangleObjectType));
	memcpy(&searchPos, pRobotIn, sizeof(rectangleObjectType));

	isSuccess = false;

	if (turnNumber != 0) {
		//start at the last turn point
		startPos.center = pPathInOut->turnPoints[pPathInOut->numberOfTurns - 1];
	}

	//try the path
	collisionDetectedFlag = collisionWithAllOtherObjects(&startPos, targetIn, &pCollisionObject);

	if (collisionDetectedFlag) {

		if (pointInObject(pCollisionObject, targetIn.x, targetIn.y)) {
			//the target is covered by the object
			return false;
		}

		distanceA = calculateDistance(startPos.center, targetIn);
		shortestDistance = 100000;
		workaroundPointIdx = INVALID_IDX;
		searchPos.center = startPos.center;

		for (int i = 0; i < pzoneIn->numberOfWorkaroundPoints; i++) {

			distanceB = calculateDistance(startPos.center, pzoneIn->workaroundPoints[i]);
			distanceC = calculateDistance(targetIn, pzoneIn->workaroundPoints[i]);

			if ((distanceB >= distanceA) || (distanceC >= distanceA)) {
				//workaround point is not on the path to the target point
				continue;
			}

			beforeWorkaroundCollision = collisionWithAllOtherObjects(&startPos, pzoneIn->workaroundPoints[i], &pCollisionObject);
			searchPos.center = pzoneIn->workaroundPoints[i];
			afterWoraroundCollision = collisionWithAllOtherObjects(&searchPos, targetIn, &pCollisionObject);

			if (!beforeWorkaroundCollision && !afterWoraroundCollision) {
				//just turn point, not turn delay
				if (shortestDistance > distanceB + distanceC) {
					workaroundPointIdx = i;
					shortestDistance = distanceB + distanceC;
				}
			}
		}

		if(workaroundPointIdx != INVALID_IDX) {
			pPathInOut->turnPoints[turnNumber] = pzoneIn->workaroundPoints[workaroundPointIdx];
			turnNumber++;
			isSuccess = true;
		}
	}
	else {
		isSuccess = true;
	}

	if (isSuccess) {
		if (turnNumber >= MAX_TURNS_ON_PATH) {
			return false;
		}

		pPathInOut->turnPoints[turnNumber] = targetIn;
		turnNumber++;

		//success
		pPathInOut->numberOfTurns = turnNumber;
		return true;
	}
	else {
		return false;
	}
}


robotMoveZoneType platform::getObjectZone(const rectangleObjectType *pObjectIn)
{
	for (int i = 0; i < NUM_OF_ZONES; i++) {
		if (objectInZone(&m_platformStructure.zones[i].area, pObjectIn)) {
			return (robotMoveZoneType)i;
		}
	}

	printf("ERROR, the input robot is not in any zone\n");
	return TOP_CORRIDOR;
}

bool platform::objectInZone(const rectangleObjectType *pZoneIn, const rectangleObjectType *pObjectIn)
{
	int inZoneCount = 0;
	double leftX, topY, bottomY, rightX;

	leftX = pObjectIn->center.x - pObjectIn->sizeX / 2;
	rightX = pObjectIn->center.x + pObjectIn->sizeX / 2;
	topY = pObjectIn->center.y + pObjectIn->sizeY / 2;
	bottomY = pObjectIn->center.y - pObjectIn->sizeY / 2;

	if (pointInObject(pZoneIn, leftX, topY)) {
		inZoneCount++;
	}
	if (pointInObject(pZoneIn, rightX, topY)) {
		inZoneCount++;
	}
	if (pointInObject(pZoneIn, leftX, bottomY)) {
		inZoneCount++;
	}
	if (pointInObject(pZoneIn, rightX, bottomY)) {
		inZoneCount++;
	}

	//must have at least two points in the zone confirm that it is in.
	if (inZoneCount >= 2) {
		return true;
	}
	else {
		return false;
	}
}

