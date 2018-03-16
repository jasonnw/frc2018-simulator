

#include "os_wrapper.h"
#include "config.h"
#include "displayPlatform.h"

const double ORIGION_POINT_X = 10;
const double ORIGION_POINT_Y = 790;
const double PIXELS_PER_INCH = (double) 1.9;
const double FRAME_DELAY_IN_MS = 16;

#ifndef _MSC_VER
//missing definitions of MS visual studio unique code
typedef int errno_t;
#define sprintf_s	sprintf
static errno_t fopen_s(FILE** pFile, const char *filename, const char *mode)
{
	*pFile = fopen(filename, mode);
	if (*pFile == NULL) {
		return -1;
	}
	else {
		return 0;
	}
}
#endif


displayPlatform::displayPlatform()
{
	int errorCode;
	m_pQueue = new messageQueue <actionMessageType> (MESSAGE_QUEUE_DEPTH, &errorCode);
	m_isDisplayPlatform = true;
	m_wallColor = { 128, 128, 128 };


	if (m_pQueue != NULL) {
		if (errorCode != 0) {
			printf("ERROR, create message Q failed\n");
			delete m_pQueue;
			m_pQueue = NULL;
		}
		else {

			m_pPlatform = new Mat(PLATFORM_RESOLUSION_y, PLATFORM_RESOLUSION_X, CV_8UC3, Scalar(0, 0, 0));

			if (m_pPlatform == NULL) {
				printf("ERROR, create display platform failed\n");
				delete m_pQueue;
				m_pQueue = NULL;
			}
			else {
				drawField();
			}
		}
	}
}


displayPlatform::~displayPlatform()
{
	if (m_pQueue != NULL) {
		delete m_pQueue;
	}

	if (m_pPlatform != NULL) {
		delete m_pPlatform;
	}
}

int displayPlatform::sendAction(const actionMessageType *pActionIn)
{
	if (m_pQueue == NULL) {
		printf("ERROR, message queue is invalid\n");
		return -1;
	}

	//logAction(pActionIn->action.actionType, pActionIn->action.projectedFinishTime, pActionIn->action.robotIndex, pActionIn->actionIndex+1000, true);

	//block on message Q
	m_pQueue->send(pActionIn);
	return 0;
}

int displayPlatform::updatePlatform(int actionIndexIn)
{
	rectangleObjectType gameOverDisplay;

	actionMessageType message;
	bool commitMessageFlag;
	bool quitFlag;
	double earliestFinishTime;
	int blueScore, redScore;

	if (m_pQueue == NULL) {
		printf("ERROR, message queue is invalid\n");
		return -1;
	}

	quitFlag = false;
	do {
		commitMessageFlag = false;
		if (0 == m_pQueue->tryReceive(&message)) {
			//logAction(message.action.actionType, message.action.projectedFinishTime, message.action.robotIndex, message.actionIndex + 2000, true);

			commitMessageFlag = message.commitActionFlag;
			quitFlag = message.quitFlag;
			if ((!quitFlag) && (!commitMessageFlag)) {
				forceRobotAction(&message.action, message.startPos, message.cubeIdx, message.alliance, message.robotIdx, actionIndexIn);
			}
		}
		else {
			waitKey(10);
		}
	} while (!commitMessageFlag);

	if (quitFlag) {
		//finish all pending actions
		earliestFinishTime = getEarliestStopTime();
		while (hasPendingActions() && (!isGameTimeOver())) {
			earliestFinishTime = getEarliestStopTime();
			if (earliestFinishTime > CLIMB_END_TIME) {
				break;
			}
			playTotheNextTime(earliestFinishTime, actionIndexIn, FRAME_DELAY_IN_MS);
		}

		if (earliestFinishTime < CLIMB_END_TIME) {
			playTotheNextTime(CLIMB_END_TIME, actionIndexIn, FRAME_DELAY_IN_MS);
		}

		getFinalScore(&redScore, &blueScore);
		logFinalRanking();

		updateField();
		gameOverDisplay.center = { 70, 180 };
		drawString(&gameOverDisplay, "Game Over, Press Any Key", 3, { 128, 128, 128 });
		drawPlatform(0);

		return 1;
	}
	else {
		earliestFinishTime = getEarliestStopTime();
		playTotheNextTime(earliestFinishTime, actionIndexIn, FRAME_DELAY_IN_MS);
		return 0;
	}
}

void displayPlatform::playTotheNextTime(double nextTimeIn, int actionIndexIn, double frameDelayIn)
{
	double currentTime = getTime();
	double frameDelay = (double) (frameDelayIn / 1000.0);

	if (frameDelay == 0) {
		frameDelay = (double) (FRAME_DELAY_IN_MS / 1000);
	}

	for (double i = currentTime + frameDelay; i < nextTimeIn; i += frameDelay)
	{
		if (0 != commitAction(i, actionIndexIn, INVALID_ALLIANCE)) {
			printf("ERROR, cannot display invalid action\n");
		}
		updateField();
		drawPlatform((int) floor(frameDelay * 2000));
	}

	//last action
	if (0 != commitAction(nextTimeIn, actionIndexIn, INVALID_ALLIANCE)) {
		printf("ERROR, cannot display the last action \n");
	}
	updateField();
	drawPlatform((int)floor(frameDelay * 1000));
}

Point displayPlatform::coordinateToPoint(double xIn, double yIn)
{
	Point result;
	int x, y;

	x = (int) floor(ORIGION_POINT_X + (xIn * PIXELS_PER_INCH) + 0.5);
	y = (int) floor(ORIGION_POINT_Y - (yIn * PIXELS_PER_INCH) + 0.5);
	result = { x, y};
	return result;
}

void displayPlatform::drawPlatform(int delayIn)
{
	imshow("FRC 2018 Game Simulation v0.1", *m_pPlatform);

	waitKey(delayIn);
}

void displayPlatform::drawObject(const rectangleObjectType *pObjectIn, bool hightLightFlagIn = false)
{
	Point point1;
	Point point2;

	point1 = coordinateToPoint(pObjectIn->center.x + pObjectIn->sizeX / 2, pObjectIn->center.y + pObjectIn->sizeY / 2);
	point2 = coordinateToPoint(pObjectIn->center.x - pObjectIn->sizeX / 2, pObjectIn->center.y - pObjectIn->sizeY / 2);

	rectangle(*m_pPlatform, point1, point2, pObjectIn->color, CV_FILLED, 8, 0);

	if (hightLightFlagIn) {
		point1 = coordinateToPoint(pObjectIn->center.x + pObjectIn->sizeX / 2 + 2, pObjectIn->center.y + pObjectIn->sizeY / 2 + 2);
		point2 = coordinateToPoint(pObjectIn->center.x - pObjectIn->sizeX / 2 - 2, pObjectIn->center.y - pObjectIn->sizeY / 2 - 2);
		rectangle(*m_pPlatform, point1, point2, { 200, 200, 200 }, 4, 8, 0);
	}
}

void displayPlatform::drawInteger(const rectangleObjectType *pObjectIn, int numberIn, const char *strIn, double sizeIn, cv::Scalar colorIn = { 200, 200, 200 })
{
	Point point1;
	char robotIdxStr[128];

	sprintf_s(robotIdxStr, "%s%d", strIn, numberIn);
	point1 = coordinateToPoint(pObjectIn->center.x - 4, pObjectIn->center.y - 8);
	putText(*m_pPlatform, robotIdxStr, point1, FONT_HERSHEY_COMPLEX_SMALL, sizeIn, colorIn, 1, CV_AA);
}

void displayPlatform::drawString(const rectangleObjectType *pObjectIn, const char *strIn, double sizeIn, cv::Scalar colorIn = { 200, 200, 200 })
{
	Point point1;
	char robotIdxStr[128];

	sprintf_s(robotIdxStr, "%s", strIn);
	point1 = coordinateToPoint(pObjectIn->center.x - 4, pObjectIn->center.y - 8);
	putText(*m_pPlatform, robotIdxStr, point1, FONT_HERSHEY_COMPLEX_SMALL, sizeIn, colorIn, 1, CV_AA);
}

void displayPlatform::drawFloat(const rectangleObjectType *pObjectIn, double numberIn, const char *strIn, double sizeIn, cv::Scalar colorIn = { 200, 200, 200 })
{
	Point point1;
	char robotIdxStr[128];

	sprintf_s(robotIdxStr, "%s%3.1f", strIn, numberIn);
	point1 = coordinateToPoint(pObjectIn->center.x - 4, pObjectIn->center.y - 8);
	putText(*m_pPlatform, robotIdxStr, point1, FONT_HERSHEY_COMPLEX_SMALL, sizeIn, colorIn, 1, CV_AA);
}

void displayPlatform::drawRobot(const rectangleObjectType *pObjectIn, allianceType allianceIn, int robotIdxIn, bool hasCubeFlagIn)
{
	Point point1;
	Point point2;
	char robotIdxStr[4];
	cv::Scalar color(50, 20, 20);
	cv::Scalar cubeColor(20, 120, 20);
	bool isLifted = isRobotLifted(allianceIn, robotIdxIn);

	drawObject(pObjectIn, isLifted);
	sprintf_s(robotIdxStr, "%d", robotIdxIn);

	point1 = coordinateToPoint(pObjectIn->center.x - 4, pObjectIn->center.y - 15);

	putText(*m_pPlatform, robotIdxStr, point1, FONT_HERSHEY_COMPLEX_SMALL, 0.8, color, 1, CV_AA);

	if (hasCubeFlagIn) {
		point1 = coordinateToPoint(pObjectIn->center.x + 8, pObjectIn->center.y + 12);
		point2 = coordinateToPoint(pObjectIn->center.x - 8, pObjectIn->center.y - 4);

		rectangle(*m_pPlatform, point1, point2, cubeColor, CV_FILLED, 8, 0);
	}
}

void displayPlatform::drawCube(coordinateType positionIn, int indexIn)
{
	Point point1;
	Point point2;
	cv::Scalar cubeColor(20, 120, 20);

	point1 = coordinateToPoint(positionIn.x + 8, positionIn.y + 8);
	point2 = coordinateToPoint(positionIn.x - 8, positionIn.y - 8);

	if ((indexIn >= CUBE_BY_RED_SWITCH) && (indexIn < CUBE_BY_BLUE_SWITCH)) {
		point1.x -= 20;
		point2.x -= 20;
	}
	if ((indexIn >= CUBE_BY_BLUE_SWITCH) && (indexIn < CUBE_BY_RED_POWER_ZONE)) {
		point1.x += 20;
		point2.x += 20;
	}

	rectangle(*m_pPlatform, point1, point2, cubeColor, CV_FILLED, 8, 0);
}

void displayPlatform::drawField(void)
{
	Point lineStart;
	Point lineEnd;

	//four walls
	lineStart = coordinateToPoint(0, 0);
	lineEnd = coordinateToPoint(m_platformStructure.eastWall, 0);
	line(*m_pPlatform, lineStart, lineEnd, m_wallColor, 1, 8);

	lineStart = coordinateToPoint(0, 0);
	lineEnd = coordinateToPoint(0, m_platformStructure.northWall);
	line(*m_pPlatform, lineStart, lineEnd, m_wallColor, 1, 8);

	lineStart = coordinateToPoint(0, m_platformStructure.northWall);
	lineEnd = coordinateToPoint(m_platformStructure.eastWall, m_platformStructure.northWall);
	line(*m_pPlatform, lineStart, lineEnd, m_wallColor, 1, 8);

	lineStart = coordinateToPoint(m_platformStructure.eastWall, 0);
	lineEnd = coordinateToPoint(m_platformStructure.eastWall, m_platformStructure.northWall);
	line(*m_pPlatform, lineStart, lineEnd, m_wallColor, 1, 8);

	lineStart = coordinateToPoint(m_platformStructure.redAutoLine, 0);
	lineEnd = coordinateToPoint(m_platformStructure.redAutoLine, m_platformStructure.northWall);
	line(*m_pPlatform, lineStart, lineEnd, m_wallColor, 1, 8);

	lineStart = coordinateToPoint(m_platformStructure.blueAutoLine, 0);
	lineEnd = coordinateToPoint(m_platformStructure.blueAutoLine, m_platformStructure.northWall);
	line(*m_pPlatform, lineStart, lineEnd, m_wallColor, 1, 8);

	for (int i = RED_SWITCH_ZONE; i < NUM_STILL_STRUCTURE; i++) {
		drawObject(&m_platformStructure.structures[i]);
	}

	drawObject(&m_platformStructure.blueExchangeZone);
	drawObject(&m_platformStructure.blueLiftZone);
	//drawObject(&m_platformStructure.bluePlatformZone);
	drawObject(&m_platformStructure.bluePowerCubeZone);
	drawObject(&m_platformStructure.blueSwitchNorthPlate);
	drawObject(&m_platformStructure.blueSwitchSouthPlate);
	drawObject(&m_platformStructure.redExchangeZone);
	drawObject(&m_platformStructure.redLiftZone);
	//drawObject(&m_platformStructure.redPlatformZone);
	drawObject(&m_platformStructure.redPowerCubeZone);
	drawObject(&m_platformStructure.redSwitchNorthPlate);
	drawObject(&m_platformStructure.redSwitchSouthPlate);
	drawObject(&m_platformStructure.scaleNorthPlate);
	drawObject(&m_platformStructure.scaleSouthPlate);

	for (int i = CUBE_BY_RED_SWITCH; i < CUBE_BY_BLUE_SWITCH; i++) {
		if (m_cubes[i].availbleFlag) {
			drawCube(m_cubes[i].position, m_cubes[i].index);
		}
	}
	for (int i = CUBE_BY_BLUE_SWITCH; i < CUBE_BY_RED_POWER_ZONE; i++) {
		if (m_cubes[i].availbleFlag) {
			drawCube(m_cubes[i].position, m_cubes[i].index);
		}
	}
	for (int i = CUBE_BY_RED_POWER_ZONE; i < CUBE_BY_BLUE_POWER_ZONE; i++) {
		if (m_cubes[i].availbleFlag) {
			drawCube(m_cubes[i].position, m_cubes[i].index);
		}
	}
	for (int i = CUBE_BY_BLUE_POWER_ZONE; i < CUBE_BY_RED_EXCHANGE_ZONE; i++) {
		if (m_cubes[i].availbleFlag) {
			drawCube(m_cubes[i].position, m_cubes[i].index);
		}
	}
}

void displayPlatform::drawButton(coordinateType positionIn, int pushStateIn, const Scalar& colorIn, int radiusIn)
{
	Point center = coordinateToPoint(positionIn.x, positionIn.y);
	circle(*m_pPlatform, center, radiusIn, colorIn, pushStateIn);
}

void displayPlatform::updateField(void)
{
	rectangleObjectType blueScoreBoard;
	rectangleObjectType redScoreBoard;
	rectangleObjectType timeBoard;
	int buttonState;

	//erase all
	*m_pPlatform = Scalar(0, 0, 0);
	drawField();
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		drawRobot(m_pRedRobots[i]->getPosition(), ALLIANCE_RED, i, m_pRedRobots[i]->hasCube());
		drawRobot(m_pBlueRobots[i]->getPosition(), ALLIANCE_BLUE, i, m_pBlueRobots[i]->hasCube());
	}
	
	if (BLUE_NORTH_SWITCH_FLAG) {
		drawInteger(&m_platformStructure.blueSwitchNorthPlate, m_state.switchBlue_BlueBlockCount, "", 2.0);
		drawInteger(&m_platformStructure.blueSwitchSouthPlate, m_state.switchBlue_RedBlockCount, "", 2.0);
	}
	else {
		drawInteger(&m_platformStructure.blueSwitchNorthPlate, m_state.switchBlue_RedBlockCount, "", 2.0);
		drawInteger(&m_platformStructure.blueSwitchSouthPlate, m_state.switchBlue_BlueBlockCount, "", 2.0);
	}

	if (RED_NORTH_SWITCH_FLAG) {
		drawInteger(&m_platformStructure.redSwitchNorthPlate, m_state.switchRed_RedBlockCount, "", 2.0);
		drawInteger(&m_platformStructure.redSwitchSouthPlate, m_state.switchRed_BlueBlockCount, "", 2.0);
	}
	else {
		drawInteger(&m_platformStructure.redSwitchNorthPlate, m_state.switchRed_BlueBlockCount, "", 2.0);
		drawInteger(&m_platformStructure.redSwitchSouthPlate, m_state.switchRed_RedBlockCount, "", 2.0);
	}

	if (RED_NORTH_SCALE_FLAG) {
		drawInteger(&m_platformStructure.scaleNorthPlate, m_state.scaleRedBlockCount, "", 2.0);
		drawInteger(&m_platformStructure.scaleSouthPlate, m_state.scaleBlueBlockCount, "", 2.0);
	}
	else {
		drawInteger(&m_platformStructure.scaleNorthPlate, m_state.scaleBlueBlockCount, "", 2.0);
		drawInteger(&m_platformStructure.scaleSouthPlate, m_state.scaleRedBlockCount, "", 2.0);
	}

	//display rank
	blueScoreBoard.sizeX = 80;
	blueScoreBoard.sizeY = 60;
	redScoreBoard.sizeX = 80;
	redScoreBoard.sizeY = 60;

	blueScoreBoard.center = { 580, 390 };
	redScoreBoard.center = { 30, 390 };
	drawInteger(&blueScoreBoard, getBlueRanking(), "Rank: ", 1, { 200, 30, 30 });
	drawInteger(&redScoreBoard, getRedRanking(), "Rank: ", 1, { 30, 30, 200 });

	//display time
	timeBoard.center = { 260, 400 };
	timeBoard.sizeX = 80;
	timeBoard.sizeY = 60;
	drawFloat(&timeBoard, getTime(), "Time: ", 1.5);

	//display scores
	blueScoreBoard.center = { 580, 400 };
	redScoreBoard.center = { 30, 400 };

	drawInteger(&blueScoreBoard, (int)getBlueScore(), "", 2.0, { 200, 30, 30 });
	drawInteger(&redScoreBoard, (int)getRedScore(), "", 2.0, { 30, 30, 200 });

	blueScoreBoard.center = { 450, 405 };
	redScoreBoard.center = { 110, 405 };
	drawInteger(&blueScoreBoard, m_state.boostBlueBlockCount, "Blue Boost: ", 1.0, { 200, 30, 30 });
	drawInteger(&redScoreBoard, m_state.boostRedBlockCount, "Red Boost: ", 1.0, { 30, 30, 200 });

	if ((m_state.blueBoostButton >= BUTTON_PUSH) && (m_state.blueBoostButton < BUTTON_PUSH_OVER_10SEC)) {
		buttonState = CV_FILLED;
	}
	else {
		buttonState = 1;
	}
	drawButton({ 434, 400 }, buttonState, { 200, 30, 30 }, 10);

	if ((m_state.redBoostButton >= BUTTON_PUSH) && (m_state.redBoostButton < BUTTON_PUSH_OVER_10SEC)) {
		buttonState = CV_FILLED;
	}
	else {
		buttonState = 1;
	}
	drawButton({ 94, 400 }, buttonState, { 30, 30, 200}, 10);

	blueScoreBoard.center = { 450, 390 };
	redScoreBoard.center = { 110, 390 };
	drawInteger(&blueScoreBoard, m_state.forceBlueBlockCount, "Blue Force: ", 1.0, { 200, 30, 30 });
	drawInteger(&redScoreBoard, m_state.forceRedBlockCount, "Red Force: ", 1.0, { 30, 30, 200 });

	if ((m_state.blueForceButton >= BUTTON_PUSH) && (m_state.blueForceButton < BUTTON_PUSH_OVER_10SEC)) {
		buttonState = CV_FILLED;
	}
	else {
		buttonState = 1;
	}
	drawButton({ 434, 385 }, buttonState, { 200, 30, 30 }, 10);

	if ((m_state.redForceButton >= BUTTON_PUSH) && (m_state.redForceButton < BUTTON_PUSH_OVER_10SEC)) {
		buttonState = CV_FILLED;
	}
	else {
		buttonState = 1;
	}
	drawButton({ 94, 385 }, buttonState, { 30, 30, 200 }, 10);

	blueScoreBoard.center = { 450, 375 };
	redScoreBoard.center = { 110, 375 };
	drawInteger(&blueScoreBoard, m_state.liftBlueBlockCount, "Blue Lift: ", 1.0, { 200, 30, 30 });
	drawInteger(&redScoreBoard, m_state.liftRedBlockCount, "Red Lift: ", 1.0, { 30, 30, 200 });

	if ((m_state.blueLiftButton >= BUTTON_PUSH) && (m_state.blueLiftButton < BUTTON_PUSH_OVER_10SEC)) {
		buttonState = CV_FILLED;
	}
	else {
		buttonState = 1;
	}
	drawButton({ 434, 370 }, buttonState, { 200, 30, 30 }, 10);

	if ((m_state.redLiftButton >= BUTTON_PUSH) && (m_state.redLiftButton < BUTTON_PUSH_OVER_10SEC)) {
		buttonState = CV_FILLED;
	}
	else {
		buttonState = 1;
	}
	drawButton({ 94, 370 }, buttonState, { 30, 30, 200 }, 10);

	//display owners
	cv::Scalar colorNoOwner(128, 180, 128);
	cv::Scalar colorBlue(240, 60, 60);
	cv::Scalar colorRed(60, 60, 240);

	switch (m_state.switchRedOwner) {
	case OWNED_BY_RED:
		drawButton(m_platformStructure.structures[RED_SWITCH_ZONE].center, CV_FILLED, colorRed, 20);
		break;
	default:
		drawButton(m_platformStructure.structures[RED_SWITCH_ZONE].center, CV_FILLED, colorNoOwner, 20);
		break;
	}

	switch (m_state.switchBlueOwner) {
	case OWNED_BY_BLUE:
		drawButton(m_platformStructure.structures[BLUE_SWITCH_ZONE].center, CV_FILLED, colorBlue, 20);
		break;
	default:
		drawButton(m_platformStructure.structures[BLUE_SWITCH_ZONE].center, CV_FILLED, colorNoOwner, 20);
		break;
	}

	switch (m_state.scaleOwner) {
	case OWNED_BY_BLUE:
		drawButton(m_platformStructure.structures[SCALE_ZONE].center, CV_FILLED, colorBlue, 25);
		break;
	case OWNED_BY_RED:
		drawButton(m_platformStructure.structures[SCALE_ZONE].center, CV_FILLED, colorRed, 25);
		break;
	default:
		drawButton(m_platformStructure.structures[SCALE_ZONE].center, CV_FILLED, colorNoOwner, 25);
		break;
	}
}

