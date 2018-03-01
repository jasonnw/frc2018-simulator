
#include "config.h"
#include "displayPlatform.h"

const float ORIGION_POINT_X = 10;
const float ORIGION_POINT_Y = 710;
const float PIXELS_PER_INCH = (float) 1.9;
const cv::Scalar wallColor = { 128, 128, 128 };
const float FRAME_DELAY_IN_MS = 16;

displayPlatform::displayPlatform()
{
	int errorCode;
	m_pQueue = new messageQueue <actionMessageType> (MESSAGE_QUEUE_DEPTH, &errorCode);

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

	//block on message Q
	m_pQueue->send(pActionIn);
	return 0;
}

int displayPlatform::updatePlatform(int actionIndexIn)
{
	actionMessageType message;
	bool commitMessageFlag;
	bool quitFlag;
	float earliestFinishTime;

	if (m_pQueue == NULL) {
		printf("ERROR, message queue is invalid\n");
		return -1;
	}

	quitFlag = false;
	do {
		commitMessageFlag = false;
		if (0 == m_pQueue->tryReceive(&message)) {
			commitMessageFlag = message.commitActionFlag;
			quitFlag = message.quitFlag;
			if (!quitFlag) {
				setRobotAction(&message.action, message.alliance, actionIndexIn);
			}
		}
		else {
			waitKey(10);
		}
	} while (!commitMessageFlag);

	if (quitFlag) {
		//finish all pending actions
		while (hasPendingActions()) {
			earliestFinishTime = getEarliestFinishTime();
			playTotheNextTime(earliestFinishTime, actionIndexIn, FRAME_DELAY_IN_MS);
		}
		updateField();
		drawPlatform(0);
		return 1;
	}
	else {
		earliestFinishTime = getEarliestFinishTime();
		playTotheNextTime(earliestFinishTime, actionIndexIn, FRAME_DELAY_IN_MS);
		return 0;
	}
}

void displayPlatform::playTotheNextTime(float nextTimeIn, int actionIndexIn, float frameDelayIn)
{
	float currentTime = getTime();
	float frameDelay = (float) (frameDelayIn / 1000.0);

	if (frameDelay == 0) {
		frameDelay = (float) (FRAME_DELAY_IN_MS / 1000);
	}

	for (float i = currentTime; i < nextTimeIn; i += frameDelay)
	{
		if (0 != commitAction(i, actionIndexIn)) {
			printf("ERROR, cannot display invalid action");
		}
		updateField();
		drawPlatform((int) floor(frameDelay * 1000));
	}

	//last action
	if (0 != commitAction(nextTimeIn, actionIndexIn)) {
		printf("ERROR, cannot display the last action \n");
	}
	updateField();
	drawPlatform((int)floor(frameDelay * 1000));
}

Point displayPlatform::coordinateToPoint(float xIn, float yIn)
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
	imshow("FRC 2018 Game Simulation", *m_pPlatform);

	waitKey(delayIn);
}

void displayPlatform::drwaObject(const rectangleObjectType *pObjectIn)
{
	Point point1;
	Point point2;

	point1 = coordinateToPoint(pObjectIn->center.x + pObjectIn->sizeX / 2, pObjectIn->center.y + pObjectIn->sizeY / 2);
	point2 = coordinateToPoint(pObjectIn->center.x - pObjectIn->sizeX / 2, pObjectIn->center.y - pObjectIn->sizeY / 2);

	rectangle(*m_pPlatform, point1, point2, pObjectIn->color, CV_FILLED, 8, 0);
}

void displayPlatform::drawField(void)
{
	Point lineStart;
	Point lineEnd;

	//four walls
	lineStart = coordinateToPoint(0, 0);
	lineEnd = coordinateToPoint(m_platformStructure.eastWall, 0);
	line(*m_pPlatform, lineStart, lineEnd, wallColor, 1, 8);

	lineStart = coordinateToPoint(0, 0);
	lineEnd = coordinateToPoint(0, m_platformStructure.northWall);
	line(*m_pPlatform, lineStart, lineEnd, wallColor, 1, 8);

	lineStart = coordinateToPoint(0, m_platformStructure.northWall);
	lineEnd = coordinateToPoint(m_platformStructure.eastWall, m_platformStructure.northWall);
	line(*m_pPlatform, lineStart, lineEnd, wallColor, 1, 8);

	lineStart = coordinateToPoint(m_platformStructure.eastWall, 0);
	lineEnd = coordinateToPoint(m_platformStructure.eastWall, m_platformStructure.northWall);
	line(*m_pPlatform, lineStart, lineEnd, wallColor, 1, 8);

	for (int i = RED_SWITCH_ZONE; i < NUM_STILL_STRUCTURE; i++) {
		drwaObject(&m_platformStructure.structures[i]);
	}

	drwaObject(&m_platformStructure.blueExchangeZone);
	drwaObject(&m_platformStructure.blueLiftZone);
	drwaObject(&m_platformStructure.bluePlatformZone);
	drwaObject(&m_platformStructure.bluePowerCubeZone);
	drwaObject(&m_platformStructure.blueSwitchNorthPlate);
	drwaObject(&m_platformStructure.blueSwitchSouthPlate);
	drwaObject(&m_platformStructure.redExchangeZone);
	drwaObject(&m_platformStructure.redLiftZone);
	drwaObject(&m_platformStructure.redPlatformZone);
	drwaObject(&m_platformStructure.redPowerCubeZone);
	drwaObject(&m_platformStructure.redSwitchNorthPlate);
	drwaObject(&m_platformStructure.redSwitchSouthPlate);
	drwaObject(&m_platformStructure.scaleNorthPlate);
	drwaObject(&m_platformStructure.scaleSouthPlate);

}

void displayPlatform::updateField(void)
{
	//erase all
	*m_pPlatform = Scalar(0, 0, 0);
	drawField();
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		drwaObject(m_redRobots[i].getPosition());
		drwaObject(m_blueRobots[i].getPosition());
	}
}

