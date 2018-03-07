// simulator.cpp : Defines the entry point for the console application.
//
//#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <windows.h>

#include "alliance.h"
#include "displayPlatform.h"

using namespace cv;
using namespace std;

//display settings
const cv::Scalar textColor(200, 200, 250);
const cv::Scalar coordinateColor(100, 100, 100);
const cv::Scalar blueColor(200, 0, 0);
const cv::Scalar redColor(0, 0, 200);

const int IMAGE_RESOLUSION_X = 1280;
const int IMAGE_RESOLUSION_y = 800;

const int ORIGIN_X = 40;
const int ORIGIN_Y = 760;

const int MAXIMUM_X = 1270;
const int MAXIMUM_Y = 20;

const int SECONDS_PER_LABEL = 10;
const int POINTS_PER_SECOND = 7;
const int SCORES_PER_LABEL = 20;
const int POINTS_PER_SCORE = 2;

const int LABAL_OFFSET_Y = 780;

//global objects
Mat gameScore(IMAGE_RESOLUSION_y, IMAGE_RESOLUSION_X, CV_8UC3, Scalar(0, 0, 0));
alliance redAlliance;
alliance blueAlliance;
platform gamePlatform;
displayPlatform showPlatform;
actionMessageType messageBuffer;

#ifndef _MSC_VER
//missing definitions of MS visual studio unique code
typedef int errno_t;
#define sprintf_s	sprintf
static errno_t fopen_s(FILE** pFile, const char *filename,  const char *mode)
{
	*pFile = fopen(filename, mode);
	if(*pFile == NULL) {
		return -1;
	}else {
		return 0;
	}
}
#endif

static DWORD WINAPI displayThreadEntry(LPVOID lpParameter)
{
	int returnVal;
	int actionCount = 0;

	showPlatform.setLogFile(stdout);
	showPlatform.setState(&initState);
	showPlatform.setRedScore(initRedScore);
	showPlatform.setBlueScore(initBlueScore);
	showPlatform.configRedRobots(RED_CONFIGURATION);
	showPlatform.configBlueRobots(BLUE_CONFIGURATION);

	showPlatform.drawPlatform(1000);

	do {
		returnVal = showPlatform.updatePlatform(actionCount);
		actionCount++;
	} while (returnVal == 0);

	return 0;
}

static void initDisplay(void)
{
	char numberString[4];

	//draw coordinate and write labels
	Point start = Point(ORIGIN_X, ORIGIN_Y);
	Point xEnd = Point(MAXIMUM_X, ORIGIN_Y);
	Point yEnd = Point(ORIGIN_X, MAXIMUM_Y);

	line(gameScore,	start, xEnd, coordinateColor, 1, 8);
	line(gameScore, start, yEnd, coordinateColor, 1, 8);

	putText(gameScore, "Time", cvPoint(MAXIMUM_X - 70, ORIGIN_Y + 30),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, textColor, 1, CV_AA);

	putText(gameScore, "Score", cvPoint(5, MAXIMUM_Y),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, textColor, 1, CV_AA);

	for (int i = 0; i < 18; i++) {
		sprintf_s(numberString, "%d", i*SECONDS_PER_LABEL);
		putText(gameScore, numberString, cvPoint(36 + i*POINTS_PER_SECOND*SECONDS_PER_LABEL, LABAL_OFFSET_Y),
			FONT_HERSHEY_COMPLEX_SMALL, 0.8, textColor, 1, CV_AA);

		Point x0 = Point(ORIGIN_X + i*POINTS_PER_SECOND*SECONDS_PER_LABEL, LABAL_OFFSET_Y - 20);
		Point x1 = Point(ORIGIN_X + i*POINTS_PER_SECOND*SECONDS_PER_LABEL, LABAL_OFFSET_Y - 25);
		line(gameScore,	x0, x1, coordinateColor, 1, 8);

		sprintf_s(numberString, "%d", i*SCORES_PER_LABEL);
		putText(gameScore, numberString, cvPoint(5, LABAL_OFFSET_Y - 20 - i * SCORES_PER_LABEL*POINTS_PER_SCORE),
			FONT_HERSHEY_COMPLEX_SMALL, 0.8, textColor, 1, CV_AA);

		Point y0 = Point(ORIGIN_X, ORIGIN_Y - i * SCORES_PER_LABEL*POINTS_PER_SCORE);
		Point y1 = Point(ORIGIN_X+5, ORIGIN_Y - i * SCORES_PER_LABEL*POINTS_PER_SCORE);
		line(gameScore, y0, y1, coordinateColor, 1, 8);
	}
}


int main(int argc, char** argv)
{
	searchActionType redAction[NUMBER_OF_ROBOTS];
	searchActionType blueAction[NUMBER_OF_ROBOTS];
	const pendingActionType *pAction;
	Point redStart, redEnd;
	Point blueStart, blueEnd;
	int actionCounter;
	int newActionCount;
	FILE *pRedActionLog = NULL;
	FILE *pBlueActionLog = NULL;
	errno_t errCode;
	float earliestFinishTime;

	if (argc != 3) {
		printf("usage: simulator [redActionLogFileName blueActionLogFileName]\n");
	}
	else {
		if ((errCode = fopen_s(&pRedActionLog, argv[1], "w")) != 0) {
			printf("Error: open red alliance log file %s failed, error code 0x%x\n", argv[1], errCode);
		 }
		if ((errCode = fopen_s(&pBlueActionLog, argv[2], "w")) != 0) {
			printf("Error: open blue alliance log file %s failed, error code 0x%x\n", argv[2], errCode);
		}
	}

	DWORD threadID;
	HANDLE threadHandle = CreateThread(0, 0, displayThreadEntry, NULL, 0, &threadID);


	initDisplay();

	//initialization
	gamePlatform.setLogFile(NULL);
	gamePlatform.setState(&initState);
	gamePlatform.setRedScore(initRedScore);
	gamePlatform.setBlueScore(initBlueScore);
	gamePlatform.configRedRobots(RED_CONFIGURATION);
	gamePlatform.configBlueRobots(BLUE_CONFIGURATION);

	redAlliance.initAlliance(ALLIANCE_RED, NULL, RED_CONFIGURATION, gamePlatform);
	blueAlliance.initAlliance(ALLIANCE_BLUE, NULL, BLUE_CONFIGURATION, gamePlatform);

	actionCounter = 0;
	redAlliance.syncLocalPlatform(gamePlatform, actionCounter);
	blueAlliance.syncLocalPlatform(gamePlatform, actionCounter);

	redStart.x = ORIGIN_X;
	redStart.y = ORIGIN_Y;
	blueStart.x = ORIGIN_X;
	blueStart.y = ORIGIN_Y;

	//enable action search log at beginning
	//redAlliance.setLogFile(pRedActionLog);
	//blueAlliance.setLogFile(pBlueActionLog);

	do {
		//enable action search log at any time
		//if (actionCounter == 72) {
		//	blueAlliance.setLogFile(pBlueActionLog);
		//}else {
		//  blueAlliance.setLogFile(NULL);
		//}
		redAlliance.getBestAction(redAction);
		blueAlliance.getBestAction(blueAction);

		messageBuffer.commitActionFlag = false;
		messageBuffer.quitFlag = false;
		messageBuffer.actionIndex = actionCounter;

		newActionCount = 0;
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {

			if ((redAction[i].actionType != INVALID_ACTION) && (redAction[i].actionType != RED_ACTION_NONE)) {
				gamePlatform.setRobotAction(&redAction[i], ALLIANCE_RED, actionCounter);
				newActionCount++;

				pAction = gamePlatform.getRobotAction(ALLIANCE_RED, i);
				memcpy(&messageBuffer.action, pAction, sizeof(pendingActionType));
				messageBuffer.startPos = gamePlatform.getRobotPos(ALLIANCE_RED, i);
				messageBuffer.alliance = ALLIANCE_RED;
				messageBuffer.robotIdx = i;
				showPlatform.sendAction(&messageBuffer);
			}

			if ((blueAction[i].actionType != INVALID_ACTION) && (blueAction[i].actionType != BLUE_ACTION_NONE)) {
				gamePlatform.setRobotAction(&blueAction[i], ALLIANCE_BLUE, actionCounter);
				newActionCount++;

				pAction = gamePlatform.getRobotAction(ALLIANCE_BLUE, i);
				memcpy(&messageBuffer.action, pAction, sizeof(pendingActionType));
				messageBuffer.startPos = gamePlatform.getRobotPos(ALLIANCE_BLUE, i);
				messageBuffer.alliance = ALLIANCE_BLUE;
				messageBuffer.robotIdx = i;
				showPlatform.sendAction(&messageBuffer);
			}
		}

		//send the last message
		if (newActionCount != 0) {
			messageBuffer.commitActionFlag = true;
			showPlatform.sendAction(&messageBuffer);
		}
		else {
			//send quit command
			messageBuffer.quitFlag = true;
			messageBuffer.commitActionFlag = true;
			showPlatform.sendAction(&messageBuffer);
		}

		earliestFinishTime = gamePlatform.getEarliestFinishTime();
		if (0 != gamePlatform.commitAction(earliestFinishTime, actionCounter, INVALID_ALLIANCE)) {
			printf("Error: Action is rejected\n");
		}

		if (newActionCount == 0) {
			//finish all pending actions and stop
			gamePlatform.finishAllPendingActions(actionCounter, INVALID_ALLIANCE);
		}
		else {
			redAlliance.syncLocalPlatform(gamePlatform, actionCounter);
			blueAlliance.syncLocalPlatform(gamePlatform, actionCounter);
		}

		//update the score display
		redEnd.x = blueEnd.x = ORIGIN_X + POINTS_PER_SECOND * (int) floor(gamePlatform.getTime() + 0.5);
		redEnd.y = ORIGIN_Y - POINTS_PER_SCORE * (int) gamePlatform.getRedScore();
		blueEnd.y = ORIGIN_Y - POINTS_PER_SCORE * (int) gamePlatform.getBlueScore();
		line(gameScore,	redStart, redEnd, redColor, 3, 8);
		line(gameScore, blueStart, blueEnd, blueColor, 3, 8);

		redStart = redEnd;
		blueStart = blueEnd;
		actionCounter++;
	} while ((!gamePlatform.isGameTimeOver()) && (newActionCount!=0));

	if (pRedActionLog != NULL) {
		fclose(pRedActionLog);
	}
	if (pBlueActionLog != NULL) {
		fclose(pBlueActionLog);
	}

	//print the final score
	gamePlatform.logFinalScore();

	redEnd.x = blueEnd.x = POINTS_PER_SECOND * (int)floor(gamePlatform.getTime() + 0.5);
	redEnd.y = ORIGIN_Y - POINTS_PER_SCORE * (int) gamePlatform.getRedScore();
	blueEnd.y = ORIGIN_Y - POINTS_PER_SCORE * (int) gamePlatform.getBlueScore();

	//stop display
	WaitForSingleObject(threadHandle, INFINITE);
	CloseHandle(threadHandle);

	//final display of score board
	imshow("FRC 2018 Game Result", gameScore);
	waitKey(0);
	return 0;
}
