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
const int IMAGE_RESOLUSION_y = 720;

const int ORIGIN_X = 40;
const int ORIGIN_Y = 680;

const int MAXIMUM_X = 1270;
const int MAXIMUM_Y = 20;

const int SECONDS_PER_LABEL = 10;
const int POINTS_PER_SECOND = 7;
const int SCORES_PER_LABEL = 20;
const int POINTS_PER_SCORE = 2;

//global objects
Mat gameScore(IMAGE_RESOLUSION_y, IMAGE_RESOLUSION_X, CV_8UC3, Scalar(0, 0, 0));
alliance redAlliance;
alliance blueAlliance;
platform gamePlatform;
displayPlatform showPlatform;
actionMessageType message[2];

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

	showPlatform.setState(&initState);
	showPlatform.setRedScore(initRedScore);
	showPlatform.setBlueScore(initBlueScore);
	showPlatform.setLogFile(NULL);
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
		putText(gameScore, numberString, cvPoint(36 + i*POINTS_PER_SECOND*SECONDS_PER_LABEL, 700),
			FONT_HERSHEY_COMPLEX_SMALL, 0.8, textColor, 1, CV_AA);

		Point x0 = Point(ORIGIN_X + i*POINTS_PER_SECOND*SECONDS_PER_LABEL, 680);
		Point x1 = Point(ORIGIN_X + i*POINTS_PER_SECOND*SECONDS_PER_LABEL, 675);
		line(gameScore,	x0, x1, coordinateColor, 1, 8);

		sprintf_s(numberString, "%d", i*SCORES_PER_LABEL);
		putText(gameScore, numberString, cvPoint(5, 680 - i * SCORES_PER_LABEL*POINTS_PER_SCORE),
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
	//two message buffers to delay the output message by 1 for last message detection
	int frontMessageIdx, backMessageIdx;

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
	gamePlatform.setState(&initState);
	gamePlatform.setRedScore(initRedScore);
	gamePlatform.setBlueScore(initBlueScore);
	gamePlatform.setLogFile(stdout);
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

		for (int i = 0; i < 2; i++) {
			message[i].actionIndex = actionCounter;
			message[i].alliance = ALLIANCE_BLUE;
			message[i].commitActionFlag = false;
			message[i].quitFlag = false;
		}

		newActionCount = 0;
		frontMessageIdx = 0;
		backMessageIdx = -1;
		for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {

			//send the previous message out
			if (backMessageIdx == frontMessageIdx) {
				showPlatform.sendAction(&message[backMessageIdx]);
				backMessageIdx ^= 1;
			}

			if ((redAction[i].actionType != INVALID_ACTION) && (redAction[i].actionType != RED_ACTION_NONE)) {
				gamePlatform.setRobotAction(&redAction[i], ALLIANCE_RED, actionCounter);
				newActionCount++;

				memcpy(&message[frontMessageIdx].action, &redAction[i], sizeof(searchActionType));
				message[frontMessageIdx].alliance = ALLIANCE_RED;
				if (backMessageIdx == -1) {
					backMessageIdx = frontMessageIdx;
				}
				frontMessageIdx ^= 1;
			}

			if (backMessageIdx == frontMessageIdx) {
				showPlatform.sendAction(&message[backMessageIdx]);
				backMessageIdx ^= 1;
			}

			if ((blueAction[i].actionType != INVALID_ACTION) && (blueAction[i].actionType != BLUE_ACTION_NONE)) {
				gamePlatform.setRobotAction(&blueAction[i], ALLIANCE_BLUE, actionCounter);
				newActionCount++;

				memcpy(&message[frontMessageIdx].action, &blueAction[i], sizeof(searchActionType));
				message[frontMessageIdx].alliance = ALLIANCE_BLUE;
				if (backMessageIdx == -1) {
					backMessageIdx = frontMessageIdx;
				}
				frontMessageIdx ^= 1;
			}
			if (backMessageIdx == frontMessageIdx) {
				showPlatform.sendAction(&message[backMessageIdx]);
				backMessageIdx ^= 1;
			}
		}

		//send the last message
		if (newActionCount != 0) {
			message[backMessageIdx].commitActionFlag = true;
			showPlatform.sendAction(&message[backMessageIdx]);
		}
		else {
			//send quit command
			message[backMessageIdx].quitFlag = true;
			message[backMessageIdx].commitActionFlag = true;
			showPlatform.sendAction(&message[backMessageIdx]);
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
		redEnd.x = blueEnd.x = POINTS_PER_SECOND * (int) floor(gamePlatform.getTime() + 0.5);
		redEnd.y = ORIGIN_Y - POINTS_PER_SCORE * gamePlatform.getRedScore();
		blueEnd.y = ORIGIN_Y - POINTS_PER_SCORE * gamePlatform.getBlueScore();
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
	redEnd.y = ORIGIN_Y - POINTS_PER_SCORE * gamePlatform.getRedScore();
	blueEnd.y = ORIGIN_Y - POINTS_PER_SCORE * gamePlatform.getBlueScore();

	//stop display
	WaitForSingleObject(threadHandle, INFINITE);
	CloseHandle(threadHandle);

	//final display of score board
	imshow("FRC 2018 Game Result", gameScore);
	waitKey(0);
	return 0;
}
