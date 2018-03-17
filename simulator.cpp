// simulator.cpp : Defines the entry point for the console application.
//
//#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "os_wrapper.h"
#include "alliance.h"
#include "displayPlatform.h"

using namespace cv;
using namespace std;

#ifndef _MSC_VER

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

//global objects
alliance redAlliance;
alliance blueAlliance;
platform gamePlatform;
displayPlatform showPlatform;
actionMessageType messageBuffer;

#ifdef _MSC_VER
static DWORD WINAPI displayThreadEntry(LPVOID lpParameter)
#else
static void* displayThreadEntry(LPVOID lpParameter)
#endif
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
		actionCount++;
		returnVal = showPlatform.updatePlatform(actionCount);
	} while (returnVal == 0);

	return 0;
}

int main(int argc, char** argv)
{
	searchActionType redAction[NUMBER_OF_ROBOTS];
	searchActionType blueAction[NUMBER_OF_ROBOTS];
	const pendingActionType *pAction;
	int actionCounter;
	int newActionCount;
	FILE *pRedActionLog = NULL;
	FILE *pBlueActionLog = NULL;
	errno_t errCode;
	double earliestFinishTime;
	bool noActionChangeFlag;

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

	//start display thread
#ifdef _MSC_VER
	DWORD threadID;
	HANDLE threadHandle = CreateThread(0, 0, displayThreadEntry, NULL, 0, &threadID);
#else
	pthread_t threadHandle;
	pthread_create(&threadHandle, 0, displayThreadEntry, NULL);
#endif

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

	//enable action search log at beginning
	//redAlliance.setLogFile(pRedActionLog);
	//blueAlliance.setLogFile(pBlueActionLog);

	do {
		actionCounter++;

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
				gamePlatform.setRobotAction(&redAction[i], ALLIANCE_RED, actionCounter, &noActionChangeFlag);
				newActionCount++;

				pAction = gamePlatform.getRobotAction(ALLIANCE_RED, i);
				memcpy(&messageBuffer.action, pAction, sizeof(pendingActionType));
				messageBuffer.startPos = gamePlatform.getRobotPos(ALLIANCE_RED, i);
				messageBuffer.cubeIdx = gamePlatform.getRobotCubeIdx(ALLIANCE_RED, i);
				messageBuffer.alliance = ALLIANCE_RED;
				messageBuffer.robotIdx = i;
				showPlatform.sendAction(&messageBuffer);
			}

			if ((blueAction[i].actionType != INVALID_ACTION) && (blueAction[i].actionType != BLUE_ACTION_NONE)) {
				gamePlatform.setRobotAction(&blueAction[i], ALLIANCE_BLUE, actionCounter, &noActionChangeFlag);
				newActionCount++;

				pAction = gamePlatform.getRobotAction(ALLIANCE_BLUE, i);
				memcpy(&messageBuffer.action, pAction, sizeof(pendingActionType));
				messageBuffer.startPos = gamePlatform.getRobotPos(ALLIANCE_BLUE, i);
				messageBuffer.cubeIdx = gamePlatform.getRobotCubeIdx(ALLIANCE_BLUE, i);
				messageBuffer.alliance = ALLIANCE_BLUE;
				messageBuffer.robotIdx = i;
				showPlatform.sendAction(&messageBuffer);
			}
		}

		//send the last message
		if ((newActionCount == 0) || (gamePlatform.isGameTimeOver())) {
			//send quit command
			messageBuffer.quitFlag = true;
			messageBuffer.commitActionFlag = true;
			showPlatform.sendAction(&messageBuffer);
		}
		else {
			//send commit command
			messageBuffer.commitActionFlag = true;
			showPlatform.sendAction(&messageBuffer);
		}

		earliestFinishTime = gamePlatform.getEarliestStopTime();
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
	} while ((!gamePlatform.isGameTimeOver()) && (newActionCount!=0));

	if (pRedActionLog != NULL) {
		fclose(pRedActionLog);
	}
	if (pBlueActionLog != NULL) {
		fclose(pBlueActionLog);
	}

	//print the final score
	gamePlatform.logFinalRanking();

	//stop display
#ifdef _MSC_VER
	WaitForSingleObject(threadHandle, INFINITE);
	CloseHandle(threadHandle);
#else
	void *returnValue;
	pthread_join(threadHandle, &returnValue);
#endif

	return 0;
}
