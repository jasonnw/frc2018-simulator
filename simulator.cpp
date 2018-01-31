// simulator.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "alliance.h"

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
Mat image(IMAGE_RESOLUSION_y, IMAGE_RESOLUSION_X, CV_8UC3, Scalar(0, 0, 0));
alliance redAlliance;
alliance blueAlliance;
platform gamePlatform;

static void initDisplay(void)
{
	char numberString[4];

	//draw coordinate and write labels
	Point start = Point(ORIGIN_X, ORIGIN_Y);
	Point xEnd = Point(MAXIMUM_X, ORIGIN_Y);
	Point yEnd = Point(ORIGIN_X, MAXIMUM_Y);

	line(image,	start, xEnd, coordinateColor, 1, 8);
	line(image, start, yEnd, coordinateColor, 1, 8);

	putText(image, "Time", cvPoint(MAXIMUM_X - 70, ORIGIN_Y + 30),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, textColor, 1, CV_AA);

	putText(image, "Score", cvPoint(5, MAXIMUM_Y),
		FONT_HERSHEY_COMPLEX_SMALL, 0.8, textColor, 1, CV_AA);

	for (int i = 0; i < 18; i++) {
		sprintf_s(numberString, "%d", i*SECONDS_PER_LABEL);
		putText(image, numberString, cvPoint(36 + i*POINTS_PER_SECOND*SECONDS_PER_LABEL, 700),
			FONT_HERSHEY_COMPLEX_SMALL, 0.8, textColor, 1, CV_AA);

		Point x0 = Point(ORIGIN_X + i*POINTS_PER_SECOND*SECONDS_PER_LABEL, 680);
		Point x1 = Point(ORIGIN_X + i*POINTS_PER_SECOND*SECONDS_PER_LABEL, 675);
		line(image,	x0, x1, coordinateColor, 1, 8);

		sprintf_s(numberString, "%d", i*SCORES_PER_LABEL);
		putText(image, numberString, cvPoint(5, 680 - i * SCORES_PER_LABEL*POINTS_PER_SCORE),
			FONT_HERSHEY_COMPLEX_SMALL, 0.8, textColor, 1, CV_AA);

		Point y0 = Point(ORIGIN_X, ORIGIN_Y - i * SCORES_PER_LABEL*POINTS_PER_SCORE);
		Point y1 = Point(ORIGIN_X+5, ORIGIN_Y - i * SCORES_PER_LABEL*POINTS_PER_SCORE);
		line(image, y0, y1, coordinateColor, 1, 8);
	}
}

int main(int argc, char** argv)
{
	pendingActionType redAction;
	pendingActionType blueAction;
	Point redStart, redEnd;
	Point blueStart, blueEnd;
	int actionCounter;
	FILE *pRedActionLog = NULL;
	FILE *pBlueActionLog = NULL;
	errno_t errCode;

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

	initDisplay();

	//initialization
	redAlliance.initAlliance(ALLIANCE_RED, NULL, RED_CONFIGURATION);
	blueAlliance.initAlliance(ALLIANCE_BLUE, NULL, BLUE_CONFIGURATION);

	gamePlatform.setState(&initState);
	gamePlatform.setRedScore(initRedScore);
	gamePlatform.setBlueScore(initBlueScore);
	gamePlatform.setLogFIle(stdout);

	actionCounter = 0;
	redAlliance.syncLocalPlatform(gamePlatform, actionCounter);
	blueAlliance.syncLocalPlatform(gamePlatform, actionCounter);

	redStart.x = ORIGIN_X;
	redStart.y = ORIGIN_Y;
	blueStart.x = ORIGIN_X;
	blueStart.y = ORIGIN_Y;
	do {
		//enable action search log at any time
		//if (actionCounter == 72) {
		//	blueAlliance.setLogFile(pBlueActionLog);
		//}else {
		//  blueAlliance.setLogFile(NULL);
		//}
		redAlliance.getBestAction(&redAction);
		blueAlliance.getBestAction(&blueAction);

		//platform only take actions in time order
		//find out which action is finished first
		if (redAction.projectedFinishTime <= blueAction.projectedFinishTime) {
			if (redAction.projectedFinishTime <= CLIMB_END_TIME) {
				if (0 != gamePlatform.takeAction(redAction.actionType, redAction.projectedFinishTime, redAction.robotIndex, actionCounter)) {
					printf("Error: Action %d of red alliance robot(%d) is rejected\n", redAction.actionType, redAction.robotIndex);
				}
				redAlliance.syncLocalPlatform(gamePlatform, actionCounter);
			}
			else {
				break; //the finish time is after game over, stop here
			}
		}
		else {
			if (blueAction.projectedFinishTime <= CLIMB_END_TIME) {
				if (0 != gamePlatform.takeAction(blueAction.actionType, blueAction.projectedFinishTime, blueAction.robotIndex, actionCounter)) {
					printf("Error: Action %d of blue alliance robot(%d) is rejected\n", blueAction.actionType, blueAction.robotIndex);
				}
				blueAlliance.syncLocalPlatform(gamePlatform, actionCounter);
			}
			else {
				break; //the finish time is after game over, stop here
			}
		}
		//Note: after one action is done, only re-run syncLocalPlatform() of the alliance which take the action, because syncLocalPlatform() is very slow.

		//update the score display
		redEnd.x = blueEnd.x = POINTS_PER_SECOND * (int) floor(gamePlatform.getTime() + 0.5);
		redEnd.y = ORIGIN_Y - POINTS_PER_SCORE * gamePlatform.getRedScore();
		blueEnd.y = ORIGIN_Y - POINTS_PER_SCORE * gamePlatform.getBlueScore();
		line(image,	redStart, redEnd, redColor, 3, 8);
		line(image, blueStart, blueEnd, blueColor, 3, 8);

		redStart = redEnd;
		blueStart = blueEnd;
		actionCounter++;
	} while (!gamePlatform.isGameTimeOver());

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

	//final display of score board
	imshow("FRC 2018 Game Simulation", image);

	waitKey(0);
	return 0;
}
