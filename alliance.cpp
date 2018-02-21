
//#include "stdafx.h"
#include <stdlib.h>
#include "alliance.h"

alliance::alliance()
{
	int searchSize;

	m_timeInSec = 0;
	m_pLogFIle = NULL;

	//reset best action to an invalid time point
	memset(m_bestAction, 0, sizeof(m_bestAction));
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_bestAction[i].actionType = INVALID_ACTION;
		m_bestAction[i].projectedFinishTime = 0;
	}
	//Set action time as 0, platform will catch this error later  

	searchSize = NUM_OF_POSSIBLE_ACTIONS;
	m_maxSearchListSize = searchSize;
	for (int i = 1; i < MAXIMUM_PENDING_ACTIONS; i++) {
		searchSize *= NUM_OF_POSSIBLE_ACTIONS;
		m_maxSearchListSize += searchSize;
	}

	//add one action to each robot to make sure no idle robot
	searchSize = NUM_OF_POSSIBLE_ACTIONS;
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		searchSize *= NUMBER_OF_ROBOTS;
	}
	m_maxSearchListSize += searchSize; //add one action per robot action
	m_maxSearchListSize++; //add the first initial action

	m_pSearchList = (searchActionType*) malloc(m_maxSearchListSize * sizeof(searchActionType));
}


alliance::~alliance()
{
	if (m_pSearchList != NULL) {
		free(m_pSearchList);
	}
}

int alliance::initAlliance(allianceType typeIn, FILE *pLogFile,
	const robotConfigurationType config1In[NUMBER_OF_ROBOTS],
	const platform &platformIn)
{
	if (m_pSearchList == NULL) {
		return -1;
	}
	resetSearchList();
	m_pLogFIle = pLogFile;
	m_allianceType = typeIn;
	m_testPlatForm = platformIn;

	if (typeIn == ALLIANCE_RED) {
		m_pRobots = m_testPlatForm.getRedRobots();
	}
	else {
		m_pRobots = m_testPlatForm.getBlueRobots();
	}
	return 0;
}

void alliance::syncLocalPlatform(const platform &platformIn, int actionIndexIn)
{
	m_timeInSec = platformIn.getTime();
	m_referencePlatForm = platformIn;

	resetSearchList();
	findBestAction(actionIndexIn);
}


//alliance class search for the best action of each robot to reach the best score.
//The search algorithm is,
//1. For the most resent platform state, try all possible actions of all robots.
//2. Try 4 future possible actions for each robot and calculate the final score 
//3. Keep the best action list for the highest score
//4. As pending time down to 0, send the action out
//5. If either side of alliance make a real action, update the local state
//6. Repeat above steps Until the game time is over

//try all possible actions,
void alliance::findBestAction(int actionIndexIn)
{
	platform testPlatForm;
	bool searchFailedFlag;

	int previousActionIndex;

	float currentTime = m_referencePlatForm.getTime();
	float bestFinishTime;
	float finishTime;
	float previousFinishTime;
	float startTime;
	float bestStartTime;

	int bestFinishRobotIdx;
	int layerStartIndex;
	int layerEndIndex;
	int previousLayerStartIndex;
	int previousLayerEndIndex;

	int bestScoreIdx;
	int projectedRedScore, projectedBlueScore, projectedScore;

	int numPendingAction;

	bool isRealActionFlag; 
	bool updetedTreeFlag;
	bool interruptFlag;

	coordinateType actionDonePos;
	coordinateType actionNewPos;
	bool actionDoneWithCube;

	coordinateType bestRobotNewPos;
	bool bestRobotDoneWithCube;

	actionTypeType startAction = (m_allianceType == ALLIANCE_RED) ? CUBE_RED_OFFENCE_SWITCH : CUBE_BLUE_OFFENCE_SWITCH;
	actionTypeType endAction = (m_allianceType == ALLIANCE_RED) ? PUSH_RED_LIFT_BUTTON : PUSH_BLUE_LIFT_BUTTON;
	actionTypeType endRealAction = (m_allianceType == ALLIANCE_RED) ? RED_ACTION_NONE : BLUE_ACTION_NONE;

	//reset best action
	memset(m_bestAction, 0, sizeof(m_bestAction));
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_bestAction[i].actionType = INVALID_ACTION;
		m_bestAction[i].projectedFinishTime = currentTime;
	}
	//Set action time as 0, platform will catch this error later  

	//create the first action entry as the root of all actions
	previousLayerStartIndex = 0;
	previousLayerEndIndex = 1;
	m_pSearchList[previousLayerStartIndex].actionType = INVALID_ACTION;
	m_pSearchList[previousLayerStartIndex].projectedFinalScore = 0; //entry not used
	m_pSearchList[previousLayerStartIndex].previousIndex = INVALID_IDX;
	m_pSearchList[previousLayerStartIndex].projectedFinishTime = currentTime;
	m_pSearchList[previousLayerStartIndex].robotIndex = INDEX_OF_ROBOT_NONE;
	//Note: this root action will not be used in real action list. It is a start point for action searching.
	
	searchFailedFlag = false;
	bestScoreIdx = 0;
	//project the final score if no action is taken
	testPlatForm = m_referencePlatForm;
	testPlatForm.getFinalScore(&projectedRedScore, &projectedBlueScore);
	projectedScore = (m_allianceType == ALLIANCE_RED) ? projectedRedScore - projectedBlueScore : projectedBlueScore - projectedRedScore;

	//build the search list
	for (int pending = 0; pending < MAXIMUM_PENDING_ACTIONS; pending++) {
		//search MAXIMUM_PENDING_ACTIONS number of steps of all possible actions 

		layerEndIndex = layerStartIndex = previousLayerEndIndex;
		updetedTreeFlag = false;
		for (int prevIdx = previousLayerStartIndex; prevIdx < previousLayerEndIndex; prevIdx++) {
			//for each previous step action, search for all possible actions as the next step

			//if this action branch is not legal 
			if (m_pSearchList[prevIdx].projectedFinalScore <= INT32_MIN) {
				continue;
			}

			//give up the current action branch if it cannot improve the score
			if ((pending >= MIN_SCORE_CHECKING_STEP) &&
				(m_pSearchList[prevIdx].projectedFinalScore < projectedScore + pending * SEARCH_CONTINUE_THRESHOLD)) {
				continue;
			}

			for (int act = startAction; act <= endAction; act++) {

				//find out if the action is a real robot moving action
				switch (act) {
				case CUBE_RED_OFFENCE_SWITCH:
				case CUBE_RED_DEFENCE_SWITCH:
				case CUBE_RED_SCALE:
				case CUBE_RED_FORCE_VAULT:
				case CUBE_RED_BOOST_VAULT:
				case CUBE_RED_LIFT_VAULT:
				case LIFT_ONE_RED_ROBOT:
				case CUBE_BLUE_OFFENCE_SWITCH:
				case CUBE_BLUE_DEFENCE_SWITCH:
				case CUBE_BLUE_SCALE:
				case CUBE_BLUE_FORCE_VAULT:
				case CUBE_BLUE_BOOST_VAULT:
				case CUBE_BLUE_LIFT_VAULT:
				case LIFT_ONE_BLUE_ROBOT:
					isRealActionFlag = true;
					break;
				default:
					isRealActionFlag = false;
					break;
				}

				//find out the earliest finish time from 3 robots
				bestFinishTime = CLIMB_END_TIME + 1;
				bestFinishRobotIdx = 0;
				for (int robot = 0; robot < NUMBER_OF_ROBOTS; robot++) {
					//for each action, find out which robot can do it with the shortest delay

					previousActionIndex = prevIdx;
					previousFinishTime = 0;
					interruptFlag = true;
					actionDonePos = m_pRobots[robot].getPosition();
					actionDoneWithCube = m_pRobots[robot].hasCube();
					startTime = currentTime;

					if (m_referencePlatForm.isRobotLifted(m_allianceType, robot) &&
						(isRealActionFlag)) {
						//robot is lifted, not available for any other actions
						previousFinishTime = CLIMB_END_TIME + 1;
					}
					else {
						do {
							//the previous action finish time
							if (m_pSearchList[previousActionIndex].robotIndex == robot) {
								previousFinishTime = m_pSearchList[previousActionIndex].projectedFinishTime;

								if (((m_pSearchList[previousActionIndex].actionType == LIFT_ONE_RED_ROBOT) ||
									(m_pSearchList[previousActionIndex].actionType == LIFT_ONE_BLUE_ROBOT)) &&
									(isRealActionFlag)) {
									//robot is lifted, not available for any other actions
									previousFinishTime = CLIMB_END_TIME + 1;
								}
								actionDonePos = m_pSearchList[previousActionIndex].actionDonePos;
								actionDoneWithCube = m_pSearchList[previousActionIndex].actionDoneWithCube;
								startTime = m_pSearchList[previousActionIndex].projectedFinishTime;
								interruptFlag = false; //wait the current action done before start the next action
								break; //the most recent finish time is the time to start the next action
							}

							previousActionIndex = m_pSearchList[previousActionIndex].previousIndex;
						} while (previousActionIndex != INVALID_IDX);

						if (previousFinishTime == 0) {
							startTime = previousFinishTime = currentTime;
						}
					}

					//the current action finish time
					finishTime = m_pRobots[robot].estimateActionDelayInSec((actionTypeType)act, currentTime, interruptFlag, 
						actionDonePos, actionDoneWithCube, &actionNewPos);
					finishTime += previousFinishTime;

					if (finishTime < bestFinishTime) {
						bestFinishTime = finishTime;
						bestFinishRobotIdx = robot;
						bestRobotNewPos = actionNewPos;
						bestRobotDoneWithCube = actionDoneWithCube;
						bestStartTime = previousFinishTime;
					}
				}

				//create a new action
				if (layerEndIndex >= m_maxSearchListSize) {
					printf("Error: search list too small\n");
				}
				else if (bestFinishTime < CLIMB_END_TIME) {
					m_pSearchList[layerEndIndex].actionType = (actionTypeType)act;
					m_pSearchList[layerEndIndex].previousIndex = prevIdx;
					m_pSearchList[layerEndIndex].projectedFinishTime = bestFinishTime;
					m_pSearchList[layerEndIndex].actionDonePos = bestRobotNewPos;
					m_pSearchList[layerEndIndex].actionIndex = actionIndexIn;

					if (robot::isActionNeedCube((actionTypeType)act)) {
						m_pSearchList[layerEndIndex].actionDoneWithCube = false;
					}
					else {
						//action don't need a cube, pass cube flag to the next action
						m_pSearchList[layerEndIndex].actionDoneWithCube = bestRobotDoneWithCube;
					}

					m_pSearchList[layerEndIndex].robotIndex = bestFinishRobotIdx;
					m_pSearchList[layerEndIndex].startTime = bestStartTime;
					m_pSearchList[layerEndIndex].projectedFinalScore = INT32_MIN;
					updetedTreeFlag = true;
					layerEndIndex++;
				}
				//else, action is not possible, skip it
			}
		}

		//update layer indexes
		if (updetedTreeFlag) {
			previousLayerStartIndex = layerStartIndex;
			previousLayerEndIndex = layerEndIndex;

			//After an action tree is created, search which action sequence can get the best score
			bestScoreIdx = findBestScoreBranch(previousLayerStartIndex, previousLayerEndIndex, actionIndexIn, &numPendingAction);
			if ((bestScoreIdx == 0) || (numPendingAction == 0)) {
				printf("ERROR: cannot find any useful actions\n");
			}
			//bestScoreIdx of the last pending action iteration is the best action list of all
		}
		else {
			break; 
			//no new action could be added, use the previous bestScoreIdx as the final result
		}
	}

	if (bestScoreIdx == 0) {
		printf("ERROR: cannot find any actions, check why NO-ACTION is not used\n");
		searchFailedFlag = true;
	}

	//add one real action for each robot to avoid idle robot
	layerEndIndex = layerStartIndex = previousLayerEndIndex;
	previousLayerStartIndex = bestScoreIdx;
	previousLayerEndIndex = bestScoreIdx + 1;
	updetedTreeFlag = false;
	for (int robot = 0; robot < NUMBER_OF_ROBOTS; robot++) {

		if (m_referencePlatForm.isRobotLifted(m_allianceType, robot)) {
			continue; //lifted on platform, no action for this robot
		}
		previousActionIndex = bestScoreIdx;
		previousFinishTime = 0;
		interruptFlag = true;
		actionDonePos = m_pRobots[robot].getPosition();
		actionDoneWithCube = m_pRobots[robot].hasCube();
		startTime = currentTime;

		do {
			//the previous action finish time
			if (m_pSearchList[previousActionIndex].robotIndex == robot) {
				previousFinishTime = m_pSearchList[previousActionIndex].projectedFinishTime;

				if ((m_pSearchList[previousActionIndex].actionType == LIFT_ONE_RED_ROBOT) ||
					(m_pSearchList[previousActionIndex].actionType == LIFT_ONE_BLUE_ROBOT)) {
					//robot is lifted, not available for any other actions
					previousFinishTime = CLIMB_END_TIME + 1;
				}

				actionDonePos = m_pSearchList[previousActionIndex].actionDonePos;
				actionDoneWithCube = m_pSearchList[previousActionIndex].actionDoneWithCube;

				interruptFlag = false; //It is not the first action of the current branch.
				break; //the most recent finish time is the time to start the next action
			}

			previousActionIndex = m_pSearchList[previousActionIndex].previousIndex;
		} while (previousActionIndex != INVALID_IDX);

		if (previousFinishTime == CLIMB_END_TIME + 1) {
			continue; //a lifting action is planned, no action for this robot
		}
		if (previousFinishTime == 0) {
			startTime = previousFinishTime = currentTime;
		}

		for (int prevIdx = previousLayerStartIndex; prevIdx < previousLayerEndIndex; prevIdx++) {
			for (int act = startAction; act <= endRealAction; act++) {

				//the current action finish time
				finishTime = m_pRobots[robot].estimateActionDelayInSec((actionTypeType)act, previousFinishTime,
					interruptFlag, actionDonePos, actionDoneWithCube, &actionNewPos);
				finishTime += previousFinishTime;

				if (finishTime >= CLIMB_END_TIME) {
					//give up, do not save this action into the list
					continue;
				}

				//create a new action
				if (layerEndIndex >= m_maxSearchListSize) {
					printf("Error: search list too small\n");
				}
				else {
					m_pSearchList[layerEndIndex].actionType = (actionTypeType)act;
					m_pSearchList[layerEndIndex].previousIndex = prevIdx;
					m_pSearchList[layerEndIndex].projectedFinishTime = finishTime;
					m_pSearchList[layerEndIndex].actionDonePos = actionNewPos;
					m_pSearchList[layerEndIndex].startTime = previousFinishTime;
					m_pSearchList[layerEndIndex].projectedFinalScore = INT32_MIN;

					if (robot::isActionNeedCube((actionTypeType)act)) {
						m_pSearchList[layerEndIndex].actionDoneWithCube = false;
					}
					else {
						//action don't need a cube, pass cube flag to the next action
						m_pSearchList[layerEndIndex].actionDoneWithCube = actionDoneWithCube;
					}

					m_pSearchList[layerEndIndex].robotIndex = robot;
					m_pSearchList[layerEndIndex].actionIndex = actionIndexIn;
					layerEndIndex++;
					updetedTreeFlag = true;
				}
			}
		}

		//update layer indexes
		if (updetedTreeFlag) {
			previousLayerStartIndex = layerStartIndex;
			previousLayerEndIndex = layerEndIndex;
			updetedTreeFlag = false;
		}
	}
	
	if (previousLayerStartIndex != bestScoreIdx) {
		//After an action tree is created, search which action sequence can get the best score
		bestScoreIdx = findBestScoreBranch(previousLayerStartIndex, previousLayerEndIndex, actionIndexIn, &numPendingAction);
	}
	else {
		//no new actions added, no need to update bestScoreIdx;

		//find the number of actions on the best score branch
		numPendingAction = 0;
		previousActionIndex = bestScoreIdx;
		while (m_pSearchList[previousActionIndex].previousIndex != INVALID_IDX) {
			numPendingAction++;
			previousActionIndex = m_pSearchList[previousActionIndex].previousIndex;
		}
	}
	if ((bestScoreIdx == 0) || (numPendingAction == 0)) {
		printf("ERROR: cannot find any useful actions\n");
		searchFailedFlag = true;
	}

	//reset the output
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_bestAction[i].actionType = INVALID_ACTION;
		m_bestAction[i].startTime = currentTime;
		m_bestAction[i].projectedFinishTime = currentTime + 1;
		m_bestAction[i].actionIndex = actionIndexIn;
		m_bestAction[i].previousIndex = INVALID_IDX;
		m_bestAction[i].projectedFinalScore = 0;
		m_bestAction[i].robotIndex = i;
	}

	if (searchFailedFlag) {
		//printf("ERROR: cannot find any useful action, all idle\n");
	}
	else {
		//plan the beast action
		float earliestRobotTime[NUMBER_OF_ROBOTS];
		previousActionIndex = bestScoreIdx;

		for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
			earliestRobotTime[i] = CLIMB_END_TIME + 2;
		}
		//note: the initial value of earliest time must be later than the initial time of each action

		for (int i = numPendingAction-1; i >= 0; i--) {

			if (earliestRobotTime[m_pSearchList[previousActionIndex].robotIndex] >= m_pSearchList[previousActionIndex].projectedFinishTime) {
				//if finished time is the same, favor the action closer to root to match the score testing sequence above.
				memcpy(&m_bestAction[m_pSearchList[previousActionIndex].robotIndex], &m_pSearchList[previousActionIndex], sizeof(searchActionType));
				earliestRobotTime[m_pSearchList[previousActionIndex].robotIndex] = m_pSearchList[previousActionIndex].projectedFinishTime;
			}

			if (m_pSearchList[previousActionIndex].previousIndex == INVALID_IDX) {
				printf("Error: index calculation error\n");
			}
			else {
				previousActionIndex = m_pSearchList[previousActionIndex].previousIndex;
			}
		}
	}
}

void alliance::resetSearchList(void)
{
	for (int i = 0; i < m_maxSearchListSize; i++) {
		m_pSearchList[i].previousIndex = INVALID_IDX;
	}
}


int alliance::findBestScoreBranch(int startIdxIn, int stopIdxIn, int actionIndexIn, int *pBranchLengthOut)
{
	actionChainType actionChain[MAXIMUM_PENDING_ACTIONS + NUMBER_OF_ROBOTS];

	int bestScore = INT32_MIN;
	float bestScoreFinishTime = 0;
	int bestScoreIdx = 0;
	float lastFinishTime = 0;
	float earliestTime;
	int earliestIndex;
	int chainIndex;
	bool isActionRejectedFlag;
	bool assignedActionFlag;
	int previousActionIndex;
	int pendingIdx;
	int executedActionCount;
	bool firstActionFlag[NUMBER_OF_ROBOTS];
	int score, finalRedScore, finalBlueScore;

	*pBranchLengthOut = 0;

	//execute each action branch
	for (int exeIdx = startIdxIn; exeIdx < stopIdxIn; exeIdx++) {
		//previousLayerStartIndex to previousLayerEndIndex is the list of actions on the last search step

		//for each last step action, find out all actions before it
		previousActionIndex = exeIdx;
		pendingIdx = 0;
		while (m_pSearchList[previousActionIndex].previousIndex != INVALID_IDX) {

			if (pendingIdx >= MAXIMUM_PENDING_ACTIONS + NUMBER_OF_ROBOTS) {
				printf("Error: actionChain size too small, too many pending actions\n");
				break;
			}
			actionChain[pendingIdx].actionIndex = previousActionIndex;
			actionChain[pendingIdx].isActionExecutedFlag = 0;
			pendingIdx++;
			previousActionIndex = m_pSearchList[previousActionIndex].previousIndex;
		}

		if (pendingIdx == 0) {
			continue; //empty chine, it only has the initial no-action entry
		}

		//run all "pending" number of actions in time order
		m_testPlatForm = m_referencePlatForm;
		m_testPlatForm.setLogFile(m_pLogFIle);
		isActionRejectedFlag = false;
		executedActionCount = 0;

		for (int robot = 0; robot < NUMBER_OF_ROBOTS; robot++) {
			firstActionFlag[robot] = true;
		}

		while (executedActionCount < pendingIdx) {
			assignedActionFlag = false;
			for (int robot = 0; robot < NUMBER_OF_ROBOTS; robot++) {
				if (m_testPlatForm.hasPendingAction(robot, m_allianceType)) {
					if (!firstActionFlag[robot]) {
						//remaining action of the previous session could be interrupted.
						assignedActionFlag= true;
						continue;
					}
				}
				earliestTime = CLIMB_END_TIME + 2;
				//note: the initial value of earliest time must be later than the initial time of each action
				earliestIndex = 0;
				chainIndex = INVALID_IDX;

				//found the earliest action to execute
				for (int j = pendingIdx - 1; j >= 0; j--) {
					//actionChain is from root to future steps. If finished time is the same, favor the earlier step.
					if ((m_pSearchList[actionChain[j].actionIndex].projectedFinishTime < earliestTime) &&
						(actionChain[j].isActionExecutedFlag == 0) && 
						(m_pSearchList[actionChain[j].actionIndex].robotIndex == robot)) {
						chainIndex = j;
						earliestTime = m_pSearchList[actionChain[j].actionIndex].projectedFinishTime;
					}
				}

				if (chainIndex == INVALID_IDX) {
					//no action for this robot
				}
				else {
					actionChain[chainIndex].isActionExecutedFlag = 1;
					earliestIndex = actionChain[chainIndex].actionIndex;
					lastFinishTime = m_pSearchList[earliestIndex].projectedFinishTime;

					if (0 != m_testPlatForm.setRobotAction(&m_pSearchList[earliestIndex], m_allianceType, actionIndexIn)) {
						isActionRejectedFlag = true;
					}
					executedActionCount++;
					assignedActionFlag = true;
					firstActionFlag[robot] = false; //newly assigned action cannot be interrupted
				}
			}
			//after every robot take at most one action, execute all robots.
			if ((assignedActionFlag) && (!isActionRejectedFlag)) {
				if (0 != m_testPlatForm.commitAction(actionIndexIn)) {
					isActionRejectedFlag = true;
					break;
				}
			}
			else if(isActionRejectedFlag) {
				break; //no reason to continue
			}
			else {
				printf("ERROR: not all tasks are done but no task to execute\n");
			}
		}

		//finish all pending actions
		while ((!isActionRejectedFlag) && (m_testPlatForm.hasPendingActions())) {
			if (0 != m_testPlatForm.commitAction(actionIndexIn)) {
				isActionRejectedFlag = true;
				break;
			}
		}

		if (isActionRejectedFlag) {
			score = INT32_MIN;
		}
		else {
			m_testPlatForm.getFinalScore(&finalRedScore, &finalBlueScore);
			if (m_allianceType == ALLIANCE_RED) {
				score = finalRedScore - finalBlueScore;
			}
			else {
				score = finalBlueScore - finalRedScore;
			}
		}

		//save the final score with the last pending action entry
		m_pSearchList[exeIdx].projectedFinalScore = score;
		m_testPlatForm.logFinalScore();

		if (score > bestScore) {
			bestScoreIdx = actionChain[0].actionIndex;
			bestScore = score;
			bestScoreFinishTime = lastFinishTime;
			*pBranchLengthOut = pendingIdx;
		}
		else if ((score == bestScore) && (bestScoreFinishTime > lastFinishTime)) {
			//favor the same score and finished earlier
			bestScoreIdx = actionChain[0].actionIndex;
			bestScore = score;
			bestScoreFinishTime = lastFinishTime;
			*pBranchLengthOut = pendingIdx;
		}
	}
	return bestScoreIdx;
}

