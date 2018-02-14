
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
		m_bestAction[i].actionType = RED_ACTION_NONE;
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
	robotPathType actionPath;
	bool searchFailedFlag;

	int previousActionIndex;

	float bestFinishTime;
	float finishTime;
	float previousFinishTime;

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

	actionTypeType startAction = (m_allianceType == ALLIANCE_RED) ? CUBE_RED_OFFENCE_SWITCH : CUBE_BLUE_OFFENCE_SWITCH;
	actionTypeType endAction = (m_allianceType == ALLIANCE_RED) ? PUSH_RED_LIFT_BUTTON : PUSH_BLUE_LIFT_BUTTON;
	actionTypeType endRealAction = (m_allianceType == ALLIANCE_RED) ? RED_ACTION_NONE : BLUE_ACTION_NONE;

	//reset best action
	memset(m_bestAction, 0, sizeof(m_bestAction));
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_bestAction[i].actionType = endAction;
		m_bestAction[i].projectedFinishTime = 0;
	}
	//Set action time as 0, platform will catch this error later  

	//create the first action entry as the root of all actions
	previousLayerStartIndex = 0;
	previousLayerEndIndex = 1;
	m_pSearchList[previousLayerStartIndex].actionType = endAction;
	m_pSearchList[previousLayerStartIndex].projectedFinalScore = 0; //entry not used
	m_pSearchList[previousLayerStartIndex].previousIndex = INT32_MAX;
	m_pSearchList[previousLayerStartIndex].projectedFinishTime = m_referencePlatForm.getTime();
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
								break; //the most recent finish time is the time to start the next action
							}

							previousActionIndex = m_pSearchList[previousActionIndex].previousIndex;
						} while (previousActionIndex != INT32_MAX);

						if (previousFinishTime == 0) {
							previousFinishTime = m_referencePlatForm.getTime();
						}
					}

					//the current action finish time
					finishTime = m_pRobots[robot].getActionDelayInSec((actionTypeType)act, previousFinishTime, &actionPath);
					finishTime += previousFinishTime;

					if (finishTime < bestFinishTime) {
						bestFinishTime = finishTime;
						bestFinishRobotIdx = robot;
					}
				}

				//create a new action
				if (layerEndIndex >= m_maxSearchListSize) {
					printf("Error: search list too small\n");
				}
				else {
					m_pSearchList[layerEndIndex].actionType = (actionTypeType)act;
					m_pSearchList[layerEndIndex].previousIndex = prevIdx;
					m_pSearchList[layerEndIndex].projectedFinishTime = bestFinishTime;

					m_pSearchList[layerEndIndex].robotIndex = bestFinishRobotIdx;
					updetedTreeFlag = true;
					layerEndIndex++;
				}
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

		do {
			//the previous action finish time
			if (m_pSearchList[previousActionIndex].robotIndex == robot) {
				previousFinishTime = m_pSearchList[previousActionIndex].projectedFinishTime;

				if ((m_pSearchList[previousActionIndex].actionType == LIFT_ONE_RED_ROBOT) ||
					(m_pSearchList[previousActionIndex].actionType == LIFT_ONE_BLUE_ROBOT)) {
					//robot is lifted, not available for any other actions
					previousFinishTime = CLIMB_END_TIME + 1;
				}
				break; //the most recent finish time is the time to start the next action
			}

			previousActionIndex = m_pSearchList[previousActionIndex].previousIndex;
		} while (previousActionIndex != INT32_MAX);

		if (previousFinishTime == CLIMB_END_TIME + 1) {
			continue; //a lifting action is planned, no action for this robot
		}
		if (previousFinishTime == 0) {
			previousFinishTime = m_referencePlatForm.getTime();
		}

		for (int prevIdx = previousLayerStartIndex; prevIdx < previousLayerEndIndex; prevIdx++) {
			for (int act = startAction; act <= endRealAction; act++) {

				//the current action finish time
				finishTime = m_pRobots[robot].getActionDelayInSec((actionTypeType)act, previousFinishTime, &actionPath);
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

					m_pSearchList[layerEndIndex].robotIndex = robot;
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
		while (m_pSearchList[previousActionIndex].previousIndex != INT32_MAX) {
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
		m_bestAction[i].actionType = endAction; //no action
		m_bestAction[i].startTime = m_referencePlatForm.getTime();
		m_bestAction[i].projectedFinishTime = m_referencePlatForm.getTime() + 1;
		m_bestAction[i].actionIndex = actionIndexIn;
		m_bestAction[i].previousIndex = INT32_MAX;
		m_bestAction[i].projectedFinalScore = 0;
		m_bestAction[i].robotIndex = 1;
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

			if (m_pSearchList[previousActionIndex].previousIndex == INT32_MAX) {
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
		m_pSearchList[i].previousIndex = INT32_MAX;
	}
}


int alliance::findBestScoreBranch(int startIdxIN, int stopIdxIn, int actionIndexIn, int *pBranchLengthOut)
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
	int previousActionIndex;
	int pendingIdx;
	int score, finalRedScore, finalBlueScore;

	*pBranchLengthOut = 0;

	//execute each action branch
	for (int exeIdx = startIdxIN; exeIdx < stopIdxIn; exeIdx++) {
		//previousLayerStartIndex to previousLayerEndIndex is the list of actions on the last search step

		//for each last step action, find out all MAXIMUM_PENDING_ACTIONS number of actions before it
		previousActionIndex = exeIdx;
		pendingIdx = 0;
		while (m_pSearchList[previousActionIndex].previousIndex != INT32_MAX) {

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
		m_testPlatForm.setLogFIle(m_pLogFIle);
		isActionRejectedFlag = false;
		for (int i = pendingIdx-1; i >= 0; i--) {
			earliestTime = CLIMB_END_TIME + 2;
			//note: the initial value of earliest time must be later than the initial time of each action
			earliestIndex = 0;
			chainIndex = INT32_MAX;

			//found the earliest action to execute
			for (int j = pendingIdx - 1; j >= 0; j--) {
				//actionChain is from root to future steps. If finished time is the same, favor the earlier step.
				if ((m_pSearchList[actionChain[j].actionIndex].projectedFinishTime < earliestTime) &&
					(actionChain[j].isActionExecutedFlag == 0)) {
					chainIndex = j;
					earliestTime = m_pSearchList[actionChain[j].actionIndex].projectedFinishTime;
				}
			}
			//Note: even the action is created in step order, because some robots may finish an action 
			//before the previous step is done. But, platform only accept input actions in time order.
			//The loop above make sure always the earliest finish action is picked first and each action 
			//is only executed once.
			if (chainIndex == INT32_MAX) {
				printf("Error: cannot find an earliest action\n");
			}
			actionChain[chainIndex].isActionExecutedFlag = 1;
			earliestIndex = actionChain[chainIndex].actionIndex;
			lastFinishTime = m_pSearchList[earliestIndex].projectedFinishTime;

			if (0 != m_testPlatForm.commitAction(m_pSearchList[earliestIndex].actionType,
				m_pSearchList[earliestIndex].projectedFinishTime,
				m_pSearchList[earliestIndex].robotIndex,
				m_allianceType,
				actionIndexIn)) {
				isActionRejectedFlag = true;
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

