
//#include "stdafx.h"
#include <stdlib.h>
#include "alliance.h"

alliance::alliance()
{
	int searchSize;

	m_timeInSec = 0;
	m_pLogFIle = NULL;

	//reset best action to an invalid time point
	memset(&m_bestAction, 0, sizeof(m_bestAction));
	m_bestAction.actionType = RED_ACTION_NONE;
	//Set action time as 0, platform will catch this error later  

	searchSize = NUM_OF_POSSIBLE_ACTIONS;
	m_maxSearchListSize = searchSize;
	for (int i = 1; i < MAXIMUM_PENDING_ACTIONS; i++) {
		searchSize *= NUM_OF_POSSIBLE_ACTIONS;
		m_maxSearchListSize += searchSize;
	}

	m_maxSearchListSize++; //add the first initial action
	m_pSearchList = (pendingActionType*) malloc(m_maxSearchListSize * sizeof(pendingActionType));
}


alliance::~alliance()
{
	if (m_pSearchList != NULL) {
		free(m_pSearchList);
	}
}

int alliance::initAlliance(allianceType typeIn, FILE *pLogFile,
	const robotConfigurationType config1In[NUMBER_OF_ROBOTS])
{
	if (m_pSearchList == NULL) {
		return -1;
	}
	resetSearchList();
	m_pLogFIle = pLogFile;
	m_allianceType = typeIn;
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_robot[i].setConfiguration(&config1In[i]);
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
	actionChainType actionChain[MAXIMUM_PENDING_ACTIONS];

	bool isActionRejectedFlag;
	bool firstActionAfterUpdateFlag;
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

	int bestScore, score;
	int finalBlueScore, finalRedScore;
	int bestScoreIdx;
	int bestNoActionCount, noActionCount;
	int projectedRedScore, projectedBlueScore, projectedScore;

	float earliestTime;
	int earliestIndex, chainIndex;

	bool isRealActionFlag; 

	actionTypeType startAction = (m_allianceType == ALLIANCE_RED) ? CUBE_RED_OFFENCE_SWITCH : CUBE_BLUE_OFFENCE_SWITCH;
	actionTypeType endAction = (m_allianceType == ALLIANCE_RED) ? RED_ACTION_NONE : BLUE_ACTION_NONE;

	//reset best action
	memset(&m_bestAction, 0, sizeof(m_bestAction));
	m_bestAction.actionType = endAction;
	m_bestAction.projectedFinishTime = 0;
	//Set action time as 0, platform will catch this error later  

	//create the first action entry as the root of all actions
	previousLayerStartIndex = 0;
	previousLayerEndIndex = 1;
	m_pSearchList[previousLayerStartIndex].actionType = endAction;
	m_pSearchList[previousLayerStartIndex].projectedFinalScore = 0; //entry not used
	m_pSearchList[previousLayerStartIndex].previousIndex = UINT32_MAX;
	m_pSearchList[previousLayerStartIndex].projectedFinishTime = m_referencePlatForm.getTime();
	m_pSearchList[previousLayerStartIndex].robotIndex = INDEX_OF_ROBOT_NONE;
	//Note: this root action will not be used in real action list. It is a start point for action searching.
	
	searchFailedFlag = false;

	//project the final score if no action is taken
	testPlatForm = m_referencePlatForm;
	testPlatForm.getFinalScore(&projectedRedScore, &projectedBlueScore);
	projectedScore = (m_allianceType == ALLIANCE_RED) ? projectedRedScore - projectedBlueScore : projectedBlueScore - projectedRedScore;

	//build the search list
	for (int pending = 0; pending < MAXIMUM_PENDING_ACTIONS; pending++) {
		//search MAXIMUM_PENDING_ACTIONS number of steps of all possible actions 

		layerEndIndex = layerStartIndex = previousLayerEndIndex;
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
					firstActionAfterUpdateFlag = false;

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
						} while (previousActionIndex != UINT32_MAX);

						if (previousFinishTime == 0) {
							previousFinishTime = m_referencePlatForm.getTime();
							firstActionAfterUpdateFlag = true;
							//the robot can start right after the decision is made.
							//it may continue run the previous decision
						}
					}

					//the current action finish time
					finishTime = m_robot[robot].getActionDelayInSec((actionTypeType)act, previousFinishTime, firstActionAfterUpdateFlag);
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
					layerEndIndex++;
				}
			}
		}

		//update layer indexes
		previousLayerStartIndex = layerStartIndex;
		previousLayerEndIndex = layerEndIndex;

		if (previousLayerStartIndex == previousLayerEndIndex) {
			//cannot find a good action
			printf("Error: SEARCH_CONTINUE_THRESHOLD %d is too big, cannot find a good action\n", SEARCH_CONTINUE_THRESHOLD);

			//give up search and mark the best action as NO_ACTION
			searchFailedFlag = true;
			break;
		}

		//After an action tree is created, search which action sequence can get the best score
		bestScore = INT32_MIN;
		bestNoActionCount = 0;
		bestScoreIdx = 0;

		//execute each action branch
		for (int exeIdx = previousLayerStartIndex; exeIdx < previousLayerEndIndex; exeIdx++) {
			//previousLayerStartIndex to previousLayerEndIndex is the list of actions on the last search step

			//for each last step action, find out all MAXIMUM_PENDING_ACTIONS number of actions before it
			previousActionIndex = exeIdx;
			for (int i = pending; i >= 0; i--) {
				actionChain[i].actionIndex = previousActionIndex;
				actionChain[i].isActionExecutedFlag = 0;
				if (m_pSearchList[previousActionIndex].previousIndex == UINT32_MAX) {
					printf("Error: index calculation error\n");
				}
				else {
					previousActionIndex = m_pSearchList[previousActionIndex].previousIndex;
				}
			}

			//run all "pending" number of actions in time order
			testPlatForm = m_referencePlatForm;
			testPlatForm.setLogFIle(m_pLogFIle);
			isActionRejectedFlag = false;
			noActionCount = 0;
			for (int i = 0; i <= pending; i++) {
				earliestTime = CLIMB_END_TIME + 2;
				//note: the initial value of earliest time must be later than the initial time of each action
				earliestIndex = 0;
				chainIndex = INT32_MAX;

				//found the earliest action to execute
				for (int j = 0; j <= pending; j++) {
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
				if (m_pSearchList[earliestIndex].actionType == endAction) {
					noActionCount++;
				}
				if (0 != testPlatForm.takeAction(m_pSearchList[earliestIndex].actionType,
					m_pSearchList[earliestIndex].projectedFinishTime,
					m_pSearchList[earliestIndex].robotIndex,
					actionIndexIn)) {
					isActionRejectedFlag = true;
				}
			}

			if (isActionRejectedFlag) {
				score = INT32_MIN;
			}
			else {
				testPlatForm.getFinalScore(&finalRedScore, &finalBlueScore);
				if (m_allianceType == ALLIANCE_RED) {
					score = finalRedScore - finalBlueScore;
				}
				else {
					score = finalBlueScore - finalRedScore;
				}
			}

			//save the final score with the last pending action entry
			m_pSearchList[exeIdx].projectedFinalScore = score;

			testPlatForm.logFinalScore();

			if (score > bestScore) {
				bestScoreIdx = actionChain[pending].actionIndex;
				bestScore = score;
				bestNoActionCount = noActionCount;
			}
			else if ((score == bestScore) && (noActionCount > bestNoActionCount)) {
				//favor the same score without action
				bestScoreIdx = actionChain[pending].actionIndex;
				bestScore = score;
				bestNoActionCount = noActionCount;
			}
		}

		//bestScoreIdx of the last pending action iteration is the best action list of all
	}

	//find the best next action and run the action
	//re-plan robot action
	for (int i = 0; i < NUMBER_OF_ROBOTS; i++) {
		m_robot[i].resetPreviousPlannedAction();
	}

	if (searchFailedFlag) {
		m_bestAction.actionType = endAction;
		m_bestAction.projectedFinalScore = 0; //entry not used
		m_bestAction.previousIndex = UINT32_MAX;
		m_bestAction.projectedFinishTime = m_referencePlatForm.getTime() + 1;
		m_bestAction.robotIndex = INDEX_OF_ROBOT_NONE;
	}
	else {
		//plan the beast action
		previousActionIndex = bestScoreIdx;
		earliestTime = CLIMB_END_TIME + 2;
		//note: the initial value of earliest time must be later than the initial time of each action

		for (int i = MAXIMUM_PENDING_ACTIONS - 1; i >= 0; i--) {

			if (earliestTime >= m_pSearchList[previousActionIndex].projectedFinishTime) {
				//if finished time is the same, favor the action closer to root to match the score testing sequence above.
				memcpy(&m_bestAction, &m_pSearchList[previousActionIndex], sizeof(m_bestAction));
				earliestTime = m_pSearchList[previousActionIndex].projectedFinishTime;
			}

			//one alliance submits the first action to the game platform and let other robots to carry out following action
			//in the case that the following action in the same sequence is picked as the next action, the delay is shorter.
			m_robot[m_pSearchList[previousActionIndex].robotIndex].setPreviousPlannedAction(&m_pSearchList[previousActionIndex]);

			if (m_pSearchList[previousActionIndex].previousIndex == UINT32_MAX) {
				printf("Error: index calculation error\n");
			}
			else {
				previousActionIndex = m_pSearchList[previousActionIndex].previousIndex;
			}
		}

		if (m_bestAction.robotIndex >= NUMBER_OF_ROBOTS) {
			printf("Error: robot index %d overflow\n", m_bestAction.robotIndex);
		}

		//if one robot executed the submitted action, it no longer has continue action for the next step
		m_robot[m_bestAction.robotIndex].resetPreviousPlannedAction();
	}
}

void alliance::resetSearchList(void)
{
	for (int i = 0; i < m_maxSearchListSize; i++) {
		m_pSearchList[i].previousIndex = UINT32_MAX;
	}
}