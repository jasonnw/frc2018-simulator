#pragma once

#include <stdint.h>
#include "config.h"
#include "robot.h"
#include "platform.h"

typedef struct actionChainType {
	int actionIndex;
	int isActionExecutedFlag;
}actionChainType;

class alliance
{
private:
	platform m_testPlatForm;
	allianceType m_allianceType;
	float m_timeInSec;
	platform m_referencePlatForm;
	const robot *m_pRobots;
	searchActionType m_bestAction[NUMBER_OF_ROBOTS];

	searchActionType *m_pSearchList;
	int m_maxSearchListSize;
	FILE *m_pLogFIle;

public:
	alliance();
	~alliance();

	int initAlliance(allianceType typeIn, FILE *pLogFile,
		const robotConfigurationType config1In[NUMBER_OF_ROBOTS],
		const platform &platformIn);

	void setLogFile(FILE *pFile)
	{
		m_pLogFIle = pFile;
	}

	void syncLocalPlatform(const platform &platformIn, int actionIndexIn);

	void getBestAction(searchActionType *pActionOut)
	{
		memcpy(pActionOut, m_bestAction, sizeof(searchActionType)*NUMBER_OF_ROBOTS);
	}

protected:
	void findBestAction(int actionIndexIn);
	void resetSearchList(void);
	int findBestScoreBranch(int startIdxIn, int stopIdxIn, int actionIndexIn, int *pBranchLengthOut);
};

