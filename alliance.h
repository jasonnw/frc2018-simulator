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
	allianceType m_allianceType;
	float m_timeInSec;
	platform m_referencePlatForm;
	robot m_robot[NUMBER_OF_ROBOTS];
	pendingActionType m_bestAction;

	pendingActionType *m_pSearchList;
	int m_maxSearchListSize;
	FILE *m_pLogFIle;

public:
	alliance();
	~alliance();

	int initAlliance(allianceType typeIn, FILE *pLogFile,
		const robotConfigurationType config1In[NUMBER_OF_ROBOTS]);

	void setLogFile(FILE *pFile)
	{
		m_pLogFIle = pFile;
	}

	void syncLocalPlatform(const platform &platformIn, int actionIndexIn);

	void getBestAction(pendingActionType *pActionOut)
	{
		memcpy(pActionOut, &m_bestAction, sizeof(pendingActionType));
	}

protected:
	void findBestAction(int actionIndexIn);
	void resetSearchList(void);
};

