#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "config.h"

typedef enum allianceType {
	ALLIANCE_RED,
	ALLIANCE_BLUE
}allianceType;


class platform
{
private:
	platformStateType m_state;
	float m_timeInSec; 
	int m_redScore;
	int m_blueScore;
	int m_liftRedRobotIndex;
	int m_liftBlueRobotIndex;
	FILE *m_pLogFIle;

public:
	platform();
	~platform();

	const platformStateType * getState(void) const { return &m_state; }
	float getTime(void) const { return m_timeInSec; }
	int getRedScore(void) const { return m_redScore; }
	int getBlueScore(void) const { return m_blueScore; }

	void setState(const platformStateType *pStateIn) { memcpy(&m_state, pStateIn,  sizeof(m_state)); }
	void setTime(float timeIn) { m_timeInSec = timeIn; }
	void setRedScore(int redScoreIn) { m_redScore = redScoreIn; }
	void setBlueScore(int blueScoreIn) { m_blueScore = blueScoreIn; }
	void setLogFIle(FILE *pFileIn) { m_pLogFIle = pFileIn; }

	const platform & operator = (const platform &srcIn)
	{
		setState(srcIn.getState());
		setTime(srcIn.getTime());
		setRedScore(srcIn.getRedScore());
		setBlueScore(srcIn.getBlueScore());

		return srcIn;
	}

	int takeAction(actionTypeType actionIn, float timeIn, int robotIndexIn, int indexIn);

	int isGameTimeOver(void);
	void getFinalScore(int *pRedScoreOut, int *pBlueScoreOut);
	void logFinalScore(void);
	bool isRobotLifted(allianceType allianceIn, int robotIdxIn);

protected:
	void updateScore(float secondsIn);
	int updateScaleSwitchScore(float secondsIn, int vaultForceBlockCountIn, int vaultBoostBlockCountIn, int balanceBlockDifferenceIn,
		vaultButtonStateType forceVaultButtonIn, vaultButtonStateType boostVaultButtonIn,
		int vaultBlockSelectionIn, ownerShipType newOnerShipIn, ownerShipType *pOwnerShipInOut);

	void logAction(actionTypeType actionIn, float timeIn, int robotIndexIn, int indexIn);

};

