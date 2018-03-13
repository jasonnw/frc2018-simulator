#pragma once

#include <windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "platform.h"

using namespace cv;
using namespace std;

template <class T> class messageQueue {
private:
	CRITICAL_SECTION   m_bufferLock;
	CONDITION_VARIABLE m_senderCondVar;
	CONDITION_VARIABLE m_recieverCondVar;

	T *m_pQueueBuffer;
	bool *m_pIsEmptyFlag;
	int m_queueSize;
	int m_readIdx;  //the next read message index
	int m_writeIdx; //the next write message index

public:
	messageQueue(int qSizeIn, int *pErrorOut)
	{
		*pErrorOut = -1;
		m_queueSize = 0;
		m_readIdx = 0;
		m_writeIdx = 0;

		m_pQueueBuffer = (T*)malloc(sizeof(T) * qSizeIn + sizeof(bool) * qSizeIn);
		if (m_pQueueBuffer != NULL) {
			m_queueSize = qSizeIn;
			*pErrorOut = 0;
			InitializeCriticalSection(&m_bufferLock);
			InitializeConditionVariable(&m_senderCondVar);
			InitializeConditionVariable(&m_recieverCondVar);

			m_pIsEmptyFlag = (bool*) &m_pQueueBuffer[qSizeIn];
			for (int i = 0; i < qSizeIn; i++) {
				m_pIsEmptyFlag[i] = true;
			}
		}
		else {
			*pErrorOut = -1;
		}

	}
	~messageQueue()
	{
		if (m_pQueueBuffer != NULL) {
			free(m_pQueueBuffer);
			DeleteCriticalSection(&m_bufferLock);
		}
	}

	bool isQueueEmpty(void)
	{
		bool isEmpty = false;
		EnterCriticalSection(&m_bufferLock);

		if (m_pIsEmptyFlag[m_readIdx]) {
			isEmpty = true;
		}

		LeaveCriticalSection(&m_bufferLock);
		return isEmpty;
	}

	bool isQueueFull(void)
	{
		bool isFull = false;
		EnterCriticalSection(&m_bufferLock);

		if (!m_pIsEmptyFlag[m_writeIdx]) {
			isFull = true;
		}

		LeaveCriticalSection(&m_bufferLock);
		return isFull;
	}


	void send(const T *pMessageIn)
	{
		EnterCriticalSection(&m_bufferLock);

		while (!m_pIsEmptyFlag[m_writeIdx]) {
			//Q is full
			SleepConditionVariableCS(&m_recieverCondVar, &m_bufferLock, INFINITE);
		}

		memcpy(&m_pQueueBuffer[m_writeIdx], pMessageIn, sizeof(T));
		m_pIsEmptyFlag[m_writeIdx] = false;
		m_writeIdx++;
		if (m_writeIdx >= m_queueSize) {
			m_writeIdx = 0;
		}

		LeaveCriticalSection(&m_bufferLock);
		WakeConditionVariable(&m_senderCondVar);
	}

	void receive(T* pMessageOut)
	{
		EnterCriticalSection(&m_bufferLock);

		while (m_pIsEmptyFlag[m_readIdx]) {
			//Q is empty
			SleepConditionVariableCS(&m_senderCondVar, &m_bufferLock, INFINITE);
		}

		memcpy(pMessageOut, &m_pQueueBuffer[m_readIdx], sizeof(T));
		m_pIsEmptyFlag[m_readIdx] = true;
		m_readIdx++;
		if (m_readIdx >= m_queueSize) {
			m_readIdx = 0;
		}

		LeaveCriticalSection(&m_bufferLock);
		WakeConditionVariable(&m_recieverCondVar);
	}

	int trySend(const T *pMessageIn)
	{
		EnterCriticalSection(&m_bufferLock);

		if (!m_pIsEmptyFlag[m_writeIdx]) {
			LeaveCriticalSection(&m_bufferLock);
			return -1;
		}
		else {
			memcpy(&m_pQueueBuffer[m_writeIdx], pMessageIn, sizeof(T));
			m_pIsEmptyFlag[m_writeIdx] = false;
			m_writeIdx++;
			if (m_writeIdx >= m_queueSize) {
				m_writeIdx = 0;
			}

			LeaveCriticalSection(&m_bufferLock);
			WakeConditionVariable(&m_senderCondVar);
			return 0;
		}
	}

	int tryReceive(T* pMessageOut)
	{
		EnterCriticalSection(&m_bufferLock);

		if (m_pIsEmptyFlag[m_readIdx]) {
			//Q is empty
			LeaveCriticalSection(&m_bufferLock);
			return -1;
		}
		else {
			memcpy(pMessageOut, &m_pQueueBuffer[m_readIdx], sizeof(T));
			m_pIsEmptyFlag[m_readIdx] = true;
			m_readIdx++;
			if (m_readIdx >= m_queueSize) {
				m_readIdx = 0;
			}

			LeaveCriticalSection(&m_bufferLock);
			WakeConditionVariable(&m_recieverCondVar);
			return 0;
		}
	}
};

const int MESSAGE_QUEUE_DEPTH = 20;
const int PLATFORM_RESOLUSION_X = 1280;
const int PLATFORM_RESOLUSION_y = 800;

typedef struct actionMessageType {
	pendingActionType action;
	allianceType alliance;
	coordinateType startPos;
	int cubeIdx;
	int actionIndex;
	int robotIdx;
	bool commitActionFlag;
	bool quitFlag;
}actionMessageType;


class displayPlatform :	public platform
{
private:
	messageQueue <actionMessageType> *m_pQueue;
	Mat *m_pPlatform;
	rectangleObjectType m_redRobotsOldPos[NUMBER_OF_ROBOTS];
	rectangleObjectType m_blueRobotsOldPos[NUMBER_OF_ROBOTS];

public:
	displayPlatform();
	~displayPlatform();


	int updatePlatform(int actionIndexIn);
	//receive message and update platform state

	int sendAction(const actionMessageType *pActionIn);
	void drawPlatform(int delayIn);

protected:
	Point coordinateToPoint(double xIn, double yIn);
	void updateField(void);
	void playTotheNextTime(double nextTimeIn, int actionIndexIn, double delayIn);

	void drawCube(coordinateType positionIn, int indexIn);
	void drawButton(coordinateType positionIn, int pushStateIn, const Scalar& colorIn);
	void drawObject(const rectangleObjectType *pObjectIn);
	void drawRobot(const rectangleObjectType *pObjectIn, int robotIdxIn, bool hasCubeFlagIn);
	void drawInteger(const rectangleObjectType *pObjectIn, int numberIn, const char *strIn, double sizeIn, cv::Scalar colorIn);
	void drawFloat(const rectangleObjectType *pObjectIn, double numberIn, const char *strIn, double sizeIn, cv::Scalar colorIn);
	void drawField(void);
};

