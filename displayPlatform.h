#pragma once

#include "os_wrapper.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include "platform.h"

using namespace cv;
using namespace std;

template <class T> class messageQueue {
private:
	pthread_mutex_t   m_bufferLock;
	pthread_cond_t m_senderCondVar;
	pthread_cond_t m_recieverCondVar;

	T *m_pQueueBuffer;
	bool *m_pIsEmptyFlag;
	int m_queueSize;
	int m_readIdx;  //the next read message index
	int m_writeIdx; //the next write message index

public:
	messageQueue(int qSizeIn, int *pErrorOut)
	{
#ifndef _MSC_VER
		pthread_mutexattr_t mutexAttr;
		pthread_condattr_t condvAttr;
#endif

		*pErrorOut = -1;
		m_queueSize = 0;
		m_readIdx = 0;
		m_writeIdx = 0;

		m_pQueueBuffer = (T*)malloc(sizeof(T) * qSizeIn + sizeof(bool) * qSizeIn);
		if (m_pQueueBuffer != NULL) {
			m_queueSize = qSizeIn;
			*pErrorOut = 0;
			pthread_mutexattr_init(&mutexAttr);
			pthread_mutex_init(&m_bufferLock, &mutexAttr);

			pthread_condattr_init(&condvAttr);
			pthread_cond_init(&m_senderCondVar, &condvAttr);
			pthread_cond_init(&m_recieverCondVar, &condvAttr);

			pthread_mutexattr_destroy(&mutexAttr);
			pthread_condattr_destroy(&condvAttr);

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
			pthread_mutex_destroy(&m_bufferLock);
			pthread_cond_destroy(&m_senderCondVar);
			pthread_cond_destroy(&m_recieverCondVar);
		}
	}

	bool isQueueEmpty(void)
	{
		bool isEmpty = false;
		pthread_mutex_lock(&m_bufferLock);

		if (m_pIsEmptyFlag[m_readIdx]) {
			isEmpty = true;
		}

		pthread_mutex_unlock(&m_bufferLock);
		return isEmpty;
	}

	bool isQueueFull(void)
	{
		bool isFull = false;
		pthread_mutex_lock(&m_bufferLock);

		if (!m_pIsEmptyFlag[m_writeIdx]) {
			isFull = true;
		}

		pthread_mutex_unlock(&m_bufferLock);
		return isFull;
	}


	void send(const T *pMessageIn)
	{
		pthread_mutex_lock(&m_bufferLock);

		while (!m_pIsEmptyFlag[m_writeIdx]) {
			//Q is full
			pthread_cond_wait(&m_recieverCondVar, &m_bufferLock);
		}

		memcpy(&m_pQueueBuffer[m_writeIdx], pMessageIn, sizeof(T));
		m_pIsEmptyFlag[m_writeIdx] = false;
		m_writeIdx++;
		if (m_writeIdx >= m_queueSize) {
			m_writeIdx = 0;
		}

		pthread_mutex_unlock(&m_bufferLock);
		pthread_cond_signal(&m_senderCondVar);
	}

	void receive(T* pMessageOut)
	{
		pthread_mutex_lock(&m_bufferLock);

		while (m_pIsEmptyFlag[m_readIdx]) {
			//Q is empty
			pthread_cond_wait(&m_senderCondVar, &m_bufferLock);
		}

		memcpy(pMessageOut, &m_pQueueBuffer[m_readIdx], sizeof(T));
		m_pIsEmptyFlag[m_readIdx] = true;
		m_readIdx++;
		if (m_readIdx >= m_queueSize) {
			m_readIdx = 0;
		}

		pthread_mutex_unlock(&m_bufferLock);
		pthread_cond_signal(&m_recieverCondVar);
	}

	int trySend(const T *pMessageIn)
	{
		pthread_mutex_lock(&m_bufferLock);

		if (!m_pIsEmptyFlag[m_writeIdx]) {
			pthread_mutex_unlock(&m_bufferLock);
			return -1;
		}
		else {
			memcpy(&m_pQueueBuffer[m_writeIdx], pMessageIn, sizeof(T));
			m_pIsEmptyFlag[m_writeIdx] = false;
			m_writeIdx++;
			if (m_writeIdx >= m_queueSize) {
				m_writeIdx = 0;
			}

			pthread_mutex_unlock(&m_bufferLock);
			pthread_cond_signal(&m_senderCondVar);
			return 0;
		}
	}

	int tryReceive(T* pMessageOut)
	{
		pthread_mutex_lock(&m_bufferLock);

		if (m_pIsEmptyFlag[m_readIdx]) {
			//Q is empty
			pthread_mutex_unlock(&m_bufferLock);
			return -1;
		}
		else {
			memcpy(pMessageOut, &m_pQueueBuffer[m_readIdx], sizeof(T));
			m_pIsEmptyFlag[m_readIdx] = true;
			m_readIdx++;
			if (m_readIdx >= m_queueSize) {
				m_readIdx = 0;
			}

			pthread_mutex_unlock(&m_bufferLock);
			pthread_cond_signal(&m_recieverCondVar);
			return 0;
		}
	}
};

const int MESSAGE_QUEUE_DEPTH = 1;
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
	cv::Scalar m_wallColor;

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
	void drawButton(coordinateType positionIn, int pushStateIn, const Scalar& colorIn, int radiusIn = 10);
	void drawObject(const rectangleObjectType *pObjectIn, bool hightLightFlagIn);
	void drawRobot(const rectangleObjectType *pObjectIn, allianceType allianceIn, int robotIdxIn, bool hasCubeFlagIn);
	void drawInteger(const rectangleObjectType *pObjectIn, int numberIn, const char *strIn, double sizeIn, cv::Scalar colorIn);
	void drawFloat(const rectangleObjectType *pObjectIn, double numberIn, const char *strIn, double sizeIn, cv::Scalar colorIn);
	void drawString(const rectangleObjectType *pObjectIn, const char *strIn, double sizeIn, cv::Scalar colorIn);
	void drawField(void);
};

