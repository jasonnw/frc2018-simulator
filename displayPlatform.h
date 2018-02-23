#pragma once

#include <windows.h>
#include "platform.h"

template <class T> class messageQueue {
private:
	CRITICAL_SECTION   m_bufferLock;
	CONDITION_VARIABLE m_senderCondVar;
	CONDITION_VARIABLE m_recieverCondVar;

	T *m_pQueueBuffer;
	int m_queueSize;
	int m_readIdx;  //the last read message index
	int m_writeIdx; //the last write message index
	bool m_writeWrapAroundFlag;

public:
	messageQueue(int qSizeIn, int *pErrorOut)
	{
		*pErrorOut = -1;
		m_queueSize = 0;
		m_readIdx = -1;
		m_writeIdx = -1;
		m_writeWrapAroundFlag = false;

		m_pQueueBuffer = (T*)malloc(sizeof(T) * qSizeIn);
		if (m_pQueueBuffer != NULL) {
			m_queueSize = qSizeIn;
			*pErrorOut = 0;
		}
	}
	~messageQueue()
	{
		if (m_pQueueBuffer != NULL) {
			free(m_pQueueBuffer);
		}
	}

	bool isQueueEmpty(void)
	{
		bool isEmpty = false;
		EnterCriticalSection(&m_bufferLock);

		if ((m_writeIdx <= m_readIdx) && (m_writeWrapAroundFlag == false)) {
			isEmpty = true;
		}

		LeaveCriticalSection(&m_bufferLock);
		return isEmpty;
	}

	bool isQueueFull(void)
	{
		bool isFull = false;
		EnterCriticalSection(&m_bufferLock);

		if ((m_writeIdx >= m_readIdx) && (m_writeWrapAroundFlag == true)) {
			isFull = true;
		}

		LeaveCriticalSection(&m_bufferLock);
		return isFull;
	}


	void puch(const T *pMessageIn)
	{
		EnterCriticalSection(&m_bufferLock);

		int writeIndex = m_writeIdx + 1;
		if (writeIndex >= m_queueSize) {
			writeIndex = 0;
			m_writeWrapAroundFlag = true;
		}

		while ((writeIndex >= m_readIdx) && (m_writeWrapAroundFlag)) {
			//Q is full
			SleepConditionVariableCS(&m_recieverCondVar, &m_bufferLock, INFINITE);
		}

		memcpy(&m_pQueueBuffer[writeIndex], pMessageIn, sizeof(T));
		m_writeIdx = writeIndex;

		LeaveCriticalSection(&m_bufferLock);
		WakeConditionVariable(&m_senderCondVar);
	}

	void pop(T* pMessageOut)
	{
		EnterCriticalSection(&m_bufferLock);
		int readIndex = m_readIdx + 1;

		if (readIndex >= m_queueSize) {
			readIndex = 0;
			m_writeWrapAroundFlag = false;
		}

		while ((readIndex > m_writeIdx) && (m_writeWrapAroundFlag == false)) {
			//Q is empty
			SleepConditionVariableCS(&m_senderCondVar, &m_bufferLock, INFINITE);
		}

		memcpy(pMessageOut, &m_pQueueBuffer[readIndex], sizeof(T));
		m_readIdx = readIndex;

		LeaveCriticalSection(&m_bufferLock);
		WakeConditionVariable(&m_recieverCondVar);
	}

	int tryPuch(const T *pMessageIn)
	{
		EnterCriticalSection(&m_bufferLock);

		int writeIndex = m_writeIdx + 1;
		if (writeIndex >= m_queueSize) {
			writeIndex = 0;
			m_writeWrapAroundFlag = true;
		}

		if ((writeIndex >= m_readIdx) && (m_writeWrapAroundFlag)) {
			LeaveCriticalSection(&m_bufferLock);
			return -1;
		}
		else {
			memcpy(&m_pQueueBuffer[writeIndex], pMessageIn, sizeof(T));
			m_writeIdx = writeIndex;

			LeaveCriticalSection(&m_bufferLock);
			WakeConditionVariable(&m_senderCondVar);
			return 0;
		}
	}

	int tryPop(T* pMessageOut)
	{
		EnterCriticalSection(&m_bufferLock);
		int readIndex = m_readIdx + 1;

		if (readIndex >= m_queueSize) {
			readIndex = 0;
			m_writeWrapAroundFlag = false;
		}

		if ((readIndex > m_writeIdx) && (m_writeWrapAroundFlag == false)) {
			//Q is empty
			return -1;
		}
		else {
			memcpy(pMessageOut, &m_pQueueBuffer[readIndex], sizeof(T));
			m_readIdx = readIndex;

			LeaveCriticalSection(&m_bufferLock);
			WakeConditionVariable(&m_recieverCondVar);
			return 0;
		}
	}


};

const int MESSAGE_QUEUE_DEPTH = 20;

typedef struct actionMessageType {
	searchActionType action;
	int actionIndex;
	bool commitActionFlag;
}actionMessageType;


class displayPlatform :
	public platform
{
private:
	messageQueue <actionMessageType> *m_pQueue;

public:
	displayPlatform();
	~displayPlatform();

	void submiteActionCommand(const actionMessageType *pActionIn);


};

