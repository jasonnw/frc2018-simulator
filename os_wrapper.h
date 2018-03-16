#pragma once
#include <stdio.h>
#include <stdint.h>

#ifdef _MSC_VER
#include <windows.h>

#define DeleteConditionVariable(x)   /*windows has no delete conditional variable*/

//pthread data types
#define pthread_mutex_t CRITICAL_SECTION  
#define pthread_cond_t CONDITION_VARIABLE 
#define pthread_mutexattr_t int
#define pthread_condattr_t int

//pthread functions
#define pthread_mutexattr_init(a) /**/
#define pthread_mutexattr_destroy(a) /**/
#define pthread_mutex_init(a, b)  InitializeCriticalSection(a)
#define pthread_mutex_lock(a) EnterCriticalSection(a)
#define pthread_mutex_unlock(a) LeaveCriticalSection (a)
#define pthread_mutex_destroy(a) DeleteCriticalSection(a)

#define pthread_condattr_init(a) /**/
#define pthread_condattr_destroy(a) /**/

#define pthread_cond_init(a, b) InitializeConditionVariable (a)
#define pthread_cond_wait(a, b) SleepConditionVariableCS(a, b, INFINITE) 
#define pthread_cond_signal(a) WakeConditionVariable(a)
#define pthread_cond_destroy(a) /*windows has no delete conditional variable*/

#else

#include <pthread.h>

#define WINAPI /**/
typedef  int32_t DWORD;
typedef void * LPVOID;
typedef int HANDLE;

#define CreateThread(thread, attr, entryPoint, arguments, flag, threadId)  pthread_create(thread, attr, entryPoint, arguments)
#endif
