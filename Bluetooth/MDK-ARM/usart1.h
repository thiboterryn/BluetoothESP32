#include "stm32l4xx.h"

#if !defined(USART1_DEFINED)
	#define USART1_DEFINED
	
	enum UsartState {idle, busyReceiving, newStringArrived, receptionTimeOut, overflowOccured};
//	#define MAX_NUMBER_OF_CHARS 200
	
	void StringToUsart1(char* string);
#endif
