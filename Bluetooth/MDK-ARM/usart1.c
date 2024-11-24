#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"
#include "usart1.h"
#include "string.h"

// Wrapper rond HAL_UART_Transmit() om zo de reeds geschreven code die StringToUsart1() 
// nodig heeft, te kunnen gebruiken...
void StringToUsart1(char* string)
{
	extern UART_HandleTypeDef huart1;
	
	HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
}
