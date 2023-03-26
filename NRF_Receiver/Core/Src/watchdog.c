/*
 * watchdog.c
 *
 *  Created on: Mar 18, 2023
 *      Author: Michal
 */

#include "main.h"
#include "watchdog.h"
#include "stdio.h"

void WDG_Refresh(IWDG_HandleTypeDef *hiwdg)
{
	if(HAL_IWDG_Refresh(hiwdg) != HAL_OK)
		  {
			  Error_Handler();
		  }
}

void WDG_ResetInfo(void)
{
	  if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
	  {
		  printf("!!! IWDG RESET !!!!\n\r");
		  __HAL_RCC_CLEAR_RESET_FLAGS();
	  }
	  else
	  {
		  printf("!!! Normal reset !!!!\n\r");
	  }
}
