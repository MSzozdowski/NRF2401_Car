/*
 * watchdog.h
 *
 *  Created on: Mar 18, 2023
 *      Author: Michal
 */

#ifndef INC_WATCHDOG_H_
#define INC_WATCHDOG_H_


void WDG_Refresh(IWDG_HandleTypeDef *hiwdg);
void WDG_ResetInfo(void);

#endif /* INC_WATCHDOG_H_ */
