/*
 * lights.h
 *
 *  Created on: Jan 29, 2023
 *      Author: Michal Szozdowski
 */

#ifndef INC_LIGHTS_H_
#define INC_LIGHTS_H_

void Lights_SetState(GPIO_TypeDef* GpioPort, uint16_t GpioPin, uint8_t lights_state);

#endif /* INC_LIGHTS_H_ */
