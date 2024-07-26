/*
 * can.c
 *
 *  Created on: Jun 3, 2024
 *      Author: work
 */

#include "main.h"

extern CAN_HandleTypeDef hcan1;

void filter_config(void){
	CAN_FilterTypeDef CanFilter;
	CanFilter.FilterActivation = ENABLE;
	CanFilter.FilterBank = 0;
	CanFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	CanFilter.FilterIdHigh = 0x0000;
	CanFilter.FilterIdLow = 0x0000;
	CanFilter.FilterMaskIdHigh = 0x0000;
	CanFilter.FilterMaskIdLow = 0x0000;
	CanFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	CanFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	CanFilter.SlaveStartFilterBank =14;

	if(HAL_CAN_ConfigFilter(&hcan1, &CanFilter) != HAL_OK){
		Error_Handler();
	}
}

