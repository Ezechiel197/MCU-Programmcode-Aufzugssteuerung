

#ifndef TERM_SIM_H
#define TERM_SIM_H

typedef enum
{
	MOTOR,
	TUER
} SIMTELE;

#include "project_config.h"

void TERM_Init(UART_HandleTypeDef *huart_debug);
void SIM_Init(UART_HandleTypeDef *huart_sim);

void TERM_Process(void);
void SIM_Process(void);

void ParseCommand(void);
void ParseSimulation(void);

void TERM_Log(const char *format, ...);
void SIM_UpdateState(SIMTELE Tele, uint16_t etage);

// Callbacks
void TERM_TxCpltCallback(UART_HandleTypeDef *huart);
void SIM_TxCpltCallback(UART_HandleTypeDef *huart);

void TERM_RxCpltCallback(UART_HandleTypeDef *huart);
void SIM_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif
