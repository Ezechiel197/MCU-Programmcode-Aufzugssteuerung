

#ifndef INC_MODBUS_MASTER_H_
#define INC_MODBUS_MASTER_H_

#include "project_config.h"

// Initialisierung
void MB_Init(UART_HandleTypeDef *huart);  // UART-Handle wird übergeben, damit der Modbus-Treiber den gewüschten Schnittstelle benutzt

// Job in die Warteschlange stellen
void MB_QueueJob(uint8_t slave, uint8_t fc, uint16_t addr, uint16_t val);// stellt Aufgabe und Abfragen auf den Zwischenspeicher der MCU, um den nächste Befehl zu bearbeiten

// Muss in der Main-Loop  gerufen werden
void MB_Process(void); // Er sendet der nächste Auftrag, überwacht den Timeout und schaltet ggf. den RS485-Treiber (DE-Pin) auf High um


void MB_TxCpltCallback(void);  // der schaltet RS485-Treiber (DE-Pin) auf Low, nachdem der letzte Byte des Telegramm gesendet wurde
void MB_RxEventCallback(uint16_t Size); // wird nach dem Empfang einer Antwort aufgerufen, der macht die CRC-Prüfsumme des empfangenen Telegramms, aktualisiert den Zustand der Modbus-treiber (interupservice routine)


bool MB_GetLastResult(uint8_t slave_id, uint8_t fc, uint16_t *value_out);  // hier wird True zurückgegebn, wenn die empfangene Daten von Modbus-Treiber bereitgestellt wurde


#endif
