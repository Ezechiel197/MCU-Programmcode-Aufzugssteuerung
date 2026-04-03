/*
 * project_config.h
 * Zentrale Definitionen für das Aufzug-Projekt
 */

#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h" // Importiert HAL Definitionen

// --- Hardware Pin Definitionen ---
#define MOTOR_EN_PORT       GPIOA
#define MOTOR_EN_PIN        GPIO_PIN_4  // für Motor an und aus
#define MOTOR_DIR_PORT      GPIOD
#define MOTOR_DIR_PIN       GPIO_PIN_14	// für Fahrrichtung Hoch und runter

// RS485 Driver Enable Pin
#define RS485_DE_PORT       GPIOD
#define RS485_DE_PIN        GPIO_PIN_4

// --- System Parameter ---
#define MAX_SUPPORTED_FLOORS 9
#define DEFAULT_FLOOR_COUNT  3
#define MOTOR_SPEED_CM_S     400  // 400 cm/s
#define CYCLE_TIME_MS        100  // 100ms Loop

// --- Modbus Parameter ---
#define MB_BAUD             19200  // Baudrate
#define MB_MASTER_ID        1
#define MB_CABIN_ID         10  // ID_der Slave_Kabine
#define MB_TIMEOUT_MS       100  // Die maximale Wartezeit, in der der Master auf eine Rückmeldung der Slave wartet, nach diese Zeit gilt der Slave als ausgefallen

// Modbus Adressen Register in den Slaves
#define MB_ADDR_CABIN_REQ   0x0001 // Zielwahl in der Kabine , Function Code FC03
#define MB_ADDR_DOOR_STATUS 0x0002 // hier liest der Master ob die Etagetür Tür zu ist
#define MB_ADDR_FLOOR_CALL  0x0003 // Rufknopf im Etage lesen, Function Code FC01
#define MB_ADDR_DOOR_CMD    0x0004 // hier wird setzt diese Register auf 1 um zu sagen, die Tür geöffnet werden kann


// --- Datenstrukturen ---

// Persistente Konfiguration (Flash)
typedef struct {
    uint8_t MaxFloors;
    uint32_t FloorPos_cm[MAX_SUPPORTED_FLOORS];    // hier wird die Etageposition als Persistenz gespeichert
    uint32_t MagicNumber; // 0xCAFEBABE zur Validierung, wenn die Wert beim Start noch steht, dann weiß der MCU, dass die persistenz Daten noch gültig sind
} SystemConfig_t;

// Laufzeit-Zustand (RAM)
typedef struct {
    uint8_t CurrentFloor;       // 1-9
    uint8_t TargetFloor;        // Zieletage
    uint32_t CurrentPos_cm;     //  Aktuelle Position in Zentiemeter
	bool DoorIsClosed;          // Status der Tür
    bool DoorIsSave;          	// Tür ist nun sicher und kann geöffnet werden

    // Anforderungs-Management
    bool CallsExternal[MAX_SUPPORTED_FLOORS];  // speichert in jede Etage den Zustand der Ruftaste in jede Etage, der im Register_adresse MB_ADDR_FLOOR_CALL geschrieben ist
    bool CallsInternal[MAX_SUPPORTED_FLOORS];	//speichert in jede Etage den Zustand der Ruftaste in jede Etage, der im Register_adresse MB_ADDR_FLOOR_CALL geschrieben ist

    // Status Flags
    bool MotorActive;  // wird auf TRUE gesetzt, wenn der Motor an geht und auf False wenn er aus geht
    bool MotorDirectionUp; // wird auf TRUE gesetzt, wenn der Motor  hochgeht und auf False wenn er aus runtergeht
} SystemState_t;

// Globale Instanzen
#ifdef ISMAIN
	SystemConfig_t SysConfig;
	SystemState_t SysState;
#else
    extern SystemConfig_t SysConfig;   // hier ist sysConfig eine externe Variable von typ der struct SystemConfig_t
    extern SystemState_t SysState;	   //
#endif
#endif // PROJECT_CONFIG_H
