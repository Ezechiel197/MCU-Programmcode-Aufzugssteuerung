

#include "tiny_os.h"
#include "modbus_master.h"
#include <string.h>



#define MAX_JOBS 20
#define MB_RX_BUF_SIZE 256

typedef struct {
    uint8_t SlaveID;
    uint8_t FuncCode;
    uint16_t Address;
    uint16_t Value;
    bool IsWrite;
} ModbusJob_t;

// Queue Variablen
static ModbusJob_t JobQueue[MAX_JOBS];
static uint8_t JobHead = 0;
static uint8_t JobTail = 0;

// Hardware & Status
static UART_HandleTypeDef *mb_huart;
static bool Busy = false;
static uint32_t LastTxTime = 0;

// Buffer
static uint8_t TxBuf[256];
static uint8_t RxBuf[MB_RX_BUF_SIZE];

// Ergebnis-Speicher
static uint16_t LastReadValue = 0;
static uint8_t LastReadSlave = 0;
static uint8_t LastReadFC = 0;
static bool NewResultAvailable = false;

// Aktueller Job
static ModbusJob_t CurrentJob;

//  CRC16 Berechnung
static uint16_t MB_CalcCRC(uint8_t *buffer, uint16_t length) {    // Prüfsumme
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// --- Initialisierung ---
void MB_Init(UART_HandleTypeDef *huart) {
	JobHead = 0;
	JobTail = 0;
    mb_huart = huart;
    // RS485 auf Empfang schalten
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
    Busy = false;
}

// --- Job Queueing ---
void MB_QueueJob(uint8_t slave, uint8_t fc, uint16_t addr, uint16_t val) {
    // Nächster Index
    uint8_t nextHead = (JobHead + 1) % MAX_JOBS;

    // Wenn Queue voll, Job verwerfen
    if (nextHead == JobTail) return;

    ModbusJob_t *j = &JobQueue[JobHead];
    j->SlaveID = slave;
    j->FuncCode = fc;
    j->Address = addr;
    j->Value = val;

    j->IsWrite = (fc == 0x05 || fc == 0x06 || fc == 0x10);  // fc05: schreibt single Coils, fc06 schreibt single Register, fc10 write multiple Register

    JobHead = nextHead;
}

// Vorbereitung der Modbus-Frame und Timeout überwachung
void MB_Process(void) {
    // 1. Timeout Überwachung
    if (Busy) {
        if ((HAL_GetTick() - LastTxTime) > MB_TIMEOUT_MS) {   // Wenn ja,Abbruch

            HAL_UART_AbortReceive_IT(mb_huart);  // der stoppt den aktiven UART-Interrupt-gesteuerten Empfang
            HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
            Busy = false;
        }
        return;
    }

    // 2. Queue prüfen
    if (JobHead == JobTail) return; // Queue leer

    // 3. Job holen
    ModbusJob_t *j = &JobQueue[JobTail];
    CurrentJob = *j;

    // Queue Index weiterschalten
    JobTail = (JobTail + 1) % MAX_JOBS;

    // 4. Frame bauen
    uint8_t len = 0;
    TxBuf[len++] = j->SlaveID;
    TxBuf[len++] = j->FuncCode;
    TxBuf[len++] = (j->Address >> 8) & 0xFF;
    TxBuf[len++] = j->Address & 0xFF;

    if (j->IsWrite) {

        TxBuf[len++] = (j->Value >> 8) & 0xFF;   // hier wird geschrieben
        TxBuf[len++] = j->Value & 0xFF;
    } else {

        TxBuf[len++] = 0x00;
        TxBuf[len++] = 0x01;    // hier wird gelesen, aber nur in einem Register
    }

    // CRC anhängen
    uint16_t crc = MB_CalcCRC(TxBuf, len);
    TxBuf[len++] = crc & 0xFF;
    TxBuf[len++] = (crc >> 8) & 0xFF;

    // 5. Senden
    // RS485 DE High
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);

    Busy = true;
    LastTxTime = HAL_GetTick();

    // Senden DMA
    // Die CPU wird sofort entlastet und kann andere Aufgaben erledigen
    if(HAL_UART_Transmit_DMA(mb_huart, TxBuf, len) != HAL_OK) {
        HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
        Busy = false;
    }
}


// Wird aufgerufen, wenn Senden fertig ist
void MB_TxCpltCallback(void) {    // Hier wird den DE-RS485 auf LOW geschaltet, nachdem die Übertragung abgeschlossen ist und der Interrupt wird ausgelöst
									// um  die Antwort zu bearbeiten
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
    HAL_UARTEx_ReceiveToIdle_IT(mb_huart, (uint8_t *)&RxBuf[0], sizeof(RxBuf));   // hier wird geachtet, ob der Buffer Voll ist ( alle Daten angekommen) oder die Idle Line (die 3.5 Zeichenlängen-Pause am Ende )
    																           // erkannt ist, was signalisiert, dass alle Daten empfangt wurden
}

void MB_RxEventCallback(uint16_t Size) {
    if (!Busy) return;

    // Bei Exception: ID + (FC|0x80) + ExCode + CRC = 5 Bytes
    if (Size < 5) {
        // Frame zu kurz
        Busy = false;
        return;
    }

    // 2. Slave ID prüfen
    if (RxBuf[0] != CurrentJob.SlaveID) {
        Busy = false;
        return;
    }

    uint16_t received_crc = RxBuf[Size-2] | (RxBuf[Size-1] << 8);
    uint16_t calculated_crc = MB_CalcCRC(RxBuf, Size - 2);

    if (received_crc == calculated_crc) {

        // Prüfen ob es eine Exception ist
        if (RxBuf[1] & 0x80) {
        }
        else if (RxBuf[1] == CurrentJob.FuncCode) {
            // Gültige Antwort
            uint16_t val = 0;

            if (CurrentJob.FuncCode == 0x03) { // Read Holding Register

                if (Size >= 7) {    // Format: ID, FC, Bytes(1), Hi, Lo, CRC
                    val = (RxBuf[3] << 8) | RxBuf[4];

                    // Ergebnis bereitstellen
                    LastReadSlave = CurrentJob.SlaveID;
                    LastReadFC = CurrentJob.FuncCode;
                    LastReadValue = val;
                    NewResultAvailable = true;
                }
            }
            else if (CurrentJob.FuncCode == 0x01) { // Read Coil

                 if (Size >= 6) {                 // Format: ID, FC, Bytes(1), StatusByte, CRC
                     val = RxBuf[3];

                     LastReadSlave = CurrentJob.SlaveID;
                     LastReadFC = CurrentJob.FuncCode;
                     LastReadValue = val;
                     NewResultAvailable = true;
                     SetEvent(MODBUS_EVENT);
                 }
            }

        }
    }

    // Transaktion abgeschlossen
    Busy = false;
}

// --- App Interface ---
bool MB_GetLastResult(uint8_t slave_id, uint8_t fc, uint16_t *value_out) {
    if (NewResultAvailable && LastReadSlave == slave_id && LastReadFC == fc) {
        *value_out = LastReadValue;
        NewResultAvailable = false; // Flag löschen
        return true;
    }
    return false;
}




