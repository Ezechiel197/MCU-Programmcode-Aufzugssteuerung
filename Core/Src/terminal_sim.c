#include "project_config.h"
#include "terminal_sim.h"
#include "tiny_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>   //Nötig für eigene printf-Funktionen
#include <string.h>   //

static UART_HandleTypeDef *h_debug;
static UART_HandleTypeDef *h_sim;

// Ringbuffer Debug TX
#define DBG_BUF_SIZE 512
static char DbgBuf[DBG_BUF_SIZE];
static uint16_t DbgHead=0, DbgHeadS=0, DbgTail=0;   // Zu schreiben und lesen im Ringbuffer
static bool DbgTxBusy = false;    // Wenn der CPU(-Mann) beim DebugPort am Senden ist

#define SIM_BUF_SIZE 64
static char SimBuf[SIM_BUF_SIZE];
static uint16_t SimHead=0, SimHeadS=0, SimTail=0;   // Zu schreiben und lesen im Ringbuffer
static bool SimTxBusy = false;    // Wenn die CPU beim DebugPort am Senden ist

// Command Input
#define DBG_INBUF_SIZE 32
static char TermCmdBuf[DBG_INBUF_SIZE];     // Hier wird alle eingehenden Befehle für den Terminal gespeichert
static uint16_t CmdIdx = 0;  	// Aktuelle Schreibposition im Befehls-Puffer
static uint8_t RxChar; 			// Der UART-Interrupt empfängt ein Zeichen in RxChar

// Simulation Input
#define SIM_INBUF_SIZE 24
static char SimCmdBuf[SIM_INBUF_SIZE];   // Hier wird alle eingehenden Befehle für die Simulation gespeichert
static char SimInBuf [SIM_INBUF_SIZE];   // Letzter Befehl zwischengespeichert
static uint16_t bSimIdx = 0; 			 // Aktuelle Schreibposition im Befehls-Puffer
//static uint8_t cSimRx; 		// Der UART-Interrupt empfängt ein Zeichen in cSimRx


/********************************************************************************
 *  Init Funktionen
 *******************************************************************************/
void TERM_Init(UART_HandleTypeDef *huart_debug) {
    h_debug = huart_debug;
    HAL_UART_Receive_IT(h_debug, &RxChar, 1);  // hier wird auf USART1 zugehört, das ankommende Zeichen speichern und ein Interrupt wird ausgelöst
}

void SIM_Init(UART_HandleTypeDef *huart_sim) {

    h_sim = huart_sim;
    memset(SimInBuf, 0, sizeof(SimInBuf));
    HAL_UARTEx_ReceiveToIdle_IT(h_sim, (uint8_t *)&SimCmdBuf[0], sizeof(SimCmdBuf));
}


/********************************************************************************
 *  Callback Check Funktionen
 *
 *  Interne Funktionen um DMA anzustoßen
 *******************************************************************************/
static void CheckDbgTx(void) {
    if (DbgTxBusy || DbgHead == DbgTail) return;

    // Berechne Länge bis Ende
    //Der DMA kann in einem Rutsch nur bis zum physikalischen Ende des Arrays  lesen
    uint16_t len = (DbgHead > DbgTail) ? (DbgHead - DbgTail) : (DBG_BUF_SIZE - DbgTail);
    DbgHeadS = DbgHead; //Position merken

    DbgTxBusy = true;
    if (HAL_UART_Transmit_DMA(h_debug, (uint8_t*)&DbgBuf[DbgTail], len) != HAL_OK)   // Die Daten werden über den DMA auf gesendet
        DbgTxBusy = false;
}

static void CheckSimTx(void) {
    if (SimTxBusy || SimHead == SimTail) return;

    // Berechne Länge bis Ende
    //Der DMA kann in einem Rutsch nur bis zum physikalischen Ende des Arrays  lesen
    uint16_t len = (SimHead > SimTail) ? (SimHead - SimTail) : (SIM_BUF_SIZE - SimTail);
    SimHeadS = SimHead; //Position merken

    SimTxBusy = true;
    if (HAL_UART_Transmit_DMA(h_sim, (uint8_t*)&SimBuf[SimTail], len) != HAL_OK)   // Die Daten werden über den DMA auf gesendet
        SimTxBusy = false;
}


/********************************************************************************
 *  Datenaustausch funktionen
 *******************************************************************************/
void TERM_Log(const char *format, ...) {   // es funktioniert wie printf(() und legt die Daten in DbgHead, die über DMA gesendet wird
    char tmp[128];
    va_list args;
    va_start(args, format);
    vsnprintf(tmp, sizeof(tmp), format, args);
    va_end(args);

    for (int i = 0; i < strlen(tmp); i++) {
        DbgBuf[DbgHead] = tmp[i];
        DbgHead = (DbgHead + 1) % DBG_BUF_SIZE;
    }
    CheckDbgTx();
}

static void SIM_Log(const char *buffer) {

    // Berechne ob buffer noch in den speicher passt
    uint16_t len = (SimHead >= SimTail) ? ((SIM_BUF_SIZE - SimHead) + SimTail) : (SimTail - SimHead);
    if (strlen(buffer) > len) return;

    for (int i = 0; i < strlen(buffer); i++) {
        SimBuf[SimHead] = buffer[i];
        SimHead = (SimHead + 1) % SIM_BUF_SIZE;
    }
    CheckSimTx();
}

void SIM_UpdateState(SIMTELE Tele, uint16_t position) {
    static char sim_msg[32]; //Textpuffer dauerhaft im Speicher bleibt und nicht gelöscht wird, sobald die Funktion zu Ende ist

    switch (Tele)
    {
    	case MOTOR:
    		// Telegram "0001", "03" Byte, "xx" MotorActive, "xx" MotorDirectionUp, "64" 100% Leistung
    	    //snprintf(sim_msg, sizeof(sim_msg), "000103%02x%02x64\n",
    	       //      (uint8_t)SysState.MotorActive, (uint8_t)SysState.MotorDirectionUp);
    		snprintf(sim_msg, sizeof(sim_msg), "000102%04x\n",
    				position);

    		break;
    	case TUER:
    		//HAL_Delay(10);
    		//while (SimTxBusy) {
    		if (position >= 1 && position <= MAX_SUPPORTED_FLOORS) {
				snprintf(sim_msg, sizeof(sim_msg), "%04x01%02x\n", (0x0010+position),!SysState.DoorIsClosed);
    		}

    		else return;
    		//}
    		break;
    	default: return;
    }

   SIM_Log((const char*)sim_msg);
   SetEvent(SIMUL_EVENT);
}

/********************************************************************************
 *  Befehlsparser funktionen
 *
 *  // Befehlsparser Terminal / Simulation
 *******************************************************************************/
void ParseCommand(void) {
    int x, y;
    char *ptr = strchr(TermCmdBuf, '\n');
    char InBuf[DBG_INBUF_SIZE];

    if (ptr == 0) return;
    memcpy(&InBuf, &TermCmdBuf, (ptr - &TermCmdBuf[0]));

    __disable_irq();
    memmove(&TermCmdBuf[0], ptr + 1, DBG_INBUF_SIZE - (ptr + 1 - &TermCmdBuf[0]));
    CmdIdx -= (ptr + 1 - &TermCmdBuf[0]);
    __enable_irq();

    if (sscanf(InBuf, "AUFZ MAX %d", &x) == 1) {  // damit kann man direkt die maximale Anzahl von Etage durch den Terminal festlegen
        if (x >= 1 && x <= MAX_SUPPORTED_FLOORS) {
            SysConfig.MaxFloors = x;
            TERM_Log("%lu: cfg_floors = %d\r\n", HAL_GetTick(), x);
        }
    } else if (sscanf(InBuf, "AUFZ TUER %d %d", &x, &y) == 2) { // das 2 sagt, wie viele Werte erwartet wird
        if (x >= 1 && x <= SysConfig.MaxFloors) {
            SysConfig.FloorPos_cm[x-1] = y;
            TERM_Log("%lu: cfg_door_%d = %d\r\n", HAL_GetTick(), x, y);   //HAL_GetTick    nimmt den Zeitpunkt an dem die Einstellung veränder wurde
        }
    }
}

void ParseSimulation(void) {
    char t_addr[5]; // +\0
    char t_byte[3]; // +\0
    uint16_t t_address;
    uint8_t  t_len;

    if (SimInBuf[0] == '\0') return;

    //Wir sollen immer Zei zeichen als codierstes Hex bekommen.
    //diese müssen wir nun zurück wandeln. sollte strtoul können,...
    t_addr[4] = '\0';
    memcpy(t_addr,&SimInBuf[0],sizeof(t_addr)-1); 		//adresse holen
    t_address = (uint16_t)strtoul(t_addr, NULL, 16);	//adresse konvertieren

    t_byte[2] = '\0';
    memcpy(t_byte,&SimInBuf[4],sizeof(t_byte)-1); 		//länge holen
    t_len = (uint8_t)strtoul(t_byte, NULL, 16);		//länge konvertieren

    //Daten interpretieren:
    if (t_address == 0x0002)
    {
    	if (t_len >= 2)
    	{
    		uint16_t t_pos;
    		memcpy(t_addr,&SimInBuf[6],sizeof(t_addr)-1); 		//;) mal sehn ob das jemand sieht...
    		t_pos = (uint16_t)strtoul(t_addr, NULL, 16);	//t_pos lalala... konvertieren!
    		SysState.CurrentPos_cm = (uint32_t) t_pos * 10;
    	}
    } else if ((t_address & 0xFFF0) == 0x0020)
    {
    	uint8_t t_tuer;
    	memcpy(t_byte,&SimInBuf[6],sizeof(t_byte)-1); 		//Byte holen
    	t_tuer = (uint8_t)strtoul(t_byte, NULL, 16);	// tür state
    	SysState.DoorIsClosed = (bool) t_tuer;			//TODO, Etage hinzufügen


    } else if ((t_address & 0xFFF0) == 0x0030)
    {
    	uint8_t t_anf;
    	memcpy(t_byte,&SimInBuf[6],sizeof(t_byte)-1); 	// Byte holen
    	t_anf = (uint8_t)strtoul(t_byte, NULL, 16);		// Aufzug state
    	if ((t_address & 0x000F) <= MAX_SUPPORTED_FLOORS)
    	{
    		if (t_anf & 0x80)
    			SysState.CallsInternal[(t_address & 0x000F)-1] = (bool)t_anf & 0x7f; // Kabine
    		else
    			SysState.CallsExternal[(t_address & 0x000F)-1] = (bool)t_anf & 0x7f; // Aussen
    	}
    }
    //Kein else fall. alles was nicht passt ist 'scheiße'.

    SimInBuf[0] = '\0';
}


/********************************************************************************
 *  Callback funktionen
 *******************************************************************************/
void TERM_TxCpltCallback(UART_HandleTypeDef *huart) {   // Dieser Teil verarbeitet jedes Zeichen, das am Computer getippt wird und an den STM32 sendet
	if (huart == h_debug && DbgTxBusy) {

    	uint16_t len = (DbgHeadS > DbgTail) ? (DbgHeadS - DbgTail) : (DBG_BUF_SIZE - DbgTail);
    	DbgTail = (DbgTail + len) % DBG_BUF_SIZE;       // Hier wird der Lesezeiger verschoben

        DbgTxBusy = false;
        SetEvent(DEBUG_EVENT);
    }
}

void SIM_TxCpltCallback(UART_HandleTypeDef *huart) {   // Dieser Teil verarbeitet jedes Zeichen, das am Computer getippt wird und an den STM32 sendet
    if (huart == h_sim && SimTxBusy) {

    	uint16_t len = (SimHeadS > SimTail) ? (SimHeadS - SimTail) : (SIM_BUF_SIZE - SimTail);
    	SimTail = (SimTail + len) % SIM_BUF_SIZE;       // Hier wird der Lesezeiger verschoben

        SimTxBusy = false;
        SetEvent(SIMUL_EVENT);
    }
}


void TERM_RxCpltCallback(UART_HandleTypeDef *huart) {   // Dieser Teil verarbeitet jedes Zeichen, das am Computer getippt wird und an den STM32 sendet
    if (huart == h_debug) {
    	if (CmdIdx < (sizeof(TermCmdBuf)-1)) TermCmdBuf[CmdIdx++] = RxChar;

		if (RxChar == '\n') {   // Erst nachdem Enter gedrückt wird, wird den Befehl gesendet
			SetEvent(DEBUG_EVENT);
		}

        HAL_UART_Receive_IT(h_debug, &RxChar, 1);
    }
}

void SIM_RxCpltCallback(UART_HandleTypeDef *huart, uint16_t Size) {   // Dieser Teil verarbeitet jedes Zeichen, das am Computer getippt wird und an den STM32 sendet
    if (huart == h_sim) {
    	bSimIdx += Size;
    	if (SimCmdBuf[bSimIdx-1] == '\n') {
            if (SimInBuf[0] == '\0')
            	memcpy(&SimInBuf, &SimCmdBuf, bSimIdx);
            SetEvent(SIMUL_EVENT);
            bSimIdx = 0;
        } else {
			if (bSimIdx >= sizeof(SimCmdBuf))
				bSimIdx = 0;
        }

    	HAL_UARTEx_ReceiveToIdle_IT(h_sim, (uint8_t*)&SimCmdBuf[bSimIdx], sizeof(SimCmdBuf)-bSimIdx);
    }
}


/********************************************************************************
 *  Time Trigger Function
 *******************************************************************************/
void TERM_Process(void) {
    CheckDbgTx();
}
void SIM_Process(void) {
    CheckSimTx();
}
