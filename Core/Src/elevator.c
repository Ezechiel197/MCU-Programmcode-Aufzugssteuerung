
#include "elevator_FSM.h"
#include "modbus_master.h"
#include "terminal_sim.h"
#include "tiny_os.h"


typedef enum {
	S_IDLE,
	S_MOVING,
    S_ARRIVED_OPEN_DOOR,
	S_DOOR_OPEN_WAIT,
	S_DOOR_CHECK_CLOSED
} State_e;

static State_e State = S_IDLE;
static uint32_t StateTimer = 0;
static uint32_t MoveStartTime = 0;
static uint32_t TravelDuration = 0;
static uint32_t StartPos = 0;

#define MOTOR_RUN() HAL_GPIO_WritePin(MOTOR_EN_PORT,MOTOR_EN_PIN,GPIO_PIN_SET);  // Motor An
#define MOTOR_STOP()  HAL_GPIO_WritePin(MOTOR_EN_PORT, MOTOR_EN_PIN, GPIO_PIN_RESET);
#define DIR_UP() HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_DIR_PIN, GPIO_PIN_SET);  // Nach oben fahren
#define DIR_DOWN() HAL_GPIO_WritePin(MOTOR_DIR_PORT, MOTOR_DIR_PIN, GPIO_PIN_RESET);

 void FSM_Init(void){
	 // Default Werte laden
	 if (SysConfig.MagicNumber != 0xCAFEBABE )
	 {
		 SysConfig.MaxFloors = DEFAULT_FLOOR_COUNT;
		 SysConfig.FloorPos_cm [0]= 0;
		 SysConfig.FloorPos_cm [1]= 400;
		 SysConfig.FloorPos_cm [2]= 800;
	 }
	 SysState.CurrentFloor = 1;
	 SysState.CurrentPos_cm = SysConfig.FloorPos_cm [0];
	 SysState.DoorIsClosed = true;
 }

 static uint8_t GetNextRequest(){
	 // Prio 1: Kabine
	 for(int i =0;i<SysConfig.MaxFloors;i++){
		 if(SysState.CallsInternal[i] && (i+1) != SysState.CurrentFloor)
			 return i+1;
	 }
	 // Prio 2: Etagen
	 for(int i =0;i<SysConfig.MaxFloors;i++){
	 		 if(SysState.CallsExternal[i] && (i+1) != SysState.CurrentFloor)
	 			 return i+1;
 }
	 return 0;
 }

 void FSM_Cycle_100ms(void) {

	 static uint8_t SlowDownCnt = 0;
	 uint32_t now = HAL_GetTick();
	 uint16_t mb_val = 0;

	 if (SlowDownCnt++ > 9)
	 {
		 MB_QueueJob(MB_CABIN_ID,0x03,MB_ADDR_CABIN_REQ,0);
		 if (MB_GetLastResult(10 + SysState.CurrentFloor, 0x01, &mb_val)) {
			if (mb_val != 0) {
				SysState.TargetFloor = mb_val;
			}
		 }
		 for(int i=0; i<SysConfig.MaxFloors;i++){
			 MB_QueueJob(10+(i+1),0x06,MB_ADDR_FLOOR_CALL,SysState.CurrentFloor);
		 }
		 SetEvent(MODBUS_EVENT);
		 SlowDownCnt = 0;
	 }


 // Logik Zustandautomat
 switch(State){
 	 case S_IDLE:
 		 SysState.TargetFloor = GetNextRequest();
 		 if(SysState.TargetFloor !=0){
 			StartPos = SysState.CurrentPos_cm;
 			uint32_t destPos = SysConfig.FloorPos_cm[SysState.TargetFloor-1];
 			uint32_t dist = (StartPos >destPos) ? (StartPos-destPos) : (destPos-StartPos);
 			TravelDuration = (dist * 1000) / MOTOR_SPEED_CM_S;
 			MoveStartTime = now;
 			// Motor bewegwn
 			if(destPos<StartPos) {DIR_DOWN();}
 			else DIR_UP();
 			MOTOR_RUN();
 			TERM_Log("%lu : state = moving target = %d \r\n", now, SysState.TargetFloor);
 			SysState.CurrentFloor = SysState.TargetFloor;
 			//SIM_UpdateState(TUER, SysState.CurrentFloor);
 			SIM_UpdateState(MOTOR, destPos);
 			State = S_MOVING;
 			}
 		 break;
 	case S_MOVING:
 		if (now - MoveStartTime >= TravelDuration) {
 			// Angekommen
 		    MOTOR_STOP();
 		                SysState.CurrentFloor = SysState.TargetFloor;
 		                SysState.CurrentPos_cm = SysConfig.FloorPos_cm[SysState.CurrentFloor-1];
 		               TERM_Log("%lu: state=arrived floor=%d\r\n", now, SysState.CurrentFloor);
 		              State = S_ARRIVED_OPEN_DOOR;
 		}
 		break ;

 	case S_ARRIVED_OPEN_DOOR:
 		MB_QueueJob(10 + SysState.CurrentFloor, 0x06, MB_ADDR_DOOR_CMD, 0x0001);
 		TERM_Log("%lu: State =door_open\r\n", now);
			SysState.DoorIsClosed = false;
			SIM_UpdateState(TUER, SysState.CurrentFloor);
		   StateTimer = now;
		   State = S_DOOR_OPEN_WAIT;
			break;

 	case S_DOOR_OPEN_WAIT:
 		// wird überprüft, ob nach 3s die Tür zu ist
 		if (now - StateTimer > 3000) {
 			TERM_Log("%lu: action=check_door_closed\r\n", now);
 			State = S_DOOR_CHECK_CLOSED;
 			StateTimer = now;
 		}
 		break;
 	case S_DOOR_CHECK_CLOSED:
 		MB_QueueJob(10 + SysState.CurrentFloor, 0x03, MB_ADDR_DOOR_STATUS, 0);
 		bool door_is_physically_closed = false;

 		if (MB_GetLastResult(10 + SysState.CurrentFloor, 0x01, &mb_val)) {
 		 		    // Wenn Wert == 1, ist Tür zu
 		 		    if (mb_val == 1) {
 		 		        door_is_physically_closed = true;
 		 		    }
 		 		}
 		// 2. Timeout
 		if (now - StateTimer > 5000) {
			  TERM_Log("%lu: error=door_timeout action=force_next\r\n", now);
			  // Ruf löschen
			  SysState.CallsInternal[SysState.CurrentFloor-1] = false;
			  SysState.CallsExternal[SysState.CurrentFloor-1] = false;

			  SysState.DoorIsClosed = true;
				 SIM_UpdateState(TUER, SysState.CurrentFloor);
			     State = S_IDLE;
		  }
 		else if (door_is_physically_closed) {
 			TERM_Log("%lu: event=door_closed_confirmed\r\n", now);

 			// Ruf löschen
			SysState.CallsInternal[SysState.CurrentFloor-1] = false;
			SysState.CallsExternal[SysState.CurrentFloor-1] = false;

			SysState.DoorIsClosed = true;
			SIM_UpdateState(TUER, SysState.CurrentFloor);
			State = S_IDLE;
 		}
        break;

 	 }
 }


