/*
 * tja1050.cpp
 *
 *  Created on: Jul 15, 2024
 *      Author: thpark
 */
#include "extern.h"
#include "tja1050.h"

tja1050 *pCAN;



tja1050::tja1050()
{
	// TODO Auto-generated constructor stub
	//init();
	canFlag.u8=0;
}

tja1050::~tja1050()
{
	// TODO Auto-generated destructor stub
}


void tja1050::init(void)
{
  //printf("TJA1050::init\r\n");
  can_rx_id=0x700;//RX_CANID; 0x700-0x70F
  can_start();
}

void tja1050::can_start()
{
  CAN_FilterTypeDef sFilterConfig;
  uint32_t filter_id=can_rx_id<<5;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh =filter_id;//FilterIdHigh; // STID[10:0] & EXTID[17:13]
  //sFilterConfig.FilterIdHigh =FilterIdHigh; // STID[10:0] & EXTID[17:13]
  sFilterConfig.FilterIdLow = 0;//FilterIdLow; // EXID[12:5] & 3 Reserved bits
  sFilterConfig.FilterMaskIdHigh =0x7F0<<5;//FilterMaskIdHigh;
  sFilterConfig.FilterMaskIdLow =0x7F0<<5;//0;//FilterMaskIdLow;
  //sFilterConfig.FilterMaskIdHigh =0;//FilterMaskIdHigh;
  //sFilterConfig.FilterMaskIdLow =0;//0;//FilterMaskIdLow;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
      Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
   {
     /* Notification Error */
     Error_Handler();
   }
}


void tja1050::put_canTxd(uint32_t tx_id, uint8_t *data)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint8_t i=0,time_out;
    memcpy(TxData, data ,8);
    //TxHeader.ExtId = tx_id;//sub_id;
    TxHeader.StdId = tx_id;//sub_id;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
    if ((HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox))!=HAL_OK){
    	printf("CAN TX error\r\n");
       // Error_Handler();
     }
    //i=0;
    time_out=5;
    while (HAL_CAN_IsTxMessagePending(&hcan, TxMailbox)){
    	if(time_out>0)time_out--;
    	if(time_out==0)break;
    }
}

void tja1050::CAN_Request_Setup_Data()
{
	uint32_t Pub_ID=0x5B0;
	uint8_t canTX_Data[8][8];
	//uint16_t pData[64];
	//if(!SYSTEM_setup_data_ok)return;
	//printf("==== 달라고해서 보냄 CAN_Request_Setup_Data ====\r\n");
	memcpy(canTX_Data, &pDataClass->setup_data, sizeof(CONFIG_TOTAL));
	for(uint8_t i=0;i<6;i++){
		put_canTxd(Pub_ID+i, canTX_Data[i]);
		printf("Setup_Data(i) (%04lX)\r\n", i, Pub_ID+i);
		for(volatile uint32_t delay = 0; delay < 72000 * 10; delay++);
	}
}

void tja1050::CAN_Request_EVT(uint8_t *req)
{
	pDataClass->vcu_sdu.LeftMotor_velocity = BUILD_UINT16(req[1], req[0]);
	pDataClass->vcu_sdu.canBrakeDelay=req[2] * 5;
	pDataClass->vcu_sdu.canBattery=req[3];
	pDataClass->vcu_sdu.RightMotor_velocity= BUILD_UINT16(req[5], req[4]);
	pDataClass->vcu_sdu.toggle.u8=req[6];
	pDataClass->vcu_sdu.btn.u8   =req[7];
	can_process_flag=1;
	//pDataClass->vcu_sdu.btn.disablePID=0;
	//printf("#### CAN_Request_EVT [%d^%d] canBattery[%d] canBrakeDelay[%d] toggle[0x%04X] btn[0x%04X] disablePID[%d]\r\n", pDataClass->vcu_sdu.RightMotor_velocity, pDataClass->vcu_sdu.LeftMotor_velocity, pDataClass->vcu_sdu.canBattery, pDataClass->vcu_sdu.canBrakeDelay, pDataClass->vcu_sdu.toggle.u8, pDataClass->vcu_sdu.btn.u8 , pDataClass->vcu_sdu.btn.disablePID);
}


void tja1050::canSetConfig(){
	memcpy(&setup_data,&canRcvBuff,sizeof(CONFIG_TOTAL));
	printf("@@@ setup_data.conf2.checkSum[%x]\r\n",setup_data.conf2.checkSum);
	printf("@@@ battery_voltage[%d]  limit_current[%d] limit_motor_temp[%d]\r\n",setup_data.conf1.battery_voltage, setup_data.conf1.limit_current,setup_data.conf1.limit_motor_temp);
	//printf("@@@ tottle_offset[%d] foreward[%d] backward[%d] brake_delay[%d]\r\n",setup_data.conf2.tottle_offset, setup_data.conf2.foreward, setup_data.conf2.backward, setup_data.conf2.brake_delay);
}
void tja1050::CAN_Request_SAVE(uint32_t id, uint8_t *req)
{
	CONFIG_TOTAL config;
	uint8_t buf[64];
	uint8_t sum=0;
	//printf("### CAN_Request_SAVE id[%x]\r\n", id);
	uint8_t memIdx=id-0x708;

	memcpy(&canRcvBuff[memIdx],req,8);
//	switch(id){
//		case 0x708:
//			memcpy(&canRcvBuff[0],req,8);
//			setup_data.conf1.idx			  =BUILD_UINT16(req[0], req[1]);
//			setup_data.conf1.battery_voltage  =BUILD_UINT16(req[2], req[3]);
//			setup_data.conf1.limit_current	  =BUILD_UINT16(req[4], req[5]);
//			setup_data.conf1.limit_motor_temp =BUILD_UINT16(req[6], req[7]);
//			break;
//		case 0x709:
//			memcpy(&canRcvBuff[1],req,8);
//			setup_data.conf1.limit_fet_temp		=BUILD_UINT16(req[0], req[1]);
//			setup_data.conf1.alarm_Battery  	=BUILD_UINT16(req[2], req[3]);
//			setup_data.conf1.cart_type	  		=BUILD_UINT16(req[4], req[5]);
//			setup_data.conf1.motor1_polarity	=BUILD_UINT16(req[6], req[7]);
//			break;
//		case 0x70A:
//			memcpy(&canRcvBuff[2],req,8);
//			setup_data.conf1.motor2_polarity	=BUILD_UINT16(req[0], req[1]);
//			setup_data.conf1.tbd  				=BUILD_UINT16(req[2], req[3]);
//			setup_data.conf2.idx	  		    =BUILD_UINT16(req[4], req[5]);
//			setup_data.conf2.tottle_offset	    =BUILD_UINT16(req[6], req[7]);
//			break;
//		case 0x70B:
//			memcpy(&canRcvBuff[3],req,8);
//			setup_data.conf2.stop_slip  		=BUILD_UINT16(req[0], req[1]);
//			setup_data.conf2.foreward  			=BUILD_UINT16(req[2], req[3]);
//			setup_data.conf2.backward	  		=BUILD_UINT16(req[4], req[5]);
//			setup_data.conf2.accel	    		=BUILD_UINT16(req[6], req[7]);
//			break;
//		case 0x70C:
//			memcpy(&canRcvBuff[4],req,8);
//			setup_data.conf2.decel				=BUILD_UINT16(req[0], req[1]);
//			setup_data.conf2.brake_delay  		=BUILD_UINT16(req[2], req[3]);
//			setup_data.conf2.brake_rate	  		=BUILD_UINT16(req[4], req[5]);
//			setup_data.conf2.checkSum	   		=BUILD_UINT16(req[6], req[7]);
//			break;
//	}
	if(id==0x70C)
	{
		memcpy(buf,&canRcvBuff,sizeof(CONFIG_TOTAL));
		memcpy(&config,&canRcvBuff,sizeof(CONFIG_TOTAL));
		for(int i=0;i<38;i++)sum=sum+buf[i];
		sum= sum &0xff;
		printf("@@@@@@ sum[%x] config.conf2.checkSum[%x]\r\n",sum, config.conf2.checkSum);
		if(sum==config.conf2.checkSum){
			pDataClass->sysFlag.SaveEEPROM=1;
			pDataClass->sysFlag.savedConfigOK=1;
			canFlag.needRxConfigData=1;
		}
		else{
			printf("##########[Error] Need Retry GetData============\r\n");
		}
	}
}

void tja1050::HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t RxData[8]={0,};
	//printf("%s\r\n", __FUNCTION__);
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
 // pDataClass->system_start_flag=1;
  //printf("StdID: %04lx, IDE: %ld, DLC: %ld\r\n", RxHeader.StdId, RxHeader.IDE, RxHeader.DLC);
 // printf("StdID[%04lx] [%d %d %d %d %d %d %d %d]\r\n",RxHeader.StdId, RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
 // pDataClass->sysFlag.canReady=CAN_ALIVE_TIMEOUT;
  can_TimeOut=CAN_ALIVE_TIMEOUT;
  if(RxHeader.StdId>=0x708 && RxHeader.StdId<=0x70C){
	  CAN_Request_SAVE(RxHeader.StdId, RxData);
  }
  else if(RxHeader.StdId==0x700){
	  canFlag.needTxsetupData=1;
	  canFlag.needRxConfigData=0;
	  pDataClass->sysFlag.savedConfigOK=0;
  }
  else if(RxHeader.StdId==0x701){
	  pDataClass->system_start_flag=1;
	  CAN_Request_EVT(RxData);
  }
}
