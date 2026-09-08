/*
 * stm32flash.cpp
 *
 *  Created on: Oct 8, 2024
 *      Author: thpark
 */
#include "extern.h"
#include "stm32flash.h"
#include "stm32f1xx_hal_flash.h"

stm32flash *pFlash_mem;

stm32flash::stm32flash()
{
	// TODO Auto-generated constructor stub
	//HAL_FLASH_Unlock();

}

stm32flash::~stm32flash()
{
	// TODO Auto-generated destructor stub
}

void stm32flash::save_to_flash_config(SYSTEM_CONF config_data)
{
#if 0
	int i;
	uint8_t sum=0;
	uint32_t mem_size;
	FLASH_MEM_CONFIG mem_config;
	volatile uint16_t mem_sz=sizeof(FLASH_MEM_CONFIG);
	volatile uint16_t config_data_size=sizeof(SYSTEM_CONF);

	mem_size=(mem_sz%4) ? ((mem_sz/4)+1)*4: mem_sz;
	//uint8_t data[mem_size+1]={0,};
	uint8_t data[128]={0,};
	uint8_t config_data_buf[config_data_size+1]={0,};

	memcpy(&config_data_buf, &config_data, config_data_size);

	for(i=0;i<config_data_size;i++){
		sum=sum+config_data_buf[i];
	}
	sum=sum&0xFF;

	mem_config.stx=0xAA55;
	memcpy(&mem_config.config_data, &config_data, config_data_size);
	mem_config.etx=0x55;
	mem_config.checksum=sum;

	//for(i=0;i<config_data_size;i++) printf("[%d] config_data_buf[%d][%02X]\r\n",config_data_size, i,config_data_buf[i]);

	memcpy(&data[0], &mem_config, mem_size);

	//for(i=0;i<mem_sz;i++) printf("mem_sz[%d][%d] data[%d][%02X]\r\n",mem_sz,mem_size, i,data[i]);

	//printf("mem_size[%d] data [%x][%x][%x][%x] [%x][%x][%x][%x]\r\n",mem_size, data[0] ,data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
	//printf("(1)stx[%x] etx[%x] checksum[%x] motor_num[%x] size(%d)(%d)\r\n",mem_config.stx, mem_config.etx, mem_config.checksum, mem_config.config_data.motor_electrical.motor_num, mem_size,config_data_size );

	//volatile uint32_t data_to_FLASH[(strlen((char*)data)/4)	+ (int)((strlen((char*)data) % 4) != 0)];
	volatile uint32_t data_to_FLASH[mem_size];
	memset((uint8_t*)data_to_FLASH, 0, mem_size);
	memcpy((uint8_t*)data_to_FLASH, (char*)data, mem_size);
	//strcpy((char*)data_to_FLASH, (char*)data);

	//volatile uint32_t data_length = (strlen((char*)data_to_FLASH) / 4)+ (int)((strlen((char*)data_to_FLASH) % 4) != 0);
	volatile uint32_t data_length = mem_size;//(strlen((char*)data_to_FLASH) / 4)+ (int)((strlen((char*)data_to_FLASH) % 4) != 0);
	volatile uint8_t pages;

	if(mem_size%page_size){
		pages=(mem_size/page_size)+1;
	}
	else pages=mem_size/page_size;

//	 pages=(mem_size%4) ? ((mem_size%4)+1)*4: mem_size;
//	= (mem_size/page_size)	+ (int)(((mem_size%page_size) != 0);
	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Allow Access to option bytes sector */
	  HAL_FLASH_OB_Unlock();

	  /* Fill EraseInit structure*/
	  FLASH_EraseInitTypeDef EraseInitStruct;
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.PageAddress = START_ADDR;
	  EraseInitStruct.NbPages = pages;
	  uint32_t PageError;

	  volatile uint32_t write_cnt=0, index=0;
	  volatile HAL_StatusTypeDef status;

	  status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	  while(index < data_length)
	  {
		  if (status == HAL_OK)
		  {
			  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, START_ADDR+write_cnt, data_to_FLASH[index]);
			  if(status == HAL_OK)
			  {
				  write_cnt += 4;
				  index++;
			  }
		  }
	  }
	  HAL_FLASH_OB_Lock();
	  HAL_FLASH_Lock();
#endif
	(void)config_data;
	printf("SYSTEM_CONF flash save ignored\r\n");
}

FLASH_MEM_CONFIG stm32flash::read_flash_config()
{
	FLASH_MEM_CONFIG mem_config;
	volatile uint32_t read_data;
	volatile uint16_t size=sizeof(FLASH_MEM_CONFIG);

	uint8_t buf[size*2]={0,};
	int i,j;
	for(i=0;i<(size/4)+1;i++){
		j=i*4;
		read_data = *(uint32_t*)(START_ADDR + j);
		//printf("read_data[%x][%08X]\r\n",j,read_data);
		buf[j + 0] = (uint8_t)read_data;
		buf[j + 1] = (uint8_t)(read_data >> 8);
		buf[j + 2] = (uint8_t)(read_data >> 16);
		buf[j + 3] = (uint8_t)(read_data >> 24);
	}
	memcpy(&mem_config,buf,size);
	return mem_config;
}

void stm32flash::Get_BackUP(){
#if 0
	FLASH_MEM_CONFIG mconfig;
	//volatile uint16_t sz=sizeof(FLASH_MEM_CONFIG);
	volatile uint16_t config_data_size=sizeof(SYSTEM_CONF);

	uint8_t config_data_buf[config_data_size+1]={0,};
	uint8_t i,sum;

	mconfig=read_flash_config();
	memcpy(&config_data_buf, &mconfig.config_data, config_data_size);
	sum=0;
	for(i=0;i<config_data_size;i++){
		sum=sum+config_data_buf[i];
	}
	sum=sum&0xFF;
#if 1 //0:For Debuging 메모리 초기값으로... 1: release
	if(mconfig.stx==0xAA55 && sum==mconfig.checksum){
		printf("\r\n@@@MEMORY [OK]~\r\n");
		//memcpy(&pDataClass->cart_config, &mconfig.config_data, sizeof(CART_SETUP));
		memcpy(&sysConf, &mconfig.config_data, sizeof(SYSTEM_CONF));
	}
	else
#endif
	{
		printf("\r\n@@@MEMORY [Fail]~\r\n");
		memcpy(&sysConf, &sysConf_init, sizeof(SYSTEM_CONF));
		save_to_flash_config(sysConf);
	}
	//uint16_t pData[32];
	//memcpy(pData, &setup_init, sizeof(CONFIG_TOTAL));
	//for(i=0;i<20;i++)printf("[%d][%d]\r\n", i,pData[i] );

	// Electromagnetic brake delay is intentionally fixed in dataclass.h.
	pDataClass->brake_delay=ELECTROMAGNETIC_BRAKE_DELAY_MS;
	//pDataClass->gamsok_idx=sysConf.brake_rate;
	//pDataClass->motor1_polarity=sysConf.motor1_polarity;
	//pDataClass->motor2_polarity=sysConf.motor2_polarity;
	//pDataClass->set_foreward=sysConf.foreward/100.0f;
	//pDataClass->set_backward=sysConf.backward/100.0f;

	SYSTEM_setup_data_ok=1;
	//printf("@@@ motor_polrarity(%d)(%d)\r\n",sysConf.motor1_polarity, sysConf.motor2_polarity );
	//printf("@@@ motor_FR[%d][%d] offset[%d]\r\n",sysConf.foreward, sysConf.backward, sysConf.tottle_offset);
	//printf("@@@ motor_Accel[%d] Decel[%d] \r\n",sysConf.accel, sysConf.decel);
	//printf("@@@ brake_delay[%d] accel[%.2f] decel[%.2f]\r\n",pDataClass->brake_delay, ACCEL_RATE, DECEL_RATE);

#endif
	SYSTEM_setup_data_ok=0;
	printf("SYSTEM_CONF flash load ignored\r\n");
}
