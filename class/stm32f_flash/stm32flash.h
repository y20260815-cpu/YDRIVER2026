/*
 * stm32flash.h
 *
 *  Created on: Oct 8, 2024
 *      Author: thpark
 */

#ifndef STM32F_FLASH_STM32FLASH_H_
#define STM32F_FLASH_STM32FLASH_H_

#define START_ADDR ((uint32_t)0x0801F800)
#define EndAddr ((uint32_t)0x0801F8FF)

//#define FLASH_STORAGE_M 0x08005000
//#define FLASH_STORAGE_Q 0x08005500
#define page_size 0x800

typedef struct
{
    uint16_t stx;
    SYSTEM_CONF config_data;
    uint8_t checksum;
    uint8_t etx;
}FLASH_MEM_CONFIG;


class stm32flash
{
public:
	stm32flash();
	virtual ~stm32flash();

	FLASH_MEM_CONFIG read_flash_config();
	void save_to_flash_config(SYSTEM_CONF config_data);
	void Get_BackUP();

};

#endif /* STM32F_FLASH_STM32FLASH_H_ */
