#pragma once

#ifndef EXTERN_H_
#define EXTERN_H_
#include "typedef.h"
#include "system_setup.h"

#include "stdio.h"
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "adc.h"
#include "tim.h"
#include "can.h"
#include "usart.h"
#include "../../class/tja1050/tja1050.h"
#include "../../class/data_class/dataclass.h"
#include "../../class/pwm16/pwm16.h"
#include "../../class/fnd595/fnd595.h"
#include "../../class/stm32f_flash/stm32flash.h"
//#include "../../class/zigbee/zigbee.h"
//#include "../../class/wifi_data/wifiParse.h"
#include "../../class/eps/eps.h"

//#include "../../class/data_class/data_class.h"
//#include "../../class/lcd_main_page/sub_menu_string.h"
//#include "../../class/spi_flash/spi_flash.h"
//#include "../../class/rs485/rs485.h"
//#include "../../class/modem/modem.h"
//#include "../../class/ctrl_jaesang_heater/jaesang_heater.h"
//#include "../../class/ctrl_evafan/ctrl_evafan.h"
////#include "../../class/img_down_load/imgdownload.h"
//#include "../../class/ctrl_ozone/OZONECTRL.h"
//#include "../../class/ctrl_comp/ctrl_comp.h"
//#include "../../class/i2c_bm8563/I2C_BM8563.h"
//#include "../../class/lcd_main_page/lcd_main_page.h"
//#include "../../class/stm32_io/stm32_io.h"
//#include "../../class/bigint/BigInt.h"
//#include "../../class/tm1637/TM1637.h"
#define ADC_CHANNEL_COUNT 10
extern uint16_t adc_buf[ADC_CHANNEL_COUNT];
/////////////////////////////////////
//#define xDBG_BOARD
//#define xDBG_EVENT
//#define xDBG_ONE_SEC
//#define xDBG_BUTTON
//#define xDBG_COMP
//#define xDBG_TEMP
//#define xDBG_TEMP_ALARM
//#define xDBG_EVAFAN
//#define xDBG_JAESANG
//#define xDBG_OZONE
//#define xDBG_I2C_RTC_API
//#define xDBG_MODEM
//#define xDBG_DATACLASS
//#define xDBG_TEMP_ALARM
//#define xDBG_UPDATE
//#define xDBG_STM32_IO
//#define xDBG_TEST_MODE
//#define xDBG_ADC
//
//#define JAESANG_TEST_CAP
////////////////////////////////////////
//#define RELAY_ERROR_EVENT
////////////////////////////////////////
//#define RELAY_BD_PHASE 2	//2:단상 3:3상
//#define POWER_ON_DELAY  5 //ppp 20140103
//#define MINUTE  60
//#define SECOND  1
//#define SYSTEM_INIT_TIMEOUT 10*SECOND
//#define MAX_PLASMA 1000
//
//#define _COMP_ON_LATENCY     pLCD_MAIN->UI_Data.data_arry[COMP_PAGE].idata[1] //"콤프지연"
//#define _EVAFAN_OFF_LATENCY  pLCD_MAIN->UI_Data.data_arry[COMP_PAGE].idata[2] //"EVA팬지연"
//#define _COMP_YENDONG_EVAFAN pLCD_MAIN->UI_Data.data_arry[COMP_PAGE].idata[3]//"COMP작동중 EVA팬"
//#define _COMPON_AT_JAESANG 	 pLCD_MAIN->UI_Data.data_arry[COMP_PAGE].idata[5]//
//#define _COMP_HYSTERESIS     pLCD_MAIN->UI_Data.data_arry[COMP_PAGE].idata[6]
//#define _TEMP_CALIBRATION    pLCD_MAIN->UI_Data.data_arry[COMP_PAGE].idata[7]
//#define _SUPER_COOL_TEMP     pLCD_MAIN->UI_Data.data_arry[COMP_PAGE].idata[8]*10
//
//#define _JAESANG_CONTROL_MODE 		pLCD_MAIN->UI_Data.data_arry[JAESANG_PAGE].idata[1]
//#define _JAESANG_TOTAL_PERIOD  		pLCD_MAIN->UI_Data.data_arry[JAESANG_PAGE].idata[2]*MINUTE
//#define _JAESANG_ON_DURATION   		pLCD_MAIN->UI_Data.data_arry[JAESANG_PAGE].idata[3]*MINUTE
//#define _JAESANG_SENSOR_PERIOD 		pLCD_MAIN->UI_Data.data_arry[JAESANG_PAGE].idata[4]*MINUTE
//#define _JAESANG_FORCE_TIME 		pLCD_MAIN->UI_Data.data_arry[JAESANG_PAGE].idata[8]*MINUTE
////#define _JAESANG_COMPON_AT_JAESANG pLCD_MAIN->UI_Data.data_arry[JAESANG_PAGE].idata[8]
//
////#define _OZONE_CONTROL_MODE   pLCD_MAIN->UI_Data.data_arry[OZONE_PAGE].idata[1]
//#define _OZONE_GEN_ON_DELAY   PLASMA_BEFORE_CIRCULA_FAN_ON //pLCD_MAIN->UI_Data.data_arry[OZONE_PAGE].idata[2]
//#define _OZONE_FAN_OFF_DELAY  PLASMA_AFTER_CIRCULA_FAN_OFF //pLCD_MAIN->UI_Data.data_arry[OZONE_PAGE].idata[3]
////#define _OZONE_DIFF           pLCD_MAIN->UI_Data.data_arry[OZONE_PAGE].idata[4]
//#define _OZONE_STANDARD_DENSITY  pLCD_MAIN->UI_Data.data_arry[OZONE_PAGE].idata[4]//ppm
//#define _OZONE_TOTAL_PERIOD   30*MINUTE  //pLCD_MAIN->UI_Data.data_arry[OZONE_PAGE].idata[5]*MINUTE //주기
////#define _OZONE_ON_DURATION    pLCD_MAIN->UI_Data.data_arry[OZONE_PAGE].idata[6] //발생시간
//
//#define _SELJUNG_ABNORMAL_KEEP_TIME pLCD_MAIN->UI_Data.data_arry[SELJUNG_PAGE].idata[1]*MINUTE
//#define _SELJUNG_ABNORMAL_HIGH_TEMP pLCD_MAIN->UI_Data.target.temp + (pLCD_MAIN->UI_Data.data_arry[SELJUNG_PAGE].idata[2] * 10)
//#define _SELJUNG_ABNORMAL_LOW_TEMP  pLCD_MAIN->UI_Data.target.temp + (pLCD_MAIN->UI_Data.data_arry[SELJUNG_PAGE].idata[3] * 10)
//#define _SELJUNG_KYUNGBO_DELAY   	 pLCD_MAIN->UI_Data.data_arry[SELJUNG_PAGE].idata[4]
//#define _SELJUNG_BUZZER     		 pLCD_MAIN->UI_Data.data_arry[SELJUNG_PAGE].idata[5]
//#define _SELJUNG_MAX_WIND           pLCD_MAIN->UI_Data.data_arry[SELJUNG_PAGE].idata[8]
//
//
#define SWAPBYTE_US(X) ((((X) & 0xFF00)>>8) | (((X) & 0x00FF)<<8))
#define BUILD_UINT16(loByte, hiByte) \
          ((int)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define _ClearBit(Data, loc)   ((Data) &= ~(0x1<<(loc)))    // 한 bit Clear
#define _SetBit(Data, loc)     ((Data) |= (0x01 << (loc)))  // 한 bit Set
#define _InvertBit(Data, loc)  ((Data) ^= (0x1 << (loc)))   // 한 bit 반전
#define _CheckBit(Data, loc)   ((Data) & (0x01 << (loc)))   // 비트 검사
#define ON 1
#define OFF 0

//missing string printf
//this is safe and convenient but not exactly efficient
inline std::string format(const char* fmt, ...){
    int size = 512;
    char* buffer = 0;
    buffer = new char[size];
    va_list vl;
    va_start(vl, fmt);
    int nsize = vsnprintf(buffer, size, fmt, vl);
    if(size<=nsize){ //fail delete buffer and try again
        delete[] buffer;
        buffer = 0;
        buffer = new char[nsize+1]; //+1 for /0
        nsize = vsnprintf(buffer, size, fmt, vl);
    }
    std::string ret(buffer);
    va_end(vl);
    delete[] buffer;
    return ret;
}

//
//#define RET_OK 1
//#define RET_ERR 0
////10K Thermist
//#define OHM_LIMIT_LO 0     //0R  OHM
//#define OHM_LIMIT_HI 40000 //40K OHM
typedef enum {
    C_DEFAULT = 0,
    C_RED,
    C_GREEN,
    C_YELLOW,
    C_BLUE,
    C_MAGENTA,
    C_CYAN,
    C_WHITE
} ccolor_t;
void delay_us(uint32_t us);
void cprintf(ccolor_t color, const char *fmt, ...);

extern tja1050 *pCAN;
extern data_class *pDataClass;
extern PWM16 *pPWM;
extern FND595 *pFND595;
extern stm32flash *pFlash_mem;
//extern zigbee *pZIGBEE;
//extern wifiParse *pWIFI;
extern eps *pEPS;

extern SYSTEM_CONF sysConf;
extern uint8_t SYSTEM_setup_data_ok;
extern uint16_t rpm;
extern PID_CONFIG pidCONF;
//extern uint8_t rxBuffer[RX_BUFFER_SIZE];

//--------------------------------------------------
//typedef enum {
//    C_DEFAULT = 0,
//    C_RED,
//    C_GREEN,
//    C_YELLOW,
//    C_BLUE,
//    C_MAGENTA,
//    C_CYAN,
//    C_WHITE
//} ccolor_t;

//static const char* color_table[] = {
//    "\033[0m",   // C_DEFAULT
//    "\033[31m",  // C_RED
//    "\033[32m",  // C_GREEN
//    "\033[33m",  // C_YELLOW
//    "\033[34m",  // C_BLUE
//    "\033[35m",  // C_MAGENTA
//    "\033[36m",  // C_CYAN
//    "\033[37m"   // C_WHITE
//};
//
//void cprintf(ccolor_t color, const char *fmt, ...)
//{
//    va_list args;
//
//    // 색상 시작
//    printf("%s", color_table[color]);
//
//    va_start(args, fmt);
//    vprintf(fmt, args);
//    va_end(args);
//
//    // 색상 리셋
//    printf("\033[0m");
//}
//example
//cprintf(C_RED,"@@@@ adc_raw(%.2f| %.2f) \r\n",    adc_raw1, adc_raw2);
//if (fabs(_rpm.f1 - target_rpm) > 50) { cprintf(C_RED, "RPM ERROR: %.0f\r\n", _rpm.f1);}
//else {   cprintf(C_GREEN, "RPM OK: %.0f\r\n", _rpm.f1);}
//--------------------------------------------------
#endif
