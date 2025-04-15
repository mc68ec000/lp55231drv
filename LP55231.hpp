/***************************************************************************//**
 * @file
 *
 * @details
 *     This file contains the driver class implementation of Ti LP55231, that is 
 * a 9-channel LED driver IC based on I2C interface with 4 selectable slave IDs.
 * It could be used as a 9-channel general-purpose PWM controller or a 9-channel
 * GPIO expander (output only) as well.
 *
 * @note
 *    LP55231 is fully compatible with LP5523.
 *
 * @warning
 *    None
 ******************************************************************************
 */

#ifndef __LP55231_HPP__
#define __LP55231_HPP__

#include <stdint.h>
#include <vector>
#include "i2c.hpp"

/**
 * @addtogroup BSP_Module
 * @{
 *    @addtogroup Device_Drivers
 *    @{
 */

 /*! @} */
/*! @} */

class II2c;

using namespace std;

class LP55231
{
public:

   static const uint8_t LP5569_PROG_MEM_PAGE_SIZE  = 32;
   
   typedef enum
   {
      D1 = 0,
      D2,
      D3,
      D4,
      D5,
      D6,
      D7,
      D8,
      D9,
      NUM_OF_DS
   } eLedIdx_t;
   
   typedef enum
   {
      D_COLOR_RED = 0,
      D_COLOR_GREEN,
      D_COLOR_BLUE,
      D_COLOR_WHITE,
      NUM_OF_D_COLORS
   } eLedColor_t;

   typedef struct 
   {
      eLedIdx_t   ledIndex;
      uint8_t     ledCurrent;
      uint8_t     ledPwmDuty;
      bool        ledSwitch;
   } ledCtrl_t;
   
   typedef enum
   {
      CM_FORCE_EXT_CLK = 0,
      CM_FORCE_INT_CLK,
      CM_AUTO_SELECTION,
      CM_INTERNAL_CLOCK,
      NUM_OF_CLOCK_MODES
   } eClockMode_t;
   
   typedef enum
   {
      CP_MODE_OFF = 0,
      CP_MODE_1X,
      CP_MODE_1P5X,
      CP_MODE_AUTO,
      NUM_OF_CP_MODES
   } eChargePumpMode_t;    
   
   typedef enum
   {
      ENABLE_ENGINE_CTRL1  = 0x0,
      ENGINE_CTRL2         = 0x1,
      OUT_RATIOMETRIC_MSB  = 0x2,
      OUT_RATIOMETRIC_LSB  = 0x3,
      OUT_ON_CONTROL_MSB   = 0x4,
      OUT_ON_CONTROL_LSB   = 0x5,
      D1_CONTROL           = 0x6,
      D2_CONTROL           = 0x7,
      D3_CONTROL           = 0x8,
      D4_CONTROL           = 0x9,
      D5_CONTROL           = 0xA,
      D6_CONTROL           = 0xB,
      D7_CONTROL           = 0xC,
      D8_CONTROL           = 0xD,
      D9_CONTROL           = 0xE,
      D1_PWM               = 0x16,
      D2_PWM               = 0x17,
      D3_PWM               = 0x18,
      D4_PWM               = 0x19,
      D5_PWM               = 0x1A,
      D6_PWM               = 0x1B,
      D7_PWM               = 0x1C,
      D8_PWM               = 0x1D,
      D9_PWM               = 0x1E,
      D1_CURRENT           = 0x26,
      D2_CURRENT           = 0x27,
      D3_CURRENT           = 0x28,
      D4_CURRENT           = 0x29,
      D5_CURRENT           = 0x2A,
      D6_CURRENT           = 0x2B,
      D7_CURRENT           = 0x2C,
      D8_CURRENT           = 0x2D,
      D9_CURRENT           = 0x2E,
      MISC                 = 0x36,
      ENG_1_PC             = 0x37,
      ENG_2_PC             = 0x38,
      ENG_3_PC             = 0x39,
      STATUS_INTERRUPT     = 0x3A,
      INT_GPO              = 0x3B,
      VARIABLE             = 0x3C,
      RESET                = 0x3D,
      TEMP_ADC_CONTROL     = 0x3E,
      TEMPERATURE_READ     = 0x3F,
      TEMPERATURE_WRITE    = 0x40,
      LED_TEST_CONTROL     = 0x41,
      LED_TEST_ADC         = 0x42,
      ENG_1_VAR_A          = 0x45,
      ENG_2_VAR_A          = 0x46,
      ENG_3_VAR_A          = 0x47,
      ENG_1_MASTER_FADER   = 0x48,
      ENG_2_MASTER_FADER   = 0x49,
      ENG_3_MASTER_FADER   = 0x4A,
      ENG_1_PROG_START     = 0x4C,
      ENG_2_PROG_START     = 0x4D,
      ENG_3_PROG_START     = 0x4E,
      PROG_MEM_PAGE_SEL    = 0x4F,
      PROG_MEM_00          = 0x50,
      PROG_MEM_01          = 0x51,
      PROG_MEM_02          = 0x52,
      PROG_MEM_03          = 0x53,
      PROG_MEM_04          = 0x54,
      PROG_MEM_05          = 0x55,
      PROG_MEM_06          = 0x56,
      PROG_MEM_07          = 0x57,
      PROG_MEM_08          = 0x58,
      PROG_MEM_09          = 0x59,
      PROG_MEM_10          = 0x5A,
      PROG_MEM_11          = 0x5B,
      PROG_MEM_12          = 0x5C,
      PROG_MEM_13          = 0x5D,
      PROG_MEM_14          = 0x5E,
      PROG_MEM_15          = 0x5F,
      PROG_MEM_16          = 0x60,
      PROG_MEM_17          = 0x61,
      PROG_MEM_18          = 0x62,
      PROG_MEM_19          = 0x63,
      PROG_MEM_20          = 0x64,
      PROG_MEM_21          = 0x65,
      PROG_MEM_22          = 0x66,
      PROG_MEM_23          = 0x67,
      PROG_MEM_24          = 0x68,
      PROG_MEM_25          = 0x69,
      PROG_MEM_26          = 0x6A,
      PROG_MEM_27          = 0x6B,
      PROG_MEM_28          = 0x6C,
      PROG_MEM_29          = 0x6D,
      PROG_MEM_30          = 0x6E,
      PROG_MEM_31          = 0x6F,
      ENG_1_MAPPING_MSB    = 0x70,
      ENG_1_MAPPING_LSB    = 0x71,
      ENG_2_MAPPING_MSB    = 0x72,
      ENG_2_MAPPING_LSB    = 0x73,
      ENG_3_MAPPING_MSB    = 0x74,
      ENG_3_MAPPING_LSB    = 0x75,
      GAIN_CHANGE_CTRL     = 0X76,
      GENERAL_FAULT        = 0x77,
      NUM_OF_REGISTERS
   } RegAddr_t;   

   LP55231( II2c *pI2c );
   virtual ~LP55231 () {};

   virtual bool Read (uint8_t regOffset, uint8_t &regVal);
   virtual bool Write (uint8_t regOffset, uint8_t regVal);
   virtual bool Write (uint8_t regOffset, uint8_t* regValPtr, uint32_t numBytes);

   void  ResetDevice (void);
   bool  IsReady (void);
   
   bool  SetChipEnable (bool enable);
   bool  SetClockMode (bool internalClk);
   bool  SetClockMode (eClockMode_t clkMode);
   bool  SetChargePump (eChargePumpMode_t cpMode);
   
   void  SetLedCtrl (vector<ledCtrl_t>& LEDVec);
   
   bool  SetAllLedsOn (void);
   bool  SetAllLedsOff (void);
   
   bool  SetLedState (uint8_t led, bool onOff);
   bool  GetLedState (uint8_t led, bool &onOff);
   
   bool  SetLedPwm (uint8_t led, uint8_t pwm);
   bool  GetLedPwm (uint8_t led, uint8_t &pwm);
   
   void  SetGlobalDutyCycle (uint8_t duty);
   void  GetGlobalDutyCycle (uint8_t &duty);
   
   void  SetGlobalCurrent (uint8_t curr);
   void  GetGlobalCurrent (uint8_t &curr);

protected:
   
   CMutex* myMutex;
   
   void    Lock( void );
   void    Unlock( void );
   
private:

   typedef struct
   {
      RegAddr_t   DxControl;
      RegAddr_t   DxPwmDuty;
      RegAddr_t   DxCurrent;
   } LedCtrlRegs_t;
   
   II2c*    m_pI2c;
   uint8_t  m_gDutyCycle;
   uint8_t  m_gCurrent;   
   
   bool     ValidRegAddr (uint32_t regAddr);   
   bool     GetControlRegs (eLedIdx_t ledNum, LedCtrlRegs_t& ledRegs);
};
#endif /* __LP55231_HPP__ */