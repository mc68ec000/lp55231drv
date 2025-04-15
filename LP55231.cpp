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


#include "LP55231.hpp"
#include "I2C.hpp"
#include "Mutex.hpp"
#include "assert.hpp"

namespace
{
   static const uint8_t RESET_REG_SOFT_RESET_MASK  = 0xFF;
   static const uint8_t CONFIG_REG_CHIP_EN_MASK    = 0x40;
   static const uint8_t MISC_REG_CLK_MODE_MASK     = 0x03;
   static const uint8_t MISC_REG_FORCE_EXT_CLK     = 0x00;
   static const uint8_t MISC_REG_FORCE_INT_CLK     = 0x01;
   static const uint8_t MISC_REG_AUTO_SELECTION    = 0x02;
   static const uint8_t MISC_REG_INTERNAL_CLOCK    = 0x03;   
   static const uint8_t MISC_REG_CP_MODE_MASK      = 0x18;   
   static const uint8_t MISC_REG_CP_MODE_OFF       = 0x00;
   static const uint8_t MISC_REG_CP_MODE_1X        = 0x08;
   static const uint8_t MISC_REG_CP_MODE_1P5X      = 0x10;
   static const uint8_t MISC_REG_CP_MODE_AUTO      = 0x18;

   static const uint8_t DEFAULT_GLOBAL_DUTY_CYCLE  = 0xFF;  // Default brightness : 100%
   static const uint8_t DEFAULT_GLOBAL_LED_CURRENT = 175;   // D current resolutin : 0.1mA/LSB   
}

/***************************************************************************//**
 * Constructor.
 *
 * @param[in]  pI2c        I2C object handle @ref II2c
 *
 * @note
 *    None
 *
 * @warning
 *    None
 ******************************************************************************
 */
LP55231::LP55231 (II2c *pI2c) : m_pI2c (pI2c)
{
   if ( LPI2C_BAUDRATE < m_pI2c->GetClockRate( ) )
   {
      ASSERT_INFO( "I2C clock frequency is too high!!" );
   }
   
   myMutex = new CMutex( "pwmMutex" );
   
   ResetDevice ();
   
   SetChipEnable (true);
   SetClockMode (LP55231::CM_FORCE_INT_CLK); 
   SetChargePump (LP55231::CP_MODE_1X);

   m_gDutyCycle = DEFAULT_GLOBAL_DUTY_CYCLE;
   m_gCurrent = DEFAULT_GLOBAL_LED_CURRENT;
}

void LP55231::Unlock(void)
{
   if( NULL != myMutex )
   {
      myMutex->Unlock( );
   }
}

void LP55231::Lock(void)
{
   if( NULL != myMutex)
   {
      myMutex->Lock( );
   }
}

/***************************************************************************//**
* Reset the device registers.
* 
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
void  LP55231::ResetDevice (void)
{
   uint8_t regAddr = RESET;
   uint8_t regVal = RESET_REG_SOFT_RESET_MASK;        // reset the driver
   
   Write( regAddr, regVal);
}


/***************************************************************************//**
* Get the readiness of the device
* 
* @return      TRUE on ready or FALSE on unready.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/
bool  LP55231::IsReady( void )
{
   bool retVal = false;
   uint8_t regAddr = ENABLE_ENGINE_CTRL1;
   uint8_t regVal;
   
   if( Read( regAddr, regVal ) )
   {
      if( regVal & CONFIG_REG_CHIP_EN_MASK )
      {
         retVal = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Enable the device using Chip Enable
* 
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::SetChipEnable (bool enable)
{
   bool retVal = false;
   uint8_t regAddr = ENABLE_ENGINE_CTRL1;
   uint8_t regVal;
   
   regVal = enable ? CONFIG_REG_CHIP_EN_MASK : 0;
   
   retVal = Write (regAddr, regVal);
   
   return retVal;
}


/***************************************************************************//**
* Set the clock mode
* 
* @param[in]   intClock    TRUE for internal clock or FALSE for external clock
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::SetClockMode (bool intClock)
{
   bool retVal = true;
   uint8_t regAddr = MISC;
   uint8_t regVal = 0;
   
   // Internal Clock Enable control in MISC register
   retVal &= Read (regAddr, regVal);
   
   regVal &= (~MISC_REG_CLK_MODE_MASK & 0xFF);
   regVal |= intClock ? MISC_REG_FORCE_INT_CLK : MISC_REG_FORCE_EXT_CLK;
   
   retVal &= Write (regAddr, regVal);
   
   return retVal;   
}


/***************************************************************************//**
* Set the clock mode
* 
* @param[in]   clkMode     @ref eClockMode_t
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/
bool LP55231::SetClockMode( eClockMode_t clkMode )
{
   bool retVal = true;
   uint8_t regAddr = MISC;
   uint8_t regVal = 0;
   uint8_t clkModeConfig = 0;
   
   // Internal Clock Enable control in MISC register
   retVal &= Read (regAddr, regVal);
   
   regVal &= (~MISC_REG_CLK_MODE_MASK & 0xFF);
   
   switch( clkMode )
   {
   default:
   case CM_FORCE_EXT_CLK:
      clkModeConfig = MISC_REG_FORCE_EXT_CLK;
      break;
      
   case CM_FORCE_INT_CLK:
      clkModeConfig = MISC_REG_FORCE_INT_CLK;
      break;
      
   case CM_AUTO_SELECTION:
      clkModeConfig = MISC_REG_AUTO_SELECTION;
      break;
      
   case CM_INTERNAL_CLOCK:
      clkModeConfig = MISC_REG_INTERNAL_CLOCK;
      break;
   }
   
   regVal |= clkModeConfig;
   
   retVal &= Write (regAddr, regVal);
   
   return retVal;
}


/***************************************************************************//**
* Set Charge Pump Mode
* 
* @param[in]   enalbe   TRUE to enable or FALSE to disable
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::SetChargePump ( eChargePumpMode_t cpMode )
{
   bool retVal = true;
   uint8_t regAddr = MISC;
   uint8_t regVal = 0;
   uint8_t cpModeConfig = 0;
   
   // Set CP_MODE to automatic
   retVal &= Read (regAddr, regVal);
   
   regVal &= (~MISC_REG_CP_MODE_MASK & 0xFF);
   
   switch( cpMode )
   {
   default:
   case CP_MODE_OFF:
      cpModeConfig = MISC_REG_CP_MODE_OFF;
      break;
      
   case CP_MODE_1X:
      cpModeConfig = MISC_REG_CP_MODE_1X;
      break;
      
   case CP_MODE_1P5X:
      cpModeConfig = MISC_REG_CP_MODE_1P5X;
      break;
      
   case CP_MODE_AUTO:
      cpModeConfig = MISC_REG_CP_MODE_AUTO;
      break;
   }
   
   regVal |= cpModeConfig;
   
   retVal &= Write (regAddr, regVal);
   
   return retVal;
}


/***************************************************************************//**
* Turn on or turn off the specific LED.
* 
* @param[in]   ledNum      LED index counting from 0 to 15 @ref eLedIdx_t
* @param[in]   onFlag      True to turn on or False to turn off
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::SetLedState (uint8_t led, bool onOffFlag)
{
   bool retVal = true;
   LedCtrlRegs_t regs;
   eLedIdx_t idx;
   
   idx = static_cast<LP55231::eLedIdx_t>(led);

   if( GetControlRegs( idx, regs ) )
   {
      uint8_t regAddr = 0;
      uint8_t ledControl = 0x00;  
      uint8_t ledPwmDuty = m_gDutyCycle;
      uint8_t ledCurrent = m_gCurrent;   
      
      regAddr = regs.DxControl;
      retVal &= Write( regAddr, ledControl );
      
      regAddr = regs.DxPwmDuty;
      ledPwmDuty = onOffFlag ? m_gDutyCycle : 0;
      retVal &= Write( regAddr, ledPwmDuty );
      
      regAddr = regs.DxCurrent;
      retVal &= Write( regAddr, ledCurrent );
   }
   
   return retVal;
}


/***************************************************************************//**
* Get the state of the specific LED.
* 
* @param[in]   ledNum      LED index counting from 0 to 15 @ref eLedIdx_t
* @param[out]  onOffFlag   True on enabled or Flase on disabled.
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::GetLedState (uint8_t led, bool &onOffFlag)
{
   bool retVal = true;
   LedCtrlRegs_t regs;
   eLedIdx_t idx;
   
   idx = static_cast<LP55231::eLedIdx_t>(led);
   onOffFlag = false;

   if( GetControlRegs( idx, regs ) )
   {
      uint8_t regAddr = 0;
      uint8_t ledPwmDuty;
      
      regAddr = regs.DxPwmDuty;
      retVal = Read (regAddr, ledPwmDuty );
      
      if (0 < ledPwmDuty)
      {
         onOffFlag = true;
      }
   }
   
   return retVal;
}


/***************************************************************************//**
* Set the PWM Duty Cycle of the channel.
* 
* @param[in]   ledNum      LED index counting from 0 to 15 @ref eLedIdx_t
* @param[in]   pwm         Duty Cycle
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::SetLedPwm (uint8_t led, uint8_t pwm)
{
   bool retVal = true;
   LedCtrlRegs_t regs;
   eLedIdx_t idx;
   uint8_t regAddr;
   
   idx = static_cast<LP55231::eLedIdx_t>(led);

   if( GetControlRegs( idx, regs ) )
   {
      regAddr = regs.DxPwmDuty;
      
      retVal = Write( regAddr, pwm );
   }
   
   return retVal;
}


/***************************************************************************//**
* Get the PWM Duty Cycle of the channel.
* 
* @param[in]   ledNum      LED index counting from 0 to 15 @ref eLedIdx_t
* @param[in]   pwm         Duty Cycle
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::GetLedPwm (uint8_t led, uint8_t &pwm)
{
   bool retVal = true;
   LedCtrlRegs_t regs;
   eLedIdx_t idx;
   uint8_t regAddr;
   uint8_t ledPwm;
   
   idx = static_cast<LP55231::eLedIdx_t>(led);

   if( GetControlRegs( idx, regs ) )
   {
      regAddr = regs.DxPwmDuty;
      
      retVal = Read( regAddr, ledPwm );
      
      pwm = ledPwm;
   }
   
   return retVal;
}


/***************************************************************************//**
* Turn on all the LEDs belong to this LED driver.
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::SetAllLedsOn (void)
{
   bool retVal = true;
   uint8_t regAddr = 0;
   uint8_t ledControl[NUM_OF_DS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // No master fading, default dimming, linear adjustment, powered by charge pump or external
   uint8_t ledPwmDuty[NUM_OF_DS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // Duty cycle for brightness control
   uint8_t ledCurrent[NUM_OF_DS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // Current resolution: 0.1mA/LSB
   
   Lock();
   
   for (uint8_t i = 0; i < NUM_OF_DS; i++)
   {
      ledPwmDuty[i] = m_gDutyCycle;
      ledCurrent[i] = m_gCurrent;
   }
   
   regAddr = D1_CONTROL;
   retVal &= m_pI2c->Write( regAddr, ledControl, NUM_OF_DS );   

   regAddr = D1_PWM;
   retVal &= m_pI2c->Write( regAddr, ledPwmDuty, NUM_OF_DS );   

   regAddr = D1_CURRENT;
   retVal &= m_pI2c->Write( regAddr, ledCurrent, NUM_OF_DS );
   
   Unlock();
   
   return retVal;
}


/***************************************************************************//**
* Turn on all the LEDs belong to this LED driver.
*
* @return      TRUE on success or FALSE on failure.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::SetAllLedsOff (void)
{
   bool retVal = true;
   uint8_t regAddr = 0;
   uint8_t ledDuty[NUM_OF_DS] = {0, 0, 0, 0, 0, 0, 0, 0, 0};  //0% duty
   
   Lock();
   
   regAddr = D1_PWM;
   retVal = m_pI2c->Write( regAddr, ledDuty, NUM_OF_DS );
   
   Unlock();
   
   return retVal;
}


/***************************************************************************//**
* Set the global LED PWM duty cycle.
*
* @param[in]   duty     Duty cycle.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
void LP55231::SetGlobalDutyCycle (uint8_t duty)
{
   m_gDutyCycle = duty;
}


/***************************************************************************//**
* Get the global LED PWM duty cycle.
*
* @param[out]  duty     Duty cycle
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
void LP55231::GetGlobalDutyCycle (uint8_t &duty)
{
   duty = m_gDutyCycle;
}


/***************************************************************************//**
* Set the global LED current in one-tenth mA.
*
* @param[in]   current  Current in 1/10 mA.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
void LP55231::SetGlobalCurrent (uint8_t current)
{
   m_gCurrent = current;
}


/***************************************************************************//**
* Get the global LED current in one-tenth mA.
*
* @param[out]  current  Current in 1/10 mA.
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
void LP55231::GetGlobalCurrent (uint8_t &current)
{
   current = m_gCurrent;
}


/***************************************************************************//**
* Set LEDs on. todo:Vince: Merge it to sharecode branch
* 
* @param[in]   vector<LEDCtr_t>     vector of the leds ctrl flags
* 
*
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
void LP55231::SetLedCtrl (vector<ledCtrl_t>& LEDVec)
{
   Lock();
   
   for(auto ledCtr : LEDVec) 
   {
      uint8_t  txBuff[2] = {0, 0};
      LedCtrlRegs_t regs;
      
      if (GetControlRegs (ledCtr.ledIndex, regs))
      {
         txBuff[0] = regs.DxControl;
         txBuff[1] = 0x00;        //no master fading, default dimming, linear adjustment, powered by charge pump or external
         m_pI2c->Write (txBuff, 2);
         
         txBuff[0] = regs.DxPwmDuty;
         txBuff[1] = ledCtr.ledSwitch ? ledCtr.ledPwmDuty : 0;            // default 0% duty
         m_pI2c->Write (txBuff, 2);
         
         txBuff[0] = regs.DxCurrent;
         txBuff[1] = ledCtr.ledCurrent;       
         m_pI2c->Write (txBuff, 2);         
      }
   }
   
   Unlock();
}


/***************************************************************************//**
* Read from the LED driver register.
* 
* @param[in]   regOffset   Register address offset
* @param[out]  regVal      Register value
*
* @returns     TRUE on succeeded or FASLSE on failed
* 
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::Read (uint8_t regOffset, uint8_t &regVal)
{
   bool retVal = false;
   uint8_t  ctrlByte = 0;        

   Lock();
   
   if( ValidRegAddr(  regOffset ) )
   {
      if ( m_pI2c )
      {
         retVal = m_pI2c->Write( regOffset, &ctrlByte, 0 );
         if( retVal )
         {
            retVal = m_pI2c->Read( &regVal, sizeof(uint8_t) );
         }
      }
      else
      {
         ASSERT_INFO( "I2C not initialized" );
      }
   }
   
   Unlock();

   return retVal;
}


/***************************************************************************//**
* Write to the LED driver register.
* 
* @param[in]   regOffset   Register address offset
* @param[in]   regVal      Register value
*
* @returns     TRUE on succeeded or FASLSE on failed
* 
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::Write (uint8_t regOffset, uint8_t regValue)
{
   bool retVal = false;

   Lock();
   
   if( ValidRegAddr( regOffset )  )
   {
      if ( m_pI2c )
      {
         retVal = m_pI2c->Write( regOffset, &regValue, sizeof( uint8_t));
      }
      else
      {
         ASSERT_INFO( "I2C not initialized" );
      }
   }
   
   Unlock();

   return retVal;
}


/***************************************************************************//**
* Write multiple LED driver registers
* 
* @param[in]   regOffset   Register address offset
* @param[in]   regValPrt   Pointer to write buffer
* @param[in]   numBytes    Num of bytes to write
*
* @returns     TRUE on succeeded or FASLSE on failed
* 
* @note        None.
* 
* @warning     None.
*******************************************************************************/ 
bool LP55231::Write (uint8_t regOffset, uint8_t* regValPtr, uint32_t numBytes)
{
   bool retVal = false;

   Lock();
   
   if( ValidRegAddr( regOffset )  )
   {
      if ( m_pI2c )
      {
         // write to io expander register
         retVal = m_pI2c->Write (regOffset, regValPtr, numBytes);
      }
      else
      {
         ASSERT_INFO ("I2C not initialized");
      }
   }
   else
   {
      ASSERT_INFO ("Write outside of memory");
   }
   
   Unlock();

   return retVal;
}


/*******************************************************************************
 * Helper function to check if it is a valid register
 *
 * @param[in]  regAddr     Register address
 *
 * @return     TRUE on valid or FALSE on invalid
 *
 * @note       None.
 * 
 * @warning    None.
 *******************************************************************************
 */
bool LP55231::ValidRegAddr (uint32_t regAddr)
{
   bool rc = false;
   
   if (regAddr < NUM_OF_REGISTERS)
   {
      rc = true;
   }
   
   return rc;
}


/***************************************************************************//**
* Helper function to get the LED config register file
*
* @return   TRUE on succeeded or FALSE on failed            
*
* @note
*     None.
*
* @warning
*     None.
*******************************************************************************/
bool LP55231::GetControlRegs (eLedIdx_t ledNum, LedCtrlRegs_t& ledRegs)
{
   bool rc = false;
   
   if (NUM_OF_DS > ledNum)
   {
      ledRegs.DxControl = static_cast<LP55231::RegAddr_t>(static_cast<uint8_t>(D1_CONTROL) + static_cast<uint8_t>(ledNum));
      ledRegs.DxPwmDuty = static_cast<LP55231::RegAddr_t>(static_cast<uint8_t>(D1_PWM) + static_cast<uint8_t>(ledNum));
      ledRegs.DxCurrent = static_cast<LP55231::RegAddr_t>(static_cast<uint8_t>(D1_CURRENT) + static_cast<uint8_t>(ledNum));
      
      rc = true;
   }
   
   return rc;
}