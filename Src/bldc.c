/*
* This file has been re-implemented with 4 new selectable motor control methods.
* Recommended control method: 3 = Sinusoidal 3rd order. This control method offers superior performanace
* compared to previous method. The new method features:
* ► reduced noise and vibrations
* ► smooth torque output
* ► improved motor efficiency -> lower energy consumption
*
* Copyright (C) 2019 Emanuel FERU <aerdronix@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "at32f4xx.h"
#include "defines.h"
#include "setup.h"
#include "config.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

extern RT_MODEL *const rtM_Left;
extern RT_MODEL *const rtM_Right;

extern DW rtDW_Left;                    /* Observable states */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtY rtY_Left;                   /* External outputs */

extern DW rtDW_Right;                   /* Observable states */
extern ExtU rtU_Right;                  /* External inputs */
extern ExtY rtY_Right;                  /* External outputs */
// ###############################################################################


volatile int pwml = 0;
volatile int pwmr = 0;

extern volatile adc_buf_t adc_buffer;

extern volatile uint32_t timeout;

uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
static uint32_t buzzerTimer = 0;

uint8_t enable          = 0;

static const uint16_t pwm_res       = 64000000 / 2 / PWM_FREQ; // = 2000

static int offsetcount = 0;
static int offsetrl1   = 2000;
static int offsetrl2   = 2000;
static int offsetrr1   = 2000;
static int offsetrr2   = 2000;
static int offsetdcl   = 2000;
static int offsetdcr   = 2000;

volatile uint8_t hall_idx_left;
volatile uint8_t hall_idx_right;

float batteryVoltage = BAT_NUMBER_OF_CELLS * 4.0;

// map hall sensor values based on one of 6 possible mappings
void hall_map(uint8_t *hall_values, uint8_t mapping)
{
  uint8_t out[3];
  switch(mapping) {
  case 0:
    out[0] = hall_values[0]; out[1] = hall_values[1]; out[2] = hall_values[2];
    break;
  case 1:
    out[0] = hall_values[0]; out[1] = hall_values[2]; out[2] = hall_values[1];
    break;
  case 2:
    out[0] = hall_values[1]; out[1] = hall_values[0]; out[2] = hall_values[2];
    break;
  case 3:
    out[0] = hall_values[1]; out[1] = hall_values[2]; out[2] = hall_values[0];
    break;
  case 4:
    out[0] = hall_values[2]; out[1] = hall_values[0]; out[2] = hall_values[1];
    break;
  case 5:
    out[0] = hall_values[2]; out[1] = hall_values[1]; out[2] = hall_values[0];
    break;
  }
  hall_values[0] = out[0]; hall_values[1] = out[1]; hall_values[2] = out[2];
}

//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
void DMA1_Channel1_IRQHandler(void) {

  DMA1->ICLR = DMA_ICLR_CTCIF1;
  // GPIO_WriteBit(LED_PORT, LED_PIN, 1);
  // GPIO_WriteBit(LED_PORT, LED_PIN, 0);

  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
    offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
    offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
    offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
    batteryVoltage = batteryVoltage * 0.99f + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01f;
  }

  //disable PWM when current limit is reached (current chopping)
  if(ABS((adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP) > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
    TMR_CtrlPWMOutputs(LEFT_TIM, DISABLE);
    //GPIO_WriteBit(LED_PORT, LED_PIN, 1);
  } else {
    TMR_CtrlPWMOutputs(LEFT_TIM, ENABLE);
    //GPIO_WriteBit(LED_PORT, LED_PIN, 0);
  }

  if(ABS((adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP)  > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
    TMR_CtrlPWMOutputs(RIGHT_TIM, DISABLE);
  } else {
    TMR_CtrlPWMOutputs(RIGHT_TIM, ENABLE);
  }

  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      GPIO_WriteBit(BUZZER_PORT, BUZZER_PIN, !GPIO_ReadOutputDataBit(BUZZER_PORT, BUZZER_PIN));
    }
  } else {
      GPIO_WriteBit(BUZZER_PORT, BUZZER_PIN, 0);
  }

  // ############################### MOTOR CONTROL ###############################

  static boolean_T OverrunFlag = false;
  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }
  OverrunFlag = true;
 
  int ul, vl, wl;
  int ur, vr, wr;
  // ========================= LEFT MOTOR ============================ 
    // Get hall sensors values
    uint8_t hall_sensors[3];
    hall_sensors[0] = !(LEFT_HALL_U_PORT->IPTDT & LEFT_HALL_U_PIN);
    hall_sensors[1] = !(LEFT_HALL_W_PORT->IPTDT & LEFT_HALL_W_PIN);
    hall_sensors[2] = !(LEFT_HALL_V_PORT->IPTDT & LEFT_HALL_V_PIN);
    hall_map(hall_sensors, hall_idx_left);

    /* Set motor inputs here */
    rtU_Left.b_hallA   = hall_sensors[0];
    rtU_Left.b_hallB   = hall_sensors[1];
    rtU_Left.b_hallC   = hall_sensors[2];
    rtU_Left.r_DC      = pwml;
    
    /* Step the controller */
    BLDC_controller_step(rtM_Left);

    /* Get motor outputs here */
    ul            = rtY_Left.DC_phaA;
    vl            = rtY_Left.DC_phaB;
    wl            = rtY_Left.DC_phaC;
  // motSpeedLeft = rtY_Left.n_mot;
  // motAngleLeft = rtY_Left.a_elecAngle;

    /* Apply commands */
    LEFT_TIM->LEFT_TIM_U    = (uint16_t)CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_V    = (uint16_t)CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
    LEFT_TIM->LEFT_TIM_W    = (uint16_t)CLAMP(wl + pwm_res / 2, 10, pwm_res-10);
  // =================================================================
  

  // ========================= RIGHT MOTOR ===========================  
    // Get hall sensors values
    hall_sensors[0] = !(RIGHT_HALL_U_PORT->IPTDT & RIGHT_HALL_U_PIN);
    hall_sensors[1] = !(RIGHT_HALL_V_PORT->IPTDT & RIGHT_HALL_V_PIN);
    hall_sensors[2] = !(RIGHT_HALL_W_PORT->IPTDT & RIGHT_HALL_W_PIN);
    hall_map(hall_sensors, hall_idx_right);

    /* Set motor inputs here */
    rtU_Right.b_hallA  = hall_sensors[0];
    rtU_Right.b_hallB  = hall_sensors[1];
    rtU_Right.b_hallC  = hall_sensors[2];
    rtU_Right.r_DC     = pwmr;

    /* Step the controller */
    BLDC_controller_step(rtM_Right);

    /* Get motor outputs here */
    ur            = rtY_Right.DC_phaA;
    vr            = rtY_Right.DC_phaB;
    wr            = rtY_Right.DC_phaC;
 // motSpeedRight = rtY_Right.n_mot;
 // motAngleRight = rtY_Right.a_elecAngle;

    /* Apply commands */
    RIGHT_TIM->RIGHT_TIM_U  = (uint16_t)CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_V  = (uint16_t)CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_W  = (uint16_t)CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
  // =================================================================

  /* Indicate task complete */
  OverrunFlag = false;
 
 // ###############################################################################

}
