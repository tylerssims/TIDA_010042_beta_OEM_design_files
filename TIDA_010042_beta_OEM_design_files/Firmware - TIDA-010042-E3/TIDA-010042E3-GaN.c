/*
 * Copyright (c) 2024, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

int high_prot_flag = 0;  // protect signal flag, using like hysteresis comparator
int low_prot_flag = 0;
int prot_count = 0;
int count_flag = 0;
int sample_count = 0;
int read_0_done = 0;
int read_1_done = 0;
int Duty = 180;           //actual duty cycle = 1-Duty/320
int MPPT_Direction = 1;
unsigned int maxFlips = 0;
int panel_power = 0;
int prev_panel_power = 0;
int dutyChange = 2;
int MPPT_flag = 0;
unsigned int Cutoff_Counter = 0;
unsigned int Reconnect_Counter = 0;
unsigned short Hysteresis_ON = 0;
unsigned short LOAD_OC_Triggered = 0;
unsigned short load_high_prot_flag = 0;
unsigned short LOAD_OC_Triggered_Counter = 0;
unsigned short exitCV = 0;
unsigned short CV_Mode = 0;
unsigned short Battery_Current_Counter = 0;
unsigned short MPPT_Loop = 0;
unsigned short MPP_Loop_Exit_Counter = 0;
unsigned short Wait_Counter = 0;
unsigned short Wait_State = 0;

#define COMP0_DAC8_OUTPUT (0xBF)   //2.46V-->60V for input voltage protection
#define BUCK_UPPER_THRESHOLD (32) //90%
#define BUCK_LOWER_THRESHOLD (144) //55%

#define sample_max (8)   //average 8 values
#define right_shift (3)

#define BATTERY_CUTOFF (1034)        //10.2V:517     20.4V:1034 ***
#define BATTERY_RECONNECT (1136)     //11.2V:568     22.4V:1136 ***
#define CUTOFF_COUNTER_THRESHOLD (10)
#define RECONNECT_COUNTER_THRESHOLD (10)  // 0.02 second delay
#define LOAD_OC_TRIGGERED_COUNTER_THRESHOLD (10)
#define LOAD_ENABLE     DL_GPIO_setPins(GPIO_GRP_0_PORT, GPIO_GRP_0_PIN_4_PIN)  //load enable,PB15
#define LOAD_DISABLE    DL_GPIO_clearPins(GPIO_GRP_0_PORT, GPIO_GRP_0_PIN_4_PIN)  //load disable,PB15
#define PANEL_ENABLE    DL_GPIO_setPins(GPIO_GRP_0_PORT, GPIO_GRP_0_PIN_0_PIN)  //panel enable,PB16
#define PANEL_DISABLE   DL_GPIO_clearPins(GPIO_GRP_0_PORT, GPIO_GRP_0_PIN_0_PIN)  //panel disable,PB16

#define CC_LIMIT (2483)                         //20A
#define CC_TO_CV_LIMIT (1338)                   //13.2V:670     26.4V:1338 ***
#define MIN_OUTPUT_CURRENT (10)                 //80mA
#define PANEL_UPPER_LIMIT (3042)                //60V
#define LOOP_EXIT_LIMIT  (100)                  // 0.2s delay
#define MPPT_enable_V   (1521)                  //12Vsys,15V:760        24Vsys,30V:1521   ***
#define MPPT_disable_V  (1354)                  //12Vsys,14.2V:720      24Vsys,26.7V:1354 ***   50.67bits/V
#define Output_current_prot (3102.5)            //25A, 124.1bits/A
#define Output_current_resumption (1241)        //10A
#define Load_current_prot (1241)                //10A
#define Load_current_resumption (1117)          //9A

unsigned int panel_voltage;
unsigned int battery_voltage;
unsigned int panel_current;
unsigned int load_current;
unsigned int output_current;

unsigned int panel_voltage_added;
unsigned int battery_voltage_added;
unsigned int panel_current_added;
unsigned int load_current_added;
unsigned int output_current_added;

unsigned int panel_voltage_averaged;
unsigned int battery_voltage_averaged;
unsigned int panel_current_averaged;
unsigned int load_current_averaged;
unsigned int output_current_averaged;

void Load_Management(void);
void MPPT(void);
void Battery_Charge_Profiling(void);
void PWM_FORCE_LOW(void);
void PWM_FORCE_RELEASE(void);

int main(void)
{
    SYSCFG_DL_init();

    NVIC_EnableIRQ(PWM_1_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_1_INST_INT_IRQN);
    NVIC_EnableIRQ(COMP_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_TEMP_INT_IRQN);

//    DL_GPIO_setPins(GPIO_GRP_0_PORT, GPIO_GRP_0_PIN_0_PIN);  //panel enable,PB16

    DL_COMP_setDACCode0(COMP_0_INST, COMP0_DAC8_OUTPUT);//DL_COMP_REF_SOURCE_VDDA_DAC);
    DL_COMP_enable(COMP_0_INST);

    DL_TimerA_startCounter(PWM_1_INST);
    DL_TimerA_startCounter(TIMER_0_INST);
    DL_TimerG_startCounter(TIMER_1_INST);


    while (1)
    {
        __WFI();
    }
}

void PWM_1_INST_IRQHandler(void)
{
  switch (DL_TimerA_getPendingInterrupt(PWM_1_INST))
  {

  case DL_TIMER_IIDX_ZERO:

          if(sample_count < sample_max)                 //get average sample value
          {
              if((read_0_done & read_1_done) == 1)      //until AD reading finished
              {
                  sample_count += 1;
                  panel_voltage_added = panel_voltage_added + panel_voltage;
                  battery_voltage_added = battery_voltage_added + battery_voltage;
                  panel_current_added = panel_current_added + panel_current;
                  load_current_added = load_current_added + load_current;
                  output_current_added = output_current_added + output_current;
                  read_0_done = 0;
                  read_1_done = 0;
              }
          }
          else
          {
              sample_count = 0;
              panel_voltage_averaged = (panel_voltage_added >> right_shift);    //get average value
              battery_voltage_averaged = (battery_voltage_added >> right_shift);
              panel_current_averaged = (panel_current_added >> right_shift);
              load_current_averaged = (load_current_added >> right_shift);
              output_current_averaged = (output_current_added >> right_shift);

              panel_voltage_added = 0;                                          //clear the added value
              battery_voltage_added = 0;
              panel_current_added = 0;
              load_current_added = 0;
              output_current_added = 0;
          }
//    DL_GPIO_setPins(GPIO_GRP_0_PORT, GPIO_GRP_0_PIN_0_PIN);
    DL_TimerA_setCaptureCompareValue(PWM_1_INST, Duty,DL_TIMER_CC_2_INDEX); // update ccr2 value Duty,120
    DL_TimerA_setCaptureCompareValue(PWM_1_INST, Duty,DL_TIMER_CC_3_INDEX); // update ccr3 value Duty
    break;
  default:
    break;
  }
}



void TIMER_1_INST_IRQHandler(void)      // main function
{
    /* Battery charging only occurs when:
     * 1. The panel voltage is greater than the battery voltage
     * 2. The panel voltage is less than the predefined maximum threshold     */
    if((panel_voltage_averaged > (battery_voltage_averaged-100)) && (panel_voltage_averaged < PANEL_UPPER_LIMIT) && (!Wait_State))
    {                                   // Since we meet all specifications for battery charging, commence creating PWM signals for gate drivers
        PANEL_ENABLE;
        if(MPPT_Loop == 1)
        {
            MPPT();                     // Calculate the MPP if the battery current and voltage are below maximum ratings
        }
        else
        {
            Battery_Charge_Profiling(); // Otherwise, maintain the battery current and voltage in a Float stage
        }
    }

    if((output_current_averaged < MIN_OUTPUT_CURRENT) && (!Wait_State))
    {                                   // Since the battery current is falling under the minimum charging current, we shut off the panel and buck controller
        Battery_Current_Counter++;      // Count how many times this statement is entered
        if((Battery_Current_Counter > 100) || (panel_voltage_averaged > PANEL_UPPER_LIMIT))
        {                               // Once current has fallen for some time, disable panel and buck and go into a waiting state
            PANEL_DISABLE;
            PWM_FORCE_LOW();
            Wait_State = 1;
            Wait_Counter = 0;
        }
    }
    else
    {
        Battery_Current_Counter = 0;
    }

    if(Wait_State)                      // Wait in this statement until sufficient power is running through the system, checking every ~5 seconds
    {
        Wait_Counter++;
        if(Wait_Counter > 2000)
        {
            Wait_State = 0;
            Battery_Current_Counter = 0;
            PWM_FORCE_RELEASE();
        }
    }

    Load_Management();                  //judge if the load current is above limitation and prevent battery from over-discharge
}



void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST))
    {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            panel_voltage = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
            read_0_done = 1;
            break;

        default:
            break;
    }
    DL_ADC12_enableConversions(ADC12_0_INST);
}



void ADC12_1_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_1_INST))
    {
        case DL_ADC12_IIDX_MEM3_RESULT_LOADED:
            battery_voltage = DL_ADC12_getMemResult(ADC12_1_INST, DL_ADC12_MEM_IDX_0);
            panel_current= DL_ADC12_getMemResult(ADC12_1_INST, DL_ADC12_MEM_IDX_1);
            load_current= DL_ADC12_getMemResult(ADC12_1_INST, DL_ADC12_MEM_IDX_2);
            output_current= DL_ADC12_getMemResult(ADC12_1_INST, DL_ADC12_MEM_IDX_3);
            read_1_done = 1;

            if(count_flag == 1)                 //over current, start counting and blank the judge program for 31ms
            {
                prot_count += 1;
                if(prot_count == 1000)          //AD sample time*1000
                {
                    prot_count = 0;
                    count_flag = 0;
                    PWM_FORCE_RELEASE();
                }
            }
            else
            {
                if(high_prot_flag == 0)                         //protection use instant value
                {
                   if(output_current > Output_current_prot)    //25A, 124.12bits/A
                   {
                      PWM_FORCE_LOW();
                      high_prot_flag = 1;
                      count_flag = 1;
                    }
                }
                else
                {
                   if(output_current < Output_current_resumption)     //10A
                   {
                      high_prot_flag = 0;
                   }
                }
            }

            if(load_high_prot_flag == 0)        //judge if load current is above limitation
            {
                if(load_current > Load_current_prot)         //10A, 124.12bits/A
                {
                    LOAD_OC_Triggered = 1;
                    load_high_prot_flag = 1;
                }
            }
            else
            {
                if(load_current < Load_current_resumption)         //9A
                {
                    LOAD_OC_Triggered = 0;
                    load_high_prot_flag = 0;
                }
            }

//                        if(low_prot_flag = 0)
//                        {
//                            if(output_current<800)
//                            {
//                                pwm_count = 190;
//                                pwm_count1 = 190;
//                                pwm_en = 0;
//                                low_prot_flag = 1;
//                            }
//                        }
//                        else
//                        {
//                            if(output_current>850)
//                            {
//                                pwm_en = 1;
//                                low_prot_flag = 0;
//                            }
//                        }

            break;

        default:
            break;
    }
    DL_ADC12_enableConversions(ADC12_1_INST);
}


/* *************************************************************************************** *
 * Call this function to calculate the maximum power point                                 *
 *  The TIDA-010042 uses the perturb-and-observe method of MPPT                            *
 *  The panel power is calculated each time this function is called and compared to the    *
 *  previous power value, then the power point and PWM duty are adjusted accordingly       *
 *  until the maximum value is found                                                       *
 * *************************************************************************************** */
void MPPT(void)
{
    prev_panel_power = panel_power;                                       // Store the previous panel power
    panel_power = panel_voltage_averaged * panel_current_averaged;        // Calculate current panel power

    if(MPPT_flag == 0)
    {
        if(panel_voltage_averaged > MPPT_enable_V)    //12Vsys,15V:760        24Vsys,30V:1521
        {
            MPPT_flag = 1;
        }
    }
    else
    {
        if(panel_voltage_averaged < MPPT_disable_V)    //12Vsys,14.2V:720      24Vsys,26.7V:1354
        {
            MPPT_flag = 0;
        }
    }

    if(MPPT_flag == 1)
    {
        if(panel_power < prev_panel_power)
        {
            MPPT_Direction = MPPT_Direction * -1;
            //Check for steady state
            maxFlips += 1;
            if(maxFlips == 50)
            {
                dutyChange = 1;
            }
        } // end if

        if(MPPT_Direction == 1)
        {
            Duty -= dutyChange;      //D=1-Duty/200

            if(Duty < BUCK_UPPER_THRESHOLD)
            {
                Duty = BUCK_UPPER_THRESHOLD;
            }
        }
        else
        {
            Duty += dutyChange;
            if(Duty > BUCK_LOWER_THRESHOLD)
            {
                Duty = BUCK_LOWER_THRESHOLD;
            }
        }
    }
    else
    {
        Duty = 200;
    }

    if((output_current >= CC_LIMIT) || (battery_voltage_averaged >= CC_TO_CV_LIMIT))    //if output current or battery voltage is above limitation, exit MPPT loop
    {
        MPP_Loop_Exit_Counter++;
        if(MPP_Loop_Exit_Counter > LOOP_EXIT_LIMIT)
        {
            MPPT_Loop = 0;
            MPP_Loop_Exit_Counter = 0;
            if (battery_voltage_averaged > CC_TO_CV_LIMIT)
                CV_Mode = 1;
            else
                CV_Mode = 0;
        }
    }
    else
    {
        MPP_Loop_Exit_Counter = 0;
    }
}



/* *************************************************************************************** *
 * Call this function to disable the load if the battery voltage is too low                *
 *  (below BATTERY_CUTOFF) to prevent over-discharging of the battery                      *
 *  The battery is reconnected once sufficient charge (above BATTERY_RECONNECT) is reached *
 *  If an load overcurrent is triggered with Comparator B, then the load is disabled until *
 *  the OC_Triggered_Count has reached the OC_TRIGGERED_COUNTER_THRESHOLD                  *
 * *************************************************************************************** */
void Load_Management(void)
{
    if((!Hysteresis_ON) && (battery_voltage_averaged < BATTERY_CUTOFF) && (!LOAD_OC_Triggered))
    {
        Cutoff_Counter++;
        if (Cutoff_Counter > CUTOFF_COUNTER_THRESHOLD)
        {
            LOAD_DISABLE;
            Hysteresis_ON = 1;
            Reconnect_Counter = 0;
        } // end if
    }
    else
        Cutoff_Counter = 0; // end if-else

    if((Hysteresis_ON) && (battery_voltage_averaged > BATTERY_RECONNECT) && (!LOAD_OC_Triggered))
    {
        Reconnect_Counter++;
        if (Reconnect_Counter > RECONNECT_COUNTER_THRESHOLD)
        {
            LOAD_ENABLE;
            Hysteresis_ON = 0;
            Cutoff_Counter = 0;
        } // end if
    }
    else
        Reconnect_Counter = 0; // end if-else

    if ((LOAD_OC_Triggered) && (!Hysteresis_ON)) // OC triggered at 8.40 A
    {
        LOAD_OC_Triggered_Counter++;
        if (LOAD_OC_Triggered_Counter > LOAD_OC_TRIGGERED_COUNTER_THRESHOLD)
        {
            LOAD_ENABLE;
            LOAD_OC_Triggered_Counter = 0;
            LOAD_OC_Triggered = 0;
        }
    }
}



/* *************************************************************************************** *
 * Call this function to hold the battery in a FLOAT state when the battery has reached    *
 *  its float voltage                                                                      *
 * *************************************************************************************** */
void Battery_Charge_Profiling(void)
{
#if 1
switch (CV_Mode)
{
case 0: //cc mode
    if((output_current_averaged >= (CC_LIMIT-20)) && (output_current_averaged < (CC_LIMIT+15))) //dont exit loop
    {
        Duty -= 1;
    }
    else if(output_current_averaged > (CC_LIMIT+15))
    {
        Duty += 1;
    }
    else if(output_current_averaged < (CC_LIMIT-20))
    {
        MPPT_Loop=1; //pump more power
    }

    if(battery_voltage_averaged >= CC_TO_CV_LIMIT)
    {
        CV_Mode =1;
    }
    break;

case 1: //Full Chargerd
      if((battery_voltage_averaged >= (CC_TO_CV_LIMIT-20)) && (battery_voltage_averaged < (CC_TO_CV_LIMIT+15))) //dont exit loop
      {
          Duty -= 1;
      }
      else if(battery_voltage_averaged > (CC_TO_CV_LIMIT+20))
      {
          Duty += 1;
      }
      else if((battery_voltage_averaged < (CC_TO_CV_LIMIT-20)) && (output_current_averaged < (CC_LIMIT-40)))
      {
          MPPT_Loop=1; //pump more power
      }

      if(output_current_averaged >= (CC_LIMIT+25))
      {
          exitCV += 1;
          if(exitCV == 10)
          {
              CV_Mode = 0;
              exitCV = 0;
          }
      }
      else
      {
          exitCV = 0;
      }

    break;
default:
    break;
}

//Sanity
if (Duty < BUCK_UPPER_THRESHOLD)
           {
               Duty = BUCK_UPPER_THRESHOLD;
           }

if (Duty > BUCK_LOWER_THRESHOLD)
            {
                Duty = BUCK_LOWER_THRESHOLD;
            }

#endif
}



void GROUP1_IRQHandler(void)
{
    switch (DL_COMP_getPendingInterrupt(COMP_0_INST))                                   //COMP FAULT
    {
        case DL_COMP_IIDX_OUTPUT_EDGE:
            DL_GPIO_clearPins(GPIO_GRP_2_PORT, GPIO_GRP_2_LED2_PIN);                    //clear fault signal
            NVIC_EnableIRQ(PWM_1_INST_INT_IRQN);                                        //enable PWM
            break;
        case DL_COMP_IIDX_OUTPUT_EDGE_INV:
            NVIC_DisableIRQ(PWM_1_INST_INT_IRQN);                                       //disable PWM
            DL_TimerA_setCaptureCompareValue(PWM_1_INST, 319,DL_TIMER_CC_2_INDEX);      // update ccr2 value
            DL_TimerA_setCaptureCompareValue(PWM_1_INST, 319,DL_TIMER_CC_3_INDEX);      // update ccr3 value
            DL_GPIO_setPins(GPIO_GRP_2_PORT, GPIO_GRP_2_LED2_PIN);                      //fault signal, LED2 on
            break;
        default:
            break;
    }

    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))                         //TEMP FAULT
    {
            case GPIO_TEMP_INT_IIDX:
                /* If temp_pin is high, turn the pwm off */
                if (DL_GPIO_readPins(GPIO_TEMP_PORT, GPIO_TEMP_PIN_3_PIN))
                {
                    NVIC_DisableIRQ(PWM_1_INST_INT_IRQN);
                    DL_TimerA_setCaptureCompareValue(PWM_1_INST, 319,DL_TIMER_CC_2_INDEX);  // update ccr2 value
                    DL_TimerA_setCaptureCompareValue(PWM_1_INST, 319,DL_TIMER_CC_3_INDEX);  // update ccr3 value
                    DL_GPIO_setPins(GPIO_GRP_2_PORT, GPIO_GRP_2_LED3_PIN);                  //fault signal, LED3 on
                }
                /* Otherwise, turn the pwm on */
                else
                {
                    Duty = 300;
                    DL_GPIO_clearPins(GPIO_GRP_2_PORT, GPIO_GRP_2_LED3_PIN);                 //clear LED3
                    NVIC_EnableIRQ(PWM_1_INST_INT_IRQN);
                }
                break;
            default:
                break;
     }

}

void PWM_FORCE_LOW(void)
{
    DL_Timer_overrideCCPOut(PWM_1_INST, DL_TIMER_FORCE_OUT_LOW,DL_TIMER_FORCE_CMPL_OUT_LOW,DL_TIMER_CC_2_INDEX);
    DL_Timer_overrideCCPOut(PWM_1_INST, DL_TIMER_FORCE_OUT_LOW,DL_TIMER_FORCE_CMPL_OUT_LOW,DL_TIMER_CC_3_INDEX);
}

void PWM_FORCE_RELEASE(void)
{
    DL_Timer_overrideCCPOut(PWM_1_INST, DL_TIMER_FORCE_OUT_DISABLED,DL_TIMER_FORCE_CMPL_OUT_DISABLED,DL_TIMER_CC_2_INDEX);
    DL_Timer_overrideCCPOut(PWM_1_INST, DL_TIMER_FORCE_OUT_DISABLED,DL_TIMER_FORCE_CMPL_OUT_DISABLED,DL_TIMER_CC_3_INDEX);
}
