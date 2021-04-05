#include "Driver_CAN.h"                	// ::CMSIS Driver:CAN
#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "Board_LED.h"                  // ::Board Support:LED

#ifdef _RTE_
#include "RTE_Components.h"             // Component selection
#endif
#ifdef RTE_CMSIS_RTOS2                  // when RTE component CMSIS RTOS2 is used
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#endif


#ifdef RTE_CMSIS_RTOS2_RTX5
/**
  * Override default HAL_GetTick function
  */
uint32_t HAL_GetTick (void) {
  static uint32_t ticks = 0U;
         uint32_t i;

  if (osKernelGetState () == osKernelRunning) {
    return ((uint32_t)osKernelGetTickCount ());
  }

  /* If Kernel is not running wait approximately 1 ms then increment 
     and return auxiliary tick counter value */
  for (i = (SystemCoreClock >> 14U); i > 0U; i--) {
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
  }
  return ++ticks;
}

#endif

extern ARM_DRIVER_CAN Driver_CAN1;
extern ARM_DRIVER_CAN Driver_CAN2;

extern osThreadId ID_CAN_Transmiter;
extern osThreadId ID_CAN_Receiver ;

void Callback_CAN_Receiver(int obj_idx, int event)
{
    switch (event)
    {
    case ARM_CAN_EVENT_RECEIVE:
        /*  Message was received successfully by the obj_idx object. */
				osSignalSet(ID_CAN_Receiver, 0x01);
				break;
    }
}

void Callback_CAN_Transmiter(int obj_idx, int event)
{
    switch (event)
    {
    case ARM_CAN_EVENT_SEND_COMPLETE:
        /*  Message was received successfully by the obj_idx object. */
       osSignalSet(ID_CAN_Transmiter, 0x01);
        break;
    }
}

void init_CAN_Transmiter()
{
	Driver_CAN2.Initialize(NULL,Callback_CAN_Transmiter);
	Driver_CAN2.PowerControl(ARM_POWER_FULL);
	Driver_CAN2.SetMode(	ARM_CAN_MODE_INITIALIZATION);
	Driver_CAN2.SetBitrate( ARM_CAN_BITRATE_NOMINAL,
													125000,
													ARM_CAN_BIT_PROP_SEG(5U)|
													ARM_CAN_BIT_PHASE_SEG1(1U)|
													ARM_CAN_BIT_PHASE_SEG2(1U)|
													ARM_CAN_BIT_SJW(1U));	
	Driver_CAN2.ObjectConfigure(1,ARM_CAN_OBJ_TX);
	Driver_CAN2.SetMode(ARM_CAN_MODE_NORMAL);
}

void init_CAN_Receiver()
{
	Driver_CAN1.Initialize(NULL,Callback_CAN_Receiver);
	Driver_CAN1.PowerControl(ARM_POWER_FULL);
	Driver_CAN1.SetMode(ARM_CAN_MODE_INITIALIZATION);
	Driver_CAN1.SetBitrate( ARM_CAN_BITRATE_NOMINAL,
													125000,
													ARM_CAN_BIT_PROP_SEG(5U)   |         // Set propagation segment to 5 time quanta
                          ARM_CAN_BIT_PHASE_SEG1(1U) |         // Set phase segment 1 to 1 time quantum (sample point at 87.5% of bit time)
                          ARM_CAN_BIT_PHASE_SEG2(1U) |         // Set phase segment 2 to 1 time quantum (total bit is 8 time quanta long)
                          ARM_CAN_BIT_SJW(1U));                // Resynchronization jump width is same as phase segment 2
	Driver_CAN1.ObjectConfigure(0,ARM_CAN_OBJ_RX);
	Driver_CAN1.SetMode(ARM_CAN_MODE_NORMAL);
}