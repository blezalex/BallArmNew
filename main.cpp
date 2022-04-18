#include "stm_lib/inc/misc.h"
#include "cmsis_boot/stm32f10x.h"
#include "stm_lib/inc/stm32f10x_dma.h"
#include "stm_lib/inc/stm32f10x_exti.h"
#include "stm_lib/inc/stm32f10x_flash.h"
#include "stm_lib/inc/stm32f10x_gpio.h"
#include "stm_lib/inc/stm32f10x_i2c.h"
#include "stm_lib/inc/stm32f10x_iwdg.h"
#include "stm_lib/inc/stm32f10x_rcc.h"
#include "stm_lib/inc/stm32f10x_adc.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "arduino.h"
#include "balanceController.hpp"
#include "boardController.hpp"
#include "drv/comms/communicator.hpp"
#include "drv/mpu6050/mpu.hpp"
#include "drv/settings/settings.hpp"
#include "drv/vesc/vesc.hpp"
#include "global.h"
#include "guards/angleGuard.hpp"
#include "guards/footpadGuard.hpp"
#include "imu/imu.hpp"
#include "io/genericOut.hpp"
#include "io/i2c.hpp"
#include "io/usart.hpp"
#include "io/rx.h"
#include "ledController.hpp"
#include "lpf.hpp"
#include "pid.hpp"
#include "stateTracker.hpp"
#include "stm32f10x_can.h"


extern "C" void EXTI15_10_IRQHandler(void) {
  if (EXTI_GetITStatus(MPU6050_INT_Exti))  // MPU6050_INT
  {
    EXTI_ClearITPendingBit(MPU6050_INT_Exti);
    mpuHandleDataReady();
  }
}

extern "C" void EXTI0_IRQHandler(void) {
	constexpr uint32_t line = EXTI_Line0;

  // GPIOB->BSRR = GPIO_Pin_3;
  if (EXTI_GetITStatus(line))
  {
    EXTI_ClearITPendingBit(line);
    on_ppm_interrupt();
  }
  
  // GPIOB->BRR = GPIO_Pin_3;
}

class InitWaiter : public UpdateListener {
 public:
  InitWaiter(GenericOut *status_led, IMU *imu, Guard *angle_guard)
      : status_led_(status_led), imu_(imu), angle_guard_(angle_guard) {}

  void processUpdate(const MpuUpdate &update) {
    if (!accGyro.calibrationComplete()) {
      imu_->compute(update, true);
    } else {
      imu_->compute(update);
    }
    angle_guard_->Update();
  }

  void waitForAccGyroCalibration() {
    uint16_t last_check_time = 0;
    while (!accGyro.calibrationComplete() || angle_guard_->CanStart()) {
      IWDG_ReloadCounter();
      if ((uint16_t)(millis() - last_check_time) > 200u) {
        last_check_time = millis();
        status_led_->toggle();
      }
    }
  }

 private:
  GenericOut *status_led_;
  IMU *imu_;
  Guard *angle_guard_;
};

static uint8_t scratch[512];

uint8_t write_pos = 0;
uint8_t read_pos = 0;
static uint8_t debug[200];

static Communicator comms;

void applyCalibrationConfig(const Config &cfg, Mpu *accGyro) {
  int16_t acc_offsets[3] = {(int16_t)cfg.callibration.acc_x,
                            (int16_t)cfg.callibration.acc_y,
                            (int16_t)cfg.callibration.acc_z};
  accGyro->applyAccZeroOffsets(acc_offsets);
}

void initRx() {
	/* GPIO configuration */
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

uint8_t debug_stream_type = 0;

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

typedef enum
{
    FAILED = 0, PASSED = !FAILED
} TestStatus;

GenericOut* status_led_ext;

void buffer2_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

// CAN commands
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS_2,
	CAN_PACKET_STATUS_3,
	CAN_PACKET_STATUS_4,
	CAN_PACKET_PING,
	CAN_PACKET_PONG,
	CAN_PACKET_DETECT_APPLY_ALL_FOC,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
	CAN_PACKET_CONF_CURRENT_LIMITS,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_FOC_ERPMS,
	CAN_PACKET_CONF_STORE_FOC_ERPMS,
	CAN_PACKET_STATUS_5,
	CAN_PACKET_POLL_TS5700N8501_STATUS,
	CAN_PACKET_CONF_BATTERY_CUT,
	CAN_PACKET_CONF_STORE_BATTERY_CUT,
	CAN_PACKET_SHUTDOWN,
	CAN_PACKET_IO_BOARD_ADC_1_TO_4,
	CAN_PACKET_IO_BOARD_ADC_5_TO_8,
	CAN_PACKET_IO_BOARD_ADC_9_TO_12,
	CAN_PACKET_IO_BOARD_DIGITAL_IN,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM,
	CAN_PACKET_BMS_V_TOT,
	CAN_PACKET_BMS_I,
	CAN_PACKET_BMS_AH_WH,
	CAN_PACKET_BMS_V_CELL,
	CAN_PACKET_BMS_BAL,
	CAN_PACKET_BMS_TEMPS,
	CAN_PACKET_BMS_HUM,
	CAN_PACKET_BMS_SOC_SOH_TEMP_STAT,
	CAN_PACKET_PSW_STAT,
	CAN_PACKET_PSW_SWITCH,
	CAN_PACKET_BMS_HW_DATA_1,
	CAN_PACKET_BMS_HW_DATA_2,
	CAN_PACKET_BMS_HW_DATA_3,
	CAN_PACKET_BMS_HW_DATA_4,
	CAN_PACKET_BMS_HW_DATA_5,
	CAN_PACKET_BMS_AH_WH_CHG_TOTAL,
	CAN_PACKET_BMS_AH_WH_DIS_TOTAL,
	CAN_PACKET_UPDATE_PID_POS_OFFSET,
	CAN_PACKET_POLL_ROTOR_POS,
	CAN_PACKET_BMS_BOOT,
	CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} CAN_PACKET_ID;


uint8_t comm_can_transmit_eid_replace(uint32_t id, const uint8_t *data, uint8_t len, bool replace) {
	CanTxMsg TxMessage;
  TxMessage.ExtId = id;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.IDE = CAN_ID_EXT;
  TxMessage.DLC = len;
  memcpy(TxMessage.Data, data, len);

  return CAN_Transmit(CAN1, &TxMessage);
}

uint8_t comm_can_set_current(uint8_t controller_id, float current) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer2_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	return comm_can_transmit_eid_replace(controller_id |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index, true);
}



TestStatus CAN_Polling(void)
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    CanTxMsg TxMessage;
    CanRxMsg RxMessage;
    uint32_t i = 0;
    uint8_t TransmitMailbox = 0;

    /* CAN register init */
    CAN_DeInit( CAN1);
    CAN_StructInit(&CAN_InitStructure);

    /* CAN cell init */
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
    //CAN_InitStructure.CAN_Prescaler = 4*16; //Prescaler for 62.5 kBps
    //CAN_InitStructure.CAN_Prescaler = 4*8; //Prescaler for 125 kBps
    //CAN_InitStructure.CAN_Prescaler = 4*4; //Prescaler for 250 kBps
    CAN_InitStructure.CAN_Prescaler = 4*2; //Prescaler for 500 kBps
    //CAN_InitStructure.CAN_Prescaler = 4*1; //Prescaler for 1 MBps

    CAN_Init(CAN1, &CAN_InitStructure);
    while(CAN_Init(CAN1, &CAN_InitStructure) != CAN_InitStatus_Success);

    /* CAN filter init */
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);



    /* transmit */
    // TxMessage.StdId = 0x11;
    TxMessage.ExtId = 0;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = 0xCA;
    TxMessage.Data[1] = 0xFE;

    TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK)
            && (i != 0xFF))
    {
        i++;
    }

    i = 0;
    while ((CAN_MessagePending(CAN1, CAN_FIFO0) < 1) && (i != 0xFF))
    {
        i++;
    }
//
    /* receive */
    RxMessage.StdId = 0x00;
    RxMessage.IDE = CAN_ID_STD;
    RxMessage.DLC = 0;
    RxMessage.Data[0] = 0x00;
    RxMessage.Data[1] = 0x00;



    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);


//    return FAILED;
//
//    if (RxMessage.StdId != 0x11)
//    {
//        return FAILED;
//    }
//
//    if (RxMessage.IDE != CAN_ID_STD)
//    {
//        return FAILED;
//    }
//
//    if (RxMessage.DLC != 2)
//    {
//        return FAILED;
//    }

//    if (RxMessage.Data[0] == 0x7D)
//    {
//      status_led_ext->toggle();
//    }



    return PASSED; /* Test Passed */
}


int main(void) {
  SystemInit();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  FLASH_SetLatency(FLASH_Latency_1);

  RCC_LSICmd(ENABLE);
  /* Wait till LSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) {
  }



//
//  /*********** CAN **********/
//
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
//
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//
//
//
//  GPIO_InitTypeDef GPIO_InitStructure;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//  GPIO_PinRemapConfig(RCC_APB2Periph_GPIOB , ENABLE);
//
//  while (1) {
//    status_led_ext->toggle();
//    CAN_Polling();
//  }


  /*********** END CAN **********/

//  /* Enable Watchdog*/
//  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
//  IWDG_SetPrescaler(IWDG_Prescaler_8);  // 4, 8, 16 ... 256
//  IWDG_SetReload(
//      0x0FFF);  // This parameter must be a number between 0 and 0x0FFF.
//  IWDG_ReloadCounter();
//  IWDG_Enable();



  initArduino();

  i2c_init();
  Serial1.Init(USART1, 115200);
  Serial2.Init(USART2, 115200);

  Config cfg = Config_init_default;
  if (readSettingsFromFlash(&cfg)) {
    const char msg[] = "Config OK\n";
    Serial1.Send((uint8_t *)msg, sizeof(msg));
    applyCalibrationConfig(cfg, &accGyro);
  } else {
    cfg = Config_init_default;
    const char msg[] = "Config DEFAULT\n";
    Serial1.Send((uint8_t *)msg, sizeof(msg));
  }

  // TODO: Consider running imu and balacning loop outside of interrupt. Set a
  // have_new_data flag in the interrupt, copy data to output if flag was
  // cleared. Ignore data if flag was not cleared. Reset flag in main loop when
  // data is copied.

  // TODO 2:  while(1) must run at specific loop time for any LPFs and pids
  // (footpad, etc to work correctly). Fix the loop time

  GenericOut status_led(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_4, 1);  // red
  status_led.init();
  status_led.setState(0);
  status_led_ext = &status_led;



  /*********** CAN **********/

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);


  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOB, &GPIO_InitStructure);



  GPIO_PinRemapConfig(GPIO_Remap1_CAN1 , ENABLE);

  CAN_Polling();

  while (1) {
    //status_led_ext->toggle();
//  	GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET);
//  	GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);
    while (comm_can_set_current(1, 10) == CAN_TxStatus_NoMailBox) status_led_ext->setState(true);
    while (comm_can_set_current(2, 10) == CAN_TxStatus_NoMailBox) status_led_ext->setState(true);
    while (comm_can_set_current(3, 10) == CAN_TxStatus_NoMailBox) status_led_ext->setState(true);

    status_led_ext->setState(false);
    delay( 10);
  }


	/*********** END CAN **********/


  IMU imu(&cfg);
  AngleGuard angle_guard(imu, &cfg.balance_settings);
  InitWaiter waiter(&status_led, &imu,
                    &angle_guard);  // wait for angle. Wait for pads too?
  accGyro.setListener(&waiter);
  accGyro.init(cfg.balance_settings.global_gyro_lpf);

  GenericOut beeper(RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_12, true);
  beeper.init(true);

  StepperOut::InitAll();
  StepperOut motor1(0);
  StepperOut motor2(1);
  StepperOut motor3(2);

  motor1.set(0);
  motor2.set(0);
  motor3.set(0);

  initRx();

  led_controller_init();
  waiter.waitForAccGyroCalibration();

  GenericOut green_led(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_3, true);
  green_led.init();

  //	GenericOut debug_out(RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_11, false);
  //	debug_out.init();

  //FootpadGuard foot_pad_guard(&cfg.foot_pad);
  Guard *guards[] = {&angle_guard};
  int guards_count = sizeof(guards) / sizeof(Guard *);

  VescComm vesc(&Serial2);
  LPF erpm_lpf(&cfg.misc.erpm_rc);
  LPF v_in_lpf(&cfg.misc.volt_rc);
  LPF duty_lpf(&cfg.misc.duty_rc);



  static BoardController main_ctrl(&cfg, imu, &motor1, &motor2, &motor3, status_led, beeper, guards,
                            guards_count, green_led, &vesc);

  accGyro.setListener(&main_ctrl);

  comms.Init(&Serial1);
  uint16_t last_check_time = 0;

  write_pos = 0;
  read_pos = 0;
  while (1) {  // background work
    IWDG_ReloadCounter();
    led_controller_update();

    if ((uint16_t)(millis() - last_check_time) > 100u) {
      last_check_time = millis();

      led_controller_set_state(vesc.mc_values_.rpm, imu.angles[ANGLE_DRIVE]);
      switch (debug_stream_type) {
        case 1:
          debug[write_pos++] = (int8_t)imu.angles[ANGLE_DRIVE];
          break;
        case 2:
          debug[write_pos++] = (int8_t) (main_ctrl.right / 10);
          break;
        case 3:
          debug[write_pos++] = (int8_t) (main_ctrl.fwd / 10);
          break;
        case 4:
          debug[write_pos++] = (int8_t)(vesc.mc_values_.avg_motor_current);
          break;
        case 5:
          debug[write_pos++] = (int8_t)(vesc.mc_values_.v_in);
          break;
        case 6:
          debug[write_pos++] = (int8_t)(vesc.mc_values_.avg_input_current);
          break;
      }

      if (write_pos >= sizeof(debug)) write_pos = 0;
    }

    uint8_t comms_msg = comms.update();
    switch (comms_msg) {
      case RequestId_READ_CONFIG: {
        int16_t data_len = saveProtoToBuffer(scratch, sizeof(scratch),
                                             Config_fields, &cfg, &Serial1);
        if (data_len > 0) {
          comms.SendMsg(ReplyId_CONFIG, scratch, data_len);
        } else {
          comms.SendMsg(ReplyId_GENERIC_FAIL);
        }
        break;
      }

      case RequestId_GET_DEBUG_BUFFER: {
        // TODO: make sure it all fits in TX buffer or an overrun will occur
        if (write_pos < read_pos) {
          int data_len_tail = sizeof(debug) - read_pos;
          if (data_len_tail > 0) {
            comms.SendMsg(ReplyId_DEBUG_BUFFER, debug + read_pos,
                          data_len_tail);
            read_pos = 0;
          }
        }

        int rem_len = write_pos - read_pos;
        if (rem_len > 0) {
          comms.SendMsg(ReplyId_DEBUG_BUFFER, debug + read_pos, rem_len);
          read_pos = write_pos;
        }
        break;
      }

      case RequestId_WRITE_CONFIG: {
        Config_Callibration c = cfg.callibration;
        bool good =
            readSettingsFromBuffer(&cfg, comms.data(), comms.data_len());
        if (good)
          comms.SendMsg(ReplyId_GENERIC_OK);
        else
          comms.SendMsg(ReplyId_GENERIC_FAIL);
        if (!cfg.has_callibration)  // use own calibration if not received one.
        {
          cfg.callibration = c;
          cfg.has_callibration = true;
        } else {
          applyCalibrationConfig(cfg, &accGyro);
        }
        break;
      }

      case RequestId_GET_STATS: {
        Stats stats = Stats_init_default;
        stats.drive_angle = imu.angles[ANGLE_DRIVE];
        stats.stear_angle = imu.angles[ANGLE_STEER];
        stats.pad_pressure1 = main_ctrl.right;
        stats.pad_pressure1 = main_ctrl.fwd;
        stats.batt_current = main_ctrl.motor1_.get();
        stats.batt_voltage = main_ctrl.motor2_.get();
//        stats.batt_current = vesc.mc_values_.avg_input_current;
//        stats.batt_voltage = vesc.mc_values_.v_in;
        stats.motor_current = vesc.mc_values_.avg_motor_current;
        stats.distance_traveled = vesc.mc_values_.tachometer_abs;
        stats.speed = vesc.mc_values_.rpm;
        stats.motor_duty = vesc.mc_values_.duty_now;
        stats.esc_temp = vesc.mc_values_.temp_mos_filtered;
        stats.motor_temp = vesc.mc_values_.temp_motor_filtered;

        int16_t data_len =
            saveProtoToBuffer(scratch, sizeof(scratch), Stats_fields, &stats);
        if (data_len != -1) {
          comms.SendMsg(ReplyId_STATS, scratch, data_len);
        } else {
          comms.SendMsg(ReplyId_GENERIC_FAIL);
        }
        break;
      }

      case RequestId_SET_DEBUG_STREAM_ID:
      	if (comms.data_len() == 1) {
      		debug_stream_type = comms.data()[0];
      	} else {
      		comms.SendMsg(ReplyId_GENERIC_FAIL);
      	}
      	break;


      case RequestId_CALLIBRATE_ACC:
        comms.SendMsg(ReplyId_GENERIC_OK);
        accGyro.callibrateAcc();
        while (!accGyro.accCalibrationComplete()) IWDG_ReloadCounter();
        cfg.has_callibration = true;
        cfg.callibration.acc_x = accGyro.getAccOffsets()[0];
        cfg.callibration.acc_y = accGyro.getAccOffsets()[1];
        cfg.callibration.acc_z = accGyro.getAccOffsets()[2];
        comms.SendMsg(ReplyId_GENERIC_OK);
        break;

      case RequestId_SAVE_CONFIG:
        saveSettingsToFlash(cfg);
        comms.SendMsg(ReplyId_GENERIC_OK);
        break;
    }

    if (vesc.update() == (uint8_t)VescComm::COMM_PACKET_ID::COMM_GET_VALUES) {
      // got a stats update; recalculate smoothed values
      // Runs at 20hz (values requested from balance controller to sync with
      // current control over USART request.
      vesc.mc_values_.erpm_smoothed = erpm_lpf.compute(vesc.mc_values_.rpm);
      vesc.mc_values_.v_in_smoothed = v_in_lpf.compute(vesc.mc_values_.v_in);
      vesc.mc_values_.duty_smoothed =
          duty_lpf.compute(vesc.mc_values_.duty_now);
    }
  }
}
