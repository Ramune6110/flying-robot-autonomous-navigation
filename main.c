/* Board : Avionics Lite
 * Author : Keigo Miyama 
 * ********************************************************************************
 * SYSTEM
 * MCU          | STM32F103VET6
 * XBee         | Xbee-S2C
 * SENSOR
 * Accel/Gyro   | MPU6000
 * Mag          | MAG3110FCR1
 * Barometer    | BMP280
 * ********************************************************************************
 */

#include "stm32f10x.h"
#include "tsprintf.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* Sensor constant value */
#define ACCEL_2G_LSB    16384.0
#define GYRO_500DEG_LSB 65.5
#define MAG_LSB         32768.0

#define AILERON_DUTY_VAL  1
#define ELEVATOR_DUTY_VAL 1
#define RUDDER_DUTY_VAL   1

/* Math Calculation Value */
#define GRAVITY_ACCLE 9.80665

/* MPU6000 */
#define MPU6000_ADDRESS             0b11010010
#define MPU6000_CONFIG              0x1A
#define MPU6000_GYRO_CONFIG         0x1B
#define MPU6000_ACCEL_CONFIG        0x1C
#define MPU6000_I2C_MST_CTRL        0x24
#define MPU6000_INT_PIN_CONFIG      0x37
#define MPU6000_ACCEL_XOUT_H        0x3B
#define MPU6000_SIGNAL_PATH_RESET   0x68
#define MPU6000_PWR_MGMT_1          0x6B
#define MPU6000_WHO_AM_I            0x75

#define MPU6000_CHIPID              0x68

/* MAG3110 */
#define MAG3110_ADDRESS             0b00011100
#define MAG3110_DR_STATUS           0x00
#define MAG3110_OUT_X_MSB           0x01
#define MAG3110_WHO_AM_I            0x07
#define MAG3110_SYSMOD              0x08
#define MAG3110_DIE_TEMP            0x0F
#define MAG3110_CTRL_REG1           0x10
#define MAG3110_CTRL_REG2           0x11

#define MAG3110_CHIPID              0xC4

/* BMP280 */
#define BMP280_ADDRESS              0b11101100
#define BMP280_DIG_T1               0x88
#define BMP280_WHO_AM_I             0xD0
#define BMP280_RESET                0xE0
#define BMP280_POWER_ON_RESET       0xB6
#define BMP280_CTRL_MEAS            0xF4
#define BMP280_CONFIG               0xF5
#define BMP280_PRESS                0xF7
#define BMP280_TEMP                 0xFA

#define BMP280_CHIPID               0x58

/* Arduino */
#define ARDUINO_ADDRESS             1

#define PE0_5           0x003f
#define TIM1_CH1        GPIO_Pin_9
#define TIM1_CH2        GPIO_Pin_11
#define TIM1_CH3        GPIO_Pin_13
#define TIM1_CH4        GPIO_Pin_14

#define TIM4_CH1        GPIO_Pin_12
#define TIM4_CH2        GPIO_Pin_13
#define TIM4_CH3        GPIO_Pin_14
#define TIM4_CH4        GPIO_Pin_15

#define EXTI0_PRIORITY      1
#define EXTI1_PRIORITY      2
#define EXTI2_PRIORITY      3
#define EXTI3_PRIORITY      4
#define EXTI4_PRIORITY      5
#define EXTI9_5_PRIORITY    6
#define TIM8_PRIORITY       7
#define TIM3_PRIORITY       10
#define TIM6_PRIORITY       12
#define TIM5_PRIORITY       13
#define TIM2_PRIORITY       20

/***********************************************************************
 * Prototype declaration
 **********************************************************************/
// System function
void BoardInit(void);
void GPIO_Configuration(void);
void SysTick_Configuration(void);

// Timer function
void TIM1_Configuration(void);            // Servo PWM
void TIM2_Configuration(void);            // Exception handling
void TIM3_Configuration(void);            // 1kHz Timer
void TIM4_Configuration(void);            // Drop System
void TIM5_Configuration(void);            // Sensor getParameter
void TIM6_Configuration(void);            // PID
void TIM7_Configuration(uint16_t usec);   // delay function
void TIM8_Configuration(void);            // Receiver getParameter

void EXTI_Configuration(void);            // NVIC Config + EXTI Config
void EXTI0_IRQHandler(void);              // Receiver Parameter[1]
void EXTI1_IRQHandler(void);              // Receiver Parameter[2]
void EXTI2_IRQHandler(void);              // Receiver Parameter[3]
void EXTI3_IRQHandler(void);              // Receiver Parameter[4]
void EXTI4_IRQHandler(void);              // Receiver Parameter[5]
void EXTI9_5_IRQHandler(void);            // Receiver Parameter[6]

void NVIC_Configuration(void);            // xbee receive function

void Convert_ReceiverData_to_PWM(void);


// Serial function
void I2C1_Configuration(void);
void Arduino_getParameter(void);
uint8_t i2c1Read(uint8_t Slave_Address, uint8_t Register_Address);
void i2c1Write(uint8_t Slave_Address, uint8_t Register_Address, uint8_t Data);

void I2C2_Configuration(void);
uint8_t i2c2Read(uint8_t Slave_Address, uint8_t Register_Address);
void i2c2Write(uint8_t Slave_Address, uint8_t Register_Address, uint8_t Data);

void USART1_Configuration(void);
void DMA1_Configuration(uint32_t Memory_Address, uint16_t Buffer_Size);
void sendData(const int8_t String[]);

// Sensor function
bool Sensor_ConectionCheck(void);
//MPU6000
void MPU6000_Configuration(void);
void MPU6000_ConfigCheck(void);
void MPU6000_getParameter(void);
//MAG3110
void MAG3110_Active(void);
void MAG3110_Standby(void);
void MAG3110_Reset(void);
void MAG3110_Configuration(void);
bool MAG3110_dataReady(void);
void MAG3110_getParameter(void);
void MAG3110_Calibration(void);
//BMP280
void BMP280_Configuration(void);
void BMP280_getTrimParameter(void);
void BMP280_getParameter(void);
void BMP280_temperature(void);
void BMP280_pressure(void);
void BMP280_altitude(void);

uint8_t arduinoRead(uint8_t Slave_Address);

// PID Function
void PID_Controller(void);
void PI_D_Controller(void);
void I_PD_Controller(void);
void PID_Roll_Controller(void);
void PI_D_Roll_Controller(void);
void I_PD_Roll_Controller(void);
void antiWindUp(void);
int32_t aileronDeg2Duty(double u);
int32_t elevatorDeg2Duty(double u);
int32_t rudderDeg2Duty(double u);
int32_t normalize(int32_t val, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax);

// MadgwickAHRS
void Madgwick_Init(void);
void MadgwickAHRS(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
void MadgwickAHRS_IMU(double gx, double gy, double gz, double ax, double ay, double az);
double getPhi(void);
double getTheta(void);
double getPsi(void);
double invSqrt(double x);
void computeAngles(void);

// General purpose function
void Toggle_LED(void);
void _delay_us(uint16_t usec);
void _delay_ms(uint16_t msec);
void sendDataOption(void);
void sendDataOffset(void);
void sendDataCalibration(void);
void sendDataPID(void);

void ADC_DMA_Configuration(void);
__IO int16_t ADCConvertedValue;

void PI_D_Controller(void);
void I_PD_Controller(void);

/**********************************************************
 * Variable
 *********************************************************/
/* Sensor variable */
int32_t count = 0;
int16_t binary_oneByteData[26] = {};   //8bits binary data
int32_t mpu6000_rawData[7] = {};
int32_t mag3110_rawData[6] = {};
int32_t bmp280_digData[12] = {};

int32_t Pre_press_correctionfactor = 0;
int32_t Pre_temp_correctionfactor = 0;
int32_t temperature;
int32_t t_fine;
uint32_t pressure;
int32_t P0 = 100530;
double T = 26.0;
double press_;
double altitude;

int16_t ccnt = 0;
int32_t accelCalib[3] = {};
int32_t magCalib[3] = {};
int32_t magMax[3] = {-32768,-32768,-32768};
int32_t magMin[3] = {32767,32767,32767};
int32_t accelDataSum[3] = {};
int32_t magDataSum[3] = {};
int32_t accelOffset[3] = {};
int32_t magOffset[3] = {-4286, 3706, 1927};

double accelData[3] = {};
double mpuTemp = 0.0;
double gyroData[3] = {};
double magData[3] = {};
double magTemp = 0.0;

// PID
int16_t ControlMode = 0;
int16_t preControlMode = 0;
int16_t ControlCounter = 0;
#define SAMPLING_RATE 0.05      //[s]
// PID Control Input
double U[3] = {0.0, 0.0, 0.0};
double U_p[3] = {0.0, 0.0, 0.0};
// Error
double phi_e;
double theta_e;
double psi_e;
//one step before Error
double phi_e_1 = 0.0;
double theta_e_1 = 0.0;
double psi_e_1 = 0.0;
//two step before Error
double phi_e_2 = 0.0;
double theta_e_2 =0.0;
double psi_e_2 = 0.0;
//
double phi_1 = 0.0;
double phi_2 = 0.0;
double theta_1 = 0.0;
double theta_2 = 0.0;
double psi_1 = 0.0;
double psi_2 = 0.0;

// Input
int32_t Aileron_u;
int32_t Elevator_u;
int32_t Rudder_u;
float Ud_a;
float Ud_e;
float Ud_r;
float Ud_pa;
float Ud_pe;
float Ud_pr;
float du_a;
float du_e;
float du_r;
//rollrate_controll
float rollrate_e;
float rollrate_e_1;
float rollrate_e_2;
float rollrate_1;
float rollrate_2;
/* PID GAIN */
double KP_A = 0.0;
double TD_A = 0.0;
double TI_A = 0.0;
double ITA_A = 0.0;
double KP_E = 0.0;
double TD_E = 0.0;
double TI_E = 0.0;
double ITA_E = 0.0;
double KP_R = 0.0;
double TD_R = 0.0;
double TI_R = 0.0;
double ITA_R = 0.0;
double KP_ROLL = 0.0;
double TI_ROLL = 0.0;
double ITA_ROLL = 0.0;
double TD_ROLL = 0.0;


// MadgwickAHRS
/* sample frequency in Hz */
#define SAMPLE_FREQDEF   50
/*  proportional gain */
#define BETADEF         0.1
double phi;
double theta;
double psi;
double invSampleFreq = 1.0 / SAMPLE_FREQDEF;
double beta = BETADEF;        // algorithm gain
double q0 = 1.0;
double q1 = 0.0;
double q2 = 0.0;
double q3 = 0.0; // quaternion of sensor frame relative to auxiliary frame
double anglesComputed = 0.0;

/* Receiver Data */
int32_t Receiver_Data[8];
int Flag_EXTI = 0;
// 1517
int32_t aileronDuty;
int32_t aileronDutyMax  = 2104;
int32_t aileronDutyMin  = 930;
// 1607
int32_t elevatorDuty;
int32_t elevatorDutyMax = 2111;
int32_t elevatorDutyMin = 1104;
int32_t throttleDuty;
// 1524
int32_t rudderDuty;
int32_t rudderDutyMax   = 2111;
int32_t rudderDutyMin   = 937;
int32_t modeDuty;
int32_t vpp;

/* Servo Trim */
int32_t aileronTrim   = 0;
int32_t elevatorTrim  = 0;
int32_t rudderTrim    = 0;

/* Send Data Baffer */
char str[400];
/* Receive Data Baffer */
#define RX_BUFFER 8
int8_t RxData[RX_BUFFER + 1];

/* Exception Handler Counter */
int16_t I2Cexception = 0;
int16_t preI2Cexception = 0;
int16_t exceptionPlace = 0;
int32_t recoveryCount = 0;

bool stopFlag = TRUE;  // Stuck Point: TRUE<ENABLE> FALSE<DISABLE>

/* 1kHz Timer */
int32_t timeCounter = 0;
bool timeFlag = FALSE;

int32_t PIDTimer = 0;

/* arduino */
int16_t data = 0;

/* eight circle */
double relativeYaw = 0;
double yawOffset = 0;
double prePsi = 0.0;
int16_t sign = 0;
bool eightCircleFlag = FALSE;
int32_t circleCount = 0;

bool timerFlag = FALSE;

int32_t ControlStatus = 0;
double altitudeOffset = 0.0;

int32_t eightCircleMode = 0;
int32_t circlingUpMode = 0;

bool R6Flag = FALSE;

double gyroRollLPF = 0.0;
double angVel = 0.0;

int32_t modeView = 0;


/***********************************************************************
 * Setting
 **********************************************************************/
/* Flag Control */
bool CalibrationFlag = FALSE;
bool ControlFlag = FALSE;

/* Significant Figure */
#define SIGNFIG 1000
// Target Value
double phi_t    = 0.0;
double theta_t  = 0.0;
double psi_t    = 0.0;
double rollrate_t = 0.0;

int32_t ctrlThrottle = 0;

/* PID U LIMIT */
#define AILERON_MAX   40      // [Deg]
#define AILERON_MIN   -40     // [Deg]
#define ELEVATOR_MAX  40      // [Deg]
#define ELEVATOR_MIN  -40     // [Deg]
#define RUDDER_MAX    40      // [Deg]
#define RUDDER_MIN    -40     // [Deg]

/* SEND DATA*/
#define INDEX
#define MODE
#define CCNT
/*
#define ACCELE_X
#define ACCELE_Y
#define ACCELE_Z
*/
//#define GYRO_X
/*
#define GYRO_Y
#define GYRO_Z
#define MAG_X
#define MAG_Y
#define MAG_Z
*/

#define PHI_OP
#define THETA_OP
#define PSI_OP

#define PHI
#define THETA
#define PSI

//#define U_AILERON
//#define U_ELEVATOR
//#define U_RUDDER

//#define TEMP
//#define PRESS
#define ALTITUDE

#define REYAW

#define GYRO_X_PID

//#define ACCELE_OFFSET
//#define MAG_OFFSET
#define RECEIVER_DATA
#define RECEIVER_DATA_PID
/**
 *******************************************************************************************
 * MAIN Method
 *******************************************************************************************
 */
int main() {
  /* System Configuration */
  BoardInit();              // Microcomputer board Init
  GPIO_Configuration();     // APB2 | GPIO Config

  SysTick_Configuration();
  EXTI_Configuration();
  NVIC_Configuration();
  ADC_DMA_Configuration();

  /* Serial Configuration */
  I2C1_Configuration();     // APB1 | I2C1 Config
  I2C2_Configuration();     // APB1 | I2C2 Config
  USART1_Configuration();   // APB2 | USART1 Config
  sendData("USART1 Transmit Check\r\n");
  RxData[RX_BUFFER] = '\0';

  Sensor_ConectionCheck();

  /* Sensor Configuration */
  MPU6000_Configuration();  // I2C2 | MPU6000 Config
  MAG3110_Configuration();  // I2C2 | MAG3110 Config
  BMP280_Configuration();   // I2C2 | BMP280 Config
  /* Timer Configuration */
  TIM1_Configuration();     // APB2 | Servo PWM Output
  TIM2_Configuration();     // APB1 | Exception handling
  TIM4_Configuration();     // APB1 | Drop System
  TIM5_Configuration();     // APB1 | Sensor Data
  TIM6_Configuration();     // APB1 | PID Contloler
  TIM8_Configuration();     // APB2 | Recever Data Input

  while(1);
}

void ADC_DMA_Configuration(void){
  ADC_InitTypeDef  ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  ADC_DeInit(ADC1);

  /* ADC1 Configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_13Cycles5);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));

  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
 **************************************************************************************
 * Interrupt Function Handler & Processing
 *************************************************************************************/
// Exception handling
void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    if(I2Cexception == preI2Cexception){
      if(I2Cexception != 0){
        switch (exceptionPlace) {
          case 1:
            sendData("PLACE : MPU6000\r\n");
            break;

          case 2:
            sendData("PLACE : MAG3110\r\n");
            break;

          case 3:
            sendData("PLACE : BMP280\r\n");
            break;

          default:
            sendData("PLACE : ?\r\n");
            break;
        }
      }
      switch (I2Cexception) {
        case 1:
          sendData("ERROR : [1:I2C_FLAG_BUSY]\r\n");
          break;
        case 2:
          sendData("ERROR : [2:I2C_EVENT_MASTER_MODE_SELECT]\r\n");
          break;
        case 3:
          sendData("ERROR : [3:I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED]\r\n");
          break;
        case 4:
          sendData("ERROR : [4:I2C_EVENT_MASTER_BYTE_TRANSMITTED]\r\n");
          break;
        case 5:
          sendData("ERROR : [5:I2C_EVENT_MASTER_MODE_SELECT]\r\n");
          break;
        case 6:
          sendData("ERROR : [6:I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED]\r\n");
          break;
        case 7:
          sendData("ERROR : [7:I2C_EVENT_MASTER_BYTE_RECEIVED]\r\n");
          break;
        case 8:
          sendData("ERROR : [8:I2C_EVENT_MASTER_BYTE_RECEIVED]\r\n");
          break;

        default:
          //sendData("debug\r\n");
          break;
      }

      if(stopFlag == TRUE && I2Cexception != 0){
        // Stuck Point
        while(1);
      }
    }
    preI2Cexception = I2Cexception;
  }
}

// Sensor Data Sensing and Calibration
void TIM5_IRQHandler(void) {
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

    if(CalibrationFlag == TRUE) {
      MAG3110_Calibration();
    } else {
      count++;
      MPU6000_getParameter();
      MAG3110_getParameter();
      BMP280_altitude();
      MadgwickAHRS(gyroData[0],gyroData[1],gyroData[2],accelData[0],accelData[1],accelData[2],magData[0],magData[1],magData[2]);
    }
    Toggle_LED();
  }
}

void TIM6_IRQHandler(void) {
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

    if(CalibrationFlag != TRUE) {
      if(ControlFlag == TRUE) {
        if(preControlMode != ControlMode){
          PIDTimer = 0; // Time Counter Reset
          tsprintf(str, "MODE:%d\r\n", ControlMode);
          sendData(str);
          tsprintf(str, "GAIN:%d,%d,%d,%d|%d,%d,%d,%d|%d,%d,%d,%d\r\n\r\n",
              (int32_t)(KP_A*100),(int32_t)(TD_A*100),(int32_t)(TI_A*100),(int32_t)(ITA_A*100),
              (int32_t)(KP_E*100),(int32_t)(TD_E*100),(int32_t)(TI_E*100),(int32_t)(ITA_E*100),
              (int32_t)(KP_R*100),(int32_t)(TD_R*100),(int32_t)(TI_R*100),(int32_t)(ITA_R*100));
          sendData(str);
        }
        preControlMode = ControlMode;

        // Relative Angle
        // Direction of rotation
        if(psi > prePsi) sign = 1;
        else if(psi < prePsi) sign = -1;
        else sign = 0;

        if(fabs(psi - prePsi) > 300.0) {
          relativeYaw += (fabs(psi - prePsi) - 360.0) * sign;
        } else {
          relativeYaw += fabs(psi - prePsi) * sign;
        }
        prePsi = psi;

        switch (ControlMode) {
          // Manual Mode
          case 0:
          break;
          // Horizontal Steady Flight
          case 1:{
            // Target Value
            phi_t    = 0.0;
            theta_t  = 20.0;
            psi_t    = 0.0;
            ctrlThrottle = 1470;

            aileronTrim   = 0;
            elevatorTrim  = 130;
            rudderTrim    = 0;

            //Aileron
            KP_A = 0.5;
            TD_A = 0.01;
            TI_A = 10;
            ITA_A = 0.1;
            // Elevator
            KP_E = 0.5;
            TD_E = 0.01;
            TI_E = 10;
            ITA_E = 0.1;
            // Rudder
            KP_R = 0.5;
            TD_R = 0.01;
            TI_R = 10;
            ITA_R = 0.1;
            break;
          }

          // Horizontal Turning Flight[Right]
          case 2:{
            // right
            // Target Value
            phi_t    = 30.0;
            theta_t  = 15.0;
            psi_t    = 0.0;
            ctrlThrottle = 1385;

            aileronTrim   = 100;
            elevatorTrim  = -320;
            rudderTrim    = 0;

            //Aileron
            KP_A = 1.5;
            TD_A = 0.1;
            TI_A = 10;
            ITA_A = 0.1;
            // Elevator
            KP_E = 0.5;
            TD_E = 0.2;
            TI_E = 10;
            ITA_E = 0.1;
            // Rudder
            KP_R = 0.0000001;
            TD_R = 0.01;
            TI_R = 10;
            ITA_R = 0.1;
            break;
          }

          // Horizontal Turning Flight[Left]
          case 3:{
            // Target Value
            phi_t    = -30.0;
            theta_t  = 15.0;
            psi_t    = 0.0;
            ctrlThrottle = 1385;

            aileronTrim   = 0; // 1517
            elevatorTrim  = -320; // 1607
            rudderTrim    = 0;   // 1524

            //Aileron
            KP_A = 1.5;
            TD_A = 0.1;
            TI_A = 10;
            ITA_A = 0.1;
            // Elevator
            KP_E = 0.5;
            TD_E = 0.2;
            TI_E = 10;
            ITA_E = 0.1;
            // Rudder
            KP_R = 0.0000001;
            TD_R = 0.01;
            TI_R = 10;
            ITA_R = 0.1;
            break;
          }

          // Circling Up Flight
          case 4:{
            if(circlingUpMode == 1 && relativeYaw < -720) {
              circlingUpMode = 2;
              prePsi = psi;
              relativeYaw = 0.0;
              altitudeOffset = altitude;
            }
            if(altitudeOffset + 3000 < altitude) circlingUpMode = 3;

            if(circlingUpMode == 1) ctrlThrottle = 1470;
            else if(circlingUpMode == 2) ctrlThrottle = 1560;
            else if(circlingUpMode == 3) ctrlThrottle = 1470;

            modeView = circlingUpMode;

            // Horizontal Turning Flight[Right]
            // Target Value
            phi_t    = 47.0;
            theta_t  = 20.0;
            psi_t    = 0.0;

            aileronTrim   = -167;
            elevatorTrim  = 270;
            rudderTrim    = 0;

            //Aileron
            KP_A = 0.75;
            TD_A = 0.1;
            TI_A = 10;
            ITA_A = 0.1;
            // Elevator
            KP_E = 0.3;
            TD_E = 0.01;
            TI_E = 10;
            ITA_E = 0.1;
            // Rudder
            KP_R = 0.0000001;
            TD_R = 0.01;
            TI_R = 10;
            ITA_R = 0.1;
            break;
          }

          // Circling Down Flight
          case 5:{
            // Target Value
            phi_t    = 35.0;
            theta_t  = 25.0;
            psi_t    = 0.0;
            ctrlThrottle = 1500;

            //Aileron
            KP_A = 0.5;
            TD_A = 0.01;
            TI_A = 10;
            ITA_A = 0.1;
            // Elevator
            KP_E = 0.5;
            TD_E = 0.01;
            TI_E = 10;
            ITA_E = 0.1;
            // Rudder
            KP_R = 0.5;
            TD_R = 0.01;
            TI_R = 10;
            ITA_R = 0.1;
            break;
          }

          // Figure Eight Flight
          case 6:{
            // Switch Horizontal Turning Flight[Left]
            // Eight circle discrimination
            if (eightCircleMode == 1 && relativeYaw > 360.0){
              // Switching Figure Eight Flight
              // Start Figure Eight Flight switching
              eightCircleMode = 2;
              relativeYaw = 0.0;

            } else if(eightCircleMode == 2 && (phi < 25.0 && phi > 15.0)){
              // Switch Horizontal Turning Flight[Right]
              eightCircleMode = 3;
            }

            switch (eightCircleMode){
              case 0:{

                break;
              }

              case 1:{
                // Horizontal Turning Flight[Left]
                // Target Value
                phi_t    = -40.0;
                theta_t  = 20.0;
                psi_t    = 0.0;
                ctrlThrottle = 1470;

                aileronTrim   = -100; // 1517
                elevatorTrim  = 250; // 1607
                rudderTrim    = 0;   // 1524

                //Aileron
                KP_A = 2.5;
                TD_A = 0.1;
                TI_A = 10;
                ITA_A = 0.1;
                // Elevator
                KP_E = 1.5;
                TD_E = 0.01;
                TI_E = 10;
                ITA_E = 0.1;
                // Rudder
                KP_R = 0.0000001;
                TD_R = 0.01;
                TI_R = 10;
                ITA_R = 0.1;
                break;
              }

              case 2:{
                // RollRate Control
                // Target Value
                theta_t  = 20.0;
                rollrate_t = 33.8;
                ctrlThrottle = 1470;

                aileronTrim   = 0;
                elevatorTrim  = 130;
                rudderTrim    = 0;

                //Aileron
                KP_A = 1.0;
                TD_ROLL = 0.01;
                TI_A = 10;
                ITA_ROLL = 0.1;
                // Elevator
                KP_E = 0.3;
                TD_E = 0.01;
                TI_E = 10;
                ITA_E = 0.1;
                // Rudder
                KP_R = 0.0000001;
                TD_R = 0.01;
                TI_R = 10;
                ITA_R = 0.1;

                /*
                // Target Value
                phi_t    = 80.0;
                theta_t  = 20.0;
                psi_t    = 0.0;
                ctrlThrottle = 1470;

                aileronTrim   = 0;
                elevatorTrim  = 130;
                rudderTrim    = -600;

                //Aileron
                KP_A = 2.0;
                TD_A = 0.1;
                TI_A = 10;
                ITA_A = 0.5;
                // Elevator
                KP_E = 1.5;
                TD_E = 0.01;
                TI_E = 10;
                ITA_E = 0.1;
                // Rudder
                KP_R = 0.0000001;
                TD_R = 0.01;
                TI_R = 10;
                ITA_R = 0.1;
                */
                break;
              }

              case 3:{
                // Horizontal Turning Flight[Right]
                // Target Value
                phi_t    = 47.0;
                theta_t  = 20.0;
                psi_t    = 0.0;
                ctrlThrottle = 1470;

                aileronTrim   = -167;
                elevatorTrim  = 270;
                rudderTrim    = 0;

                //Aileron
                KP_A = 0.75;
                TD_A = 0.1;
                TI_A = 10;
                ITA_A = 0.1;
                // Elevator
                KP_E = 0.3;
                TD_E = 0.01;
                TI_E = 10;
                ITA_E = 0.1;
                // Rudder
                KP_R = 0.0000001;
                TD_R = 0.01;
                TI_R = 10;
                ITA_R = 0.1;
                break;
              }
              default:
                break;
            }
            break;
          }

          // Automatic Takeoff and Landing
          case 7:{
            PIDTimer++;
            if((PIDTimer*0.05) <= 3.0){
              // Target Value
              phi_t    = 0.0;
              theta_t  = 18.0;
              psi_t    = 0.0;
              ctrlThrottle = 1500;

              //Aileron
              KP_A = 1;
              TD_A = 0.01;
              TI_A = 10;
              ITA_A = 0.1;
              // Elevator
              KP_E = 0.5;
              TD_E = 0.01;
              TI_E = 10;
              ITA_E = 0.1;
              // Rudder
              KP_R = 0.5;
              TD_R = 0.01;
              TI_R = 10;
              ITA_R = 0.1;
            } else if ((PIDTimer*0.05) <= 8.0) {
              // Target Value
              phi_t    = 0.0;
              theta_t  = 15.0;
              psi_t    = 0.0;
              ctrlThrottle = 1450;

              //Aileron
              KP_A = 1;
              TD_A = 0.01;
              TI_A = 10;
              ITA_A = 0.1;
              // Elevator
              KP_E = 0.5;
              TD_E = 0.01;
              TI_E = 10;
              ITA_E = 0.1;
              // Rudder
              KP_R = 0.5;
              TD_R = 0.01;
              TI_R = 10;
              ITA_R = 0.1;
            } else if ((PIDTimer*0.05) > 8.0) {
              // Target Value
              phi_t    = 0.0;
              theta_t  = 6.0;
              psi_t    = 0.0;
              ctrlThrottle = 1200;

              //Aileron
              KP_A = 1;
              TD_A = 0.01;
              TI_A = 10;
              ITA_A = 0.1;
              // Elevator
              KP_E = 0.5;
              TD_E = 0.01;
              TI_E = 10;
              ITA_E = 0.1;
              // Rudder
              KP_R = 0.5;
              TD_R = 0.01;
              TI_R = 10;
              ITA_R = 0.1;
            }

            break;
          }

          // Drop Mode
          case 8:{

          }

          default:
            break;
        }

        if (eightCircleMode == 2){
          PID_Roll_Controller();
          //PI_D_Controller();

        } else {
          PID_Controller();
        }
        //PI_D_Controller();
        if(ControlMode == 8){
          sendDataOption();
        } else {
          sendDataPID();
        }

      } else {
        PIDTimer = 0; // Time Counter Reset
        ControlMode = 0;
        eightCircleMode = 0;
        circlingUpMode = 0;
        modeView = 0;
        eightCircleFlag = FALSE;
        sendDataOption();
        //tsprintf(str, "V:%d\r\n", ADCConvertedValue);
        //sendData(str);
        //Arduino_getParameter();
      }
    }
  }
}

// Reciver Data Output - Servo PWM -
void TIM8_UP_IRQHandler(void) {
  if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM8, TIM_IT_Update);

    Convert_ReceiverData_to_PWM();

    if((Receiver_Data[5]/72) > 1735) {
      ControlFlag = TRUE;
      if(eightCircleFlag == FALSE) {
        prePsi = psi;
        relativeYaw = 0.0;
        eightCircleFlag = TRUE;
        eightCircleMode = 1;
        circlingUpMode = 1;
      }

      switch (ControlCounter){
        case 0:{
          ControlMode = 8;
          break;
        }

        case 1:{
          ControlMode = 3;
          break;
        }

        case 2:{
          ControlMode = 6;
          break;
        }

        case 3:{
          ControlMode = 4;
          break;
        }

        case 4:{
          ControlMode = 2;
          break;
        }

        default:
          break;
      }

      if(ControlMode == 8){
        TIM_SetCompare1(TIM1, aileronDuty);
        TIM_SetCompare2(TIM1, elevatorDuty);
        TIM_SetCompare3(TIM1, throttleDuty);
        TIM_SetCompare4(TIM1, rudderDuty);
        TIM_SetCompare3(TIM4, aileronDuty);    //Open
        //TIM_SetCompare2(TIM4, 1130);    //Open
        //TIM_SetCompare3(TIM4, 1130);    //Open
        //TIM_SetCompare4(TIM4, 1130);    //Open
      } else {
        TIM_SetCompare1(TIM1, Aileron_u + aileronTrim);
        TIM_SetCompare2(TIM1, Elevator_u + elevatorTrim);
        TIM_SetCompare3(TIM1, ctrlThrottle);
        TIM_SetCompare4(TIM1, rudderDuty + rudderTrim);
      }
    } else {
      ControlFlag = FALSE;
      eightCircleFlag = FALSE;
      ControlMode = 0;
      TIM_SetCompare1(TIM1, aileronDuty);
      TIM_SetCompare2(TIM1, elevatorDuty);
      TIM_SetCompare3(TIM1, throttleDuty);
      TIM_SetCompare4(TIM1, rudderDuty);
      TIM_SetCompare3(TIM4, aileronDuty);      //Close
      /*
      TIM_SetCompare2(TIM4, 1770);      //Close
      TIM_SetCompare3(TIM4, 1770);      //Close
      TIM_SetCompare4(TIM4, 1770);      //Close
      */
    }

    if((Receiver_Data[6]/72) < 1300) {
      R6Flag = TRUE;
    }

    if((Receiver_Data[6]/72) > 1735 && R6Flag == TRUE) {
      ControlCounter++;
      R6Flag = FALSE;
      if(ControlCounter > 4) ControlCounter = 0;
    }
  }
}

void EXTI0_IRQHandler(void) {
  EXTI_ClearITPendingBit( EXTI_Line0 );

  if( GPIO_ReadInputDataBit( GPIOE , GPIO_Pin_0 ) ){
    SysTick->VAL = 0xFFFFFF;
    Flag_EXTI = 1;
  }
  else{
    if(Flag_EXTI){
      Receiver_Data[0] = SysTick->VAL;
      Receiver_Data[1] = 0xFFFFFF - Receiver_Data[0];
    }
  }
}

void EXTI1_IRQHandler(void) {
  EXTI_ClearITPendingBit(EXTI_Line1);

  if(Flag_EXTI){
    Receiver_Data[7] = SysTick->VAL;
    Receiver_Data[2] = Receiver_Data[0] - Receiver_Data[7];
    Receiver_Data[0] = Receiver_Data[7];
  }
}

void EXTI2_IRQHandler(void) {
  EXTI_ClearITPendingBit(EXTI_Line2);

  if(Flag_EXTI){
    Receiver_Data[7] = SysTick->VAL;
    Receiver_Data[3] = Receiver_Data[0] - Receiver_Data[7];
    Receiver_Data[0] = Receiver_Data[7];
  }
}

void EXTI3_IRQHandler(void) {
  EXTI_ClearITPendingBit(EXTI_Line3);

  if(Flag_EXTI){
    Receiver_Data[7] = SysTick->VAL;
    Receiver_Data[4] = Receiver_Data[0] - Receiver_Data[7];
    Receiver_Data[0] = Receiver_Data[7];
  }
}

void EXTI4_IRQHandler(void) {
  EXTI_ClearITPendingBit(EXTI_Line4);

  if(Flag_EXTI)
  {
    Receiver_Data[7] = SysTick->VAL;
    Receiver_Data[5] = Receiver_Data[0] - Receiver_Data[7];
    Receiver_Data[0] = Receiver_Data[7];
  }
}

void EXTI9_5_IRQHandler(void) {
  if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
    EXTI_ClearITPendingBit(EXTI_Line5);

    if(Flag_EXTI) {
      Receiver_Data[6] = Receiver_Data[0] - SysTick->VAL;
      Flag_EXTI = 0;
    }
  }
}

void DMA1_Channel5_IRQHandler(void) {
  /* Test on DMA1 Channel5 Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA1_IT_TC5)) {
    tsprintf(str, "%d\r\n", RxData);
    sendData(str);
    while(1);

    /* Clear DMA1 Channel5 Transfer Complete interrupt pending bits */
    DMA_ClearITPendingBit(DMA1_IT_TC5);
  }
}

void Convert_ReceiverData_to_PWM(void) {
  aileronDuty   = Receiver_Data[1] / 72;
  elevatorDuty  = Receiver_Data[2] / 72;
  throttleDuty  = Receiver_Data[3] / 72;
  rudderDuty    = Receiver_Data[4] / 72;
  modeDuty      = Receiver_Data[5] / 72;
  vpp           = Receiver_Data[6] / 72;
}

/*
 **************************************************************************************
 * Sensor Configuration
 **************************************************************************************/

bool Sensor_ConectionCheck(void) {
  int16_t chipId = 0xFF;

  chipId = i2c2Read(MPU6000_ADDRESS, MPU6000_WHO_AM_I);
  if(chipId == MPU6000_CHIPID) {
    tsprintf(str, "MPU6000 Connection Success [0x%02x]\r\n", chipId);
    sendData(str);
  }
  else {
    sendData("MPU6000 Connection Failed\r\n");
    return FALSE;
  }

  chipId = i2c2Read(MAG3110_ADDRESS, MAG3110_WHO_AM_I);
  if(chipId == MAG3110_CHIPID) {
    tsprintf(str, "MAG3110 Connection Success [0x%02x]\r\n", chipId);
    sendData(str);
  }
  else {
    sendData("MAG3110 Connection Failed\r\n");
    return FALSE;
  }

  chipId = i2c2Read(BMP280_ADDRESS, BMP280_WHO_AM_I);
  if(chipId == BMP280_CHIPID) {
    tsprintf(str, "BMP280 Connection Success [0x%02x]\r\n", chipId);
    sendData(str);
  }
  else {
    sendData("BMP280 Connection Failed\r\n");
    return FALSE;
  }

  return TRUE;
}

void MPU6000_Configuration(void) {
  // POWER Management [0x6B] <0b00000000>
  i2c2Write(MPU6000_ADDRESS, MPU6000_PWR_MGMT_1, 0x00);
  // CONFIG [0x1A] <0b00000110> 5Hz
  //i2c2Write(MPU6000_ADDRESS, MPU6000_CONFIG, 0x06);
  // GYRO_CONFIG [0x1B] <0b00001000> FSR:500deg/s
  i2c2Write(MPU6000_ADDRESS, MPU6000_GYRO_CONFIG, 0x08);
  // ACCEL_CONFIG [0x1C] <0b11100000> FSR:2G
  i2c2Write(MPU6000_ADDRESS, MPU6000_ACCEL_CONFIG, 0x00);
  // I2C_MST_CTRL [0x1D] <0b00011101>
  //i2c2Write(MPU6000_ADDRESS, MPU6000_I2C_MST_CTRL, 0x1D);
  // INT_PIN_CFG [0x37] <0b00000010> I2C BYPATH ENABLE
  //i2c2Write(MPU6000_ADDRESS, MPU6000_INT_PIN_CONFIG, 0x02);
  // SIGNAL_PATH_RESET [0x68] <0b00000111>
  //i2c2Write(MPU6000_ADDRESS, MPU6000_SIGNAL_PATH_RESET, 0x07);
}

void MPU6000_getParameter(void) {
  // MPU6000 Sensor Data Burst Transmission
  I2Cexception = 0;
  exceptionPlace = 1;
  // I2C2 BUSY
  I2Cexception++;//1
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
    // Generate Start Condition
  I2C_AcknowledgeConfig(I2C2, ENABLE);
  I2C_GenerateSTART(I2C2, ENABLE);
  // Master Mode & I2C BUSY
  I2Cexception++;//2
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    // Send Address + Direction Bit(0)[Write]
  I2C_Send7bitAddress(I2C2, MPU6000_ADDRESS, I2C_Direction_Transmitter);
  // Transmitter Mode & DataRegister Empty & I2C BUSY
  I2Cexception++;//3
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    // Send Register Address
  I2C_SendData(I2C2, MPU6000_ACCEL_XOUT_H);
  // Data Transmitted & I2C BUSY
  I2Cexception++;//4
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  // Generate Start Condition
  I2C_GenerateSTART(I2C2, ENABLE);
  // Master Mode & I2C BUSY
  I2Cexception++;//5
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
  // Send Address + Direction Bit(1)[Read]
  I2C_Send7bitAddress(I2C2, MPU6000_ADDRESS, I2C_Direction_Receiver);
  // Receiver Mode & I2C BUSY
  I2Cexception++;//6
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  // Ack Enable
  I2C_AcknowledgeConfig(I2C2, ENABLE);
  I2Cexception++;//7
  for(uint8_t i = 0; i < 14-1; i++)
  {
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    binary_oneByteData[i] = I2C_ReceiveData(I2C2);
  }
  I2C_AcknowledgeConfig(I2C2, DISABLE);
  I2C_GenerateSTOP(I2C2, ENABLE);
  I2Cexception++;//8
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)){
    recoveryCount++;
    if(recoveryCount > 9){
      recoveryCount = 0;
      break;
    }
  }
  binary_oneByteData[13] = I2C_ReceiveData(I2C2);
  I2Cexception = 0;
  exceptionPlace = 0;

  for(uint8_t i = 0; i < 7; i++){
    mpu6000_rawData[i] = (binary_oneByteData[i*2]<<8) | binary_oneByteData[i*2+1];
  }

  for(uint8_t i = 0; i < 3; i++){
    if(mpu6000_rawData[i] > 32767){
      accelData[i] = -1 * GRAVITY_ACCLE * (65535.0-(double)(mpu6000_rawData[i] - accelOffset[i])) / ACCEL_2G_LSB;
    } else {
      accelData[i] = GRAVITY_ACCLE * (double)(mpu6000_rawData[i] - accelOffset[i]) / ACCEL_2G_LSB;
    }
  }

  if(mpu6000_rawData[3] > 32767){
    mpuTemp = -1 * (65535.0 - (double)mpu6000_rawData[3]) / 340.0 + 36.53;
  } else {
    mpuTemp = (double)mpu6000_rawData[3] / 340.0 + 36.53;
  }

  for(uint8_t i = 0; i < 3; i++){
    if(mpu6000_rawData[i+4] > 32767){
      gyroData[i] = -1 * (65535.0 - (double)mpu6000_rawData[i+4]) / GYRO_500DEG_LSB;
    } else {
      gyroData[i] = (double)mpu6000_rawData[i+4] / GYRO_500DEG_LSB;
    }
  }
}

void MAG3110_Standby(void) {
  int8_t status = i2c2Read(MAG3110_ADDRESS, MAG3110_CTRL_REG1);
  _delay_ms(3);
  i2c2Write(MAG3110_ADDRESS, MAG3110_CTRL_REG1, status &= ~(0x03));
  _delay_ms(3);
}

void MAG3110_Active(void) {
  int8_t status = i2c2Read(MAG3110_ADDRESS, MAG3110_CTRL_REG1);
  _delay_ms(3);
  i2c2Write(MAG3110_ADDRESS, MAG3110_CTRL_REG1, status | 0x01);
}

void MAG3110_Reset(void) {
  MAG3110_Standby();
  i2c2Write(MAG3110_ADDRESS, MAG3110_CTRL_REG1, 0x00);
  _delay_ms(3);
  i2c2Write(MAG3110_ADDRESS, MAG3110_CTRL_REG2, 0x80);
  _delay_ms(3);
}

void MAG3110_Configuration(void) {
  MAG3110_Reset();
  MAG3110_Active();
}

bool MAG3110_dataReady(void) {
  // ZYXDR : 1 Data Ready
  int8_t status = i2c2Read(MAG3110_ADDRESS, MAG3110_DR_STATUS);
  _delay_ms(3);

  return (status&0x08)>>3;
}

void MAG3110_getParameter(void) {
  I2Cexception = 0;
  exceptionPlace = 2;
  // I2C2 BUSY
  I2Cexception++;//1
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
  // Generate Start Condition
  I2C_AcknowledgeConfig(I2C2, ENABLE);
  I2C_GenerateSTART(I2C2, ENABLE);
  // Master Mode & I2C BUSY
  I2Cexception++;//2
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
  // Send Address + Direction Bit(0)[Write]
  I2C_Send7bitAddress(I2C2, MAG3110_ADDRESS, I2C_Direction_Transmitter);
  // Transmitter Mode & DataRegister Empty & I2C BUSY
  I2Cexception++;//3
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  // Send Register Address
  I2C_SendData(I2C2, MAG3110_OUT_X_MSB);
  // Data Transmitted & I2C BUSY
  I2Cexception++;//4
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  // Generate Start Condition
  I2C_GenerateSTART(I2C2, ENABLE);
  // Master Mode & I2C BUSY
  I2Cexception++;//5
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
  // Send Address + Direction Bit(1)[Read]
  I2C_Send7bitAddress(I2C2, MAG3110_ADDRESS, I2C_Direction_Receiver);
  // Receiver Mode & I2C BUSY
  I2Cexception++;//6
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  // Ack Enable
  I2C_AcknowledgeConfig(I2C2, ENABLE);
  I2Cexception++;//7
  for(uint8_t i = 0; i < 6-1; i++) {
      while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
      binary_oneByteData[i] = I2C_ReceiveData(I2C2);
  }
  I2C_AcknowledgeConfig(I2C2, DISABLE);
  I2C_GenerateSTOP(I2C2, ENABLE);
  I2Cexception++;//8
  // Popular Stuck Point !
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)){
    recoveryCount++;
    if(recoveryCount > 9){
      recoveryCount = 0;
      break;
    }
  }
  binary_oneByteData[5] = I2C_ReceiveData(I2C2);

  I2Cexception = 0;
  exceptionPlace = 0;

  for(uint8_t i = 0; i < 3; i++){
      mag3110_rawData[i] = ((binary_oneByteData[i*2]<<8) | binary_oneByteData[i*2+1]);
  }

  for(uint8_t i = 0; i < 3; i++){
    if(mag3110_rawData[i] > 32767){
      magData[i] = -1000*(65535.0-(double)(mag3110_rawData[i]-magOffset[i]))/MAG_LSB;
    } else {
      magData[i] = 1000*(double)(mag3110_rawData[i]-magOffset[i])/MAG_LSB;
    }
  }
}

void MAG3110_Calibration(void) {
  if(ccnt < 500){
    MAG3110_getParameter();
    for(uint8_t i = 0; i < 3; i++) {
      if(mag3110_rawData[i] > 32767) {
        magCalib[i] = -1 * (65535 - mag3110_rawData[i]);
      } else {
        magCalib[i] = mag3110_rawData[i];
      }
      if(magMax[i] < magCalib[i]) {
        magMax[i] = magCalib[i];
      }
      if(magMin[i] > magCalib[i]) {
        magMin[i] = magCalib[i];
      }
    }
    sendDataCalibration();
  } else {
    for(uint8_t i = 0; i < 3; i++) {
      magOffset[i] = (magMax[i] + magMin[i])/2;
    }
    sendDataOffset();
    CalibrationFlag = FALSE;
    while(1); // Stack Point
  }
  ccnt++;
}

void BMP280_Configuration(void){
  _delay_ms(3);
  // Power on reset
  i2c2Write(BMP280_ADDRESS, BMP280_RESET, BMP280_POWER_ON_RESET );
  _delay_ms(3);
  //Get correction factor
  BMP280_getTrimParameter();
  // Filter Setting
  i2c2Write(BMP280_ADDRESS, BMP280_CTRL_MEAS, 0x54);
  // To 0xF5, Write 0x10 <0b 000 100 00> StandbyTime = 0.5ms, Filter coefficient = 16, SPI: Disable
  i2c2Write(BMP280_ADDRESS, BMP280_CONFIG, 0x10);
  // To 0xF4, Write 0x57 <0b 010 101 11> NomalMode, OverSampling: Temp*2,Pres*16 (Indoor navigation)
  i2c2Write(BMP280_ADDRESS, BMP280_CTRL_MEAS, 0x57);
  _delay_ms(50);

  int32_t sumP = 0;
  for(uint16_t i = 0; i < 100; i++){
    BMP280_pressure();
    sumP += (int32_t)pressure;
  }
  P0 = sumP / 100;
}

void BMP280_getTrimParameter(void){
  // BMP280 Sensor Data Burst Transmission
  // BMP280 Trimming parameter readout
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));                               // I2C2 BUSY
  I2C_GenerateSTART(I2C2, ENABLE);                                              // Generate Start Condition
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C2, BMP280_ADDRESS, I2C_Direction_Transmitter);        // Send Address + Direction Bit(0)[Write]
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    // Transmitter Mode & DataRegister Empty & I2C BUSY
  I2C_SendData(I2C2, BMP280_DIG_T1);                                        // Send Register Address
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));             // Data Transmitted & I2C BUSY
  I2C_GenerateSTART(I2C2, ENABLE);                                              // Generate Start Condition
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C2, BMP280_ADDRESS, I2C_Direction_Receiver);           // Send Address + Direction Bit(1)[Read]
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));       // Receiver Mode & I2C BUSY
  I2C_AcknowledgeConfig(I2C2, ENABLE);                                          // Ack Enable
  for(uint8_t i = 0; i < 24-1; i++)
  {
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    binary_oneByteData[i] = I2C_ReceiveData(I2C2);
  }
  I2C_AcknowledgeConfig(I2C2, DISABLE);
  I2C_GenerateSTOP(I2C2, ENABLE);

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
  binary_oneByteData[23] = I2C_ReceiveData(I2C2);

  bmp280_digData[0] = (uint16_t)(((uint16_t)binary_oneByteData[1]<<8)|((uint16_t)binary_oneByteData[0]));
  bmp280_digData[1] = (int16_t)(((int16_t)binary_oneByteData[3]<<8)|((int16_t)binary_oneByteData[2]));
  bmp280_digData[2] = (int16_t)(((int16_t)binary_oneByteData[5]<<8)|((int16_t)binary_oneByteData[4]));
  bmp280_digData[3] = (uint16_t)(((uint16_t)binary_oneByteData[7]<<8)|((uint16_t)binary_oneByteData[6]));
  bmp280_digData[4] = (int16_t)(((int16_t)binary_oneByteData[9]<<8)|((int16_t)binary_oneByteData[8]));
  bmp280_digData[5] = (int16_t)(((int16_t)binary_oneByteData[11]<<8)|((int16_t)binary_oneByteData[10]));
  bmp280_digData[6] = (int16_t)(((int16_t)binary_oneByteData[13]<<8)|((int16_t)binary_oneByteData[12]));
  bmp280_digData[7] = (int16_t)(((int16_t)binary_oneByteData[15]<<8)|((int16_t)binary_oneByteData[14]));
  bmp280_digData[8] = (int16_t)(((int16_t)binary_oneByteData[17]<<8)|((int16_t)binary_oneByteData[16]));
  bmp280_digData[9] = (int16_t)(((int16_t)binary_oneByteData[19]<<8)|((int16_t)binary_oneByteData[18]));
  bmp280_digData[10] = (int16_t)(((int16_t)binary_oneByteData[21]<<8)|((int16_t)binary_oneByteData[20]));
  bmp280_digData[11] = (int16_t)(((int16_t)binary_oneByteData[23]<<8)|((int16_t)binary_oneByteData[22]));

}

void BMP280_getParameter(void){
  I2Cexception = 0;
  exceptionPlace = 3;
  // BMP280 Sensor Data Burst Transmission
  // BMP280 Pressure & Temperature
  I2Cexception++;//1
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));                               // I2C2 BUSY
  I2C_GenerateSTART(I2C2, ENABLE);
  I2Cexception++;//2                                           // Generate Start Condition
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C2, BMP280_ADDRESS, I2C_Direction_Transmitter);        // Send Address + Direction Bit(0)[Write]
  I2Cexception++;//3
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    // Transmitter Mode & DataRegister Empty & I2C BUSY
  I2C_SendData(I2C2, BMP280_PRESS);                                        // Send Register Address
  I2Cexception++;//4
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));             // Data Transmitted & I2C BUSY
  I2C_GenerateSTART(I2C2, ENABLE);                                              // Generate Start Condition
  I2Cexception++;//5
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C2, BMP280_ADDRESS, I2C_Direction_Receiver);           // Send Address + Direction Bit(1)[Read]
  I2Cexception++;//6
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));       // Receiver Mode & I2C BUSY
  I2C_AcknowledgeConfig(I2C2, ENABLE);                                          // Ack Enable
  I2Cexception++;//7
  for(uint8_t i = 0; i < 6-1; i++)
  {
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    binary_oneByteData[i] = I2C_ReceiveData(I2C2);
  }
  I2C_AcknowledgeConfig(I2C2, DISABLE);
  I2C_GenerateSTOP(I2C2, ENABLE);
  I2Cexception++;//8
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)){
    recoveryCount++;
    if(recoveryCount > 9){
      recoveryCount = 0;
      break;
    }
  }
  binary_oneByteData[5] = I2C_ReceiveData(I2C2);

  I2Cexception = 0;
  exceptionPlace = 0;

  Pre_press_correctionfactor = (int32_t)((((uint32_t)(binary_oneByteData[0]))<<12) | (((uint32_t)(binary_oneByteData[1]))<<4) | ((uint32_t)binary_oneByteData[2]>>4));
  Pre_temp_correctionfactor = (int32_t)((((uint32_t)(binary_oneByteData[3]))<<12) | (((uint32_t)(binary_oneByteData[4]))<<4) | (((uint32_t)(binary_oneByteData[5]))>>4));

}

void BMP280_temperature(void)
{
  BMP280_getParameter();
  int32_t var1, var2;
  var1 = ((((Pre_temp_correctionfactor>>3) - ((int32_t)bmp280_digData[0]<<1))) * ((int32_t)bmp280_digData[1]))>>11;             // New correction factor
  var2 = (((((Pre_temp_correctionfactor>>4) - ((int32_t)bmp280_digData[0])) * ((Pre_temp_correctionfactor>>4) - ((int32_t)bmp280_digData[0])))>>12) * ((int32_t)bmp280_digData[2]))>>14;
  t_fine = var1 + var2;
  temperature = (t_fine * 5 + 128) >> 8;
}

void BMP280_pressure(void)
{
  BMP280_temperature();
  int32_t var1, var2;
  var1 = (((int32_t)t_fine)>>1)-(int32_t)64000;
  var2 = (((var1>>2) * (var1>>2))>>11) * ((int32_t)bmp280_digData[8]);
  var2 = var2+((var1*((int32_t)bmp280_digData[7]))<<1);
  var2 = (var2>>2)+(((int32_t)bmp280_digData[6])<<16);
  var1 = (((bmp280_digData[5]*(((var1>>2) * (var1>>2))>>13))>>3)+((((int32_t)bmp280_digData[4])*var1)>>1))>>18;
  var1 = ((((32768+var1))*((int32_t)bmp280_digData[3]))>>15);
  pressure = (((uint32_t)(((int32_t)1048576)-Pre_press_correctionfactor)-(var2>>12)))*3125;
  if(var1!=0)
    {
      if(pressure < 0x80000000)
        pressure = (pressure << 1)/((uint32_t)var1);
      else
        pressure = (pressure/(uint32_t)var1) * 2;
        var1 = (((int32_t)bmp280_digData[11])*((int32_t)(((pressure>>3)*(pressure>>3))>>13)))>>12;
        var2 = (((int32_t)(pressure>>2))*((int32_t)bmp280_digData[10]))>>13;
        pressure = (uint32_t)((int32_t)pressure+((var1+var2+bmp280_digData[9])>>4));
    }
  else
    pressure = 0;

}

void BMP280_altitude(void){
  BMP280_pressure();
  press_ = pow(((double)P0 / (double)((int32_t)pressure)),(1.0/5.257));
  altitude = ((press_ - 1.0) * (T + 273.15) / 0.0065);
}

void Arduino_getParameter(void){
  sendData("debug\r\n");
  data = (int16_t)arduinoRead(ARDUINO_ADDRESS);
  tsprintf(str, "Voltage: %x\r\n", data);
  sendData(str);
}

/**
 **********************************************************************************
 * Serial Comunication Function
 **********************************************************************************
 */
void sendData(const int8_t String[]) {
  DMA1_Configuration((uint32_t)String, strlen(String));
  DMA_Cmd(DMA1_Channel4, ENABLE);                         // DMA channel 4 : USART1_TX (page.227)

  while(DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);
}

uint8_t i2c1Read(uint8_t Slave_Address, uint8_t Register_Address) {
  sendData("1");
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));                               // I2C1 BUSY
  I2C_GenerateSTART(I2C1, ENABLE);                                              // Generate Start Condition

  sendData("2");
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C1, Slave_Address, I2C_Direction_Transmitter);          // Send Address + Direction Bit(0)[Write]

  sendData("3");
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    // Transmitter Mode & DataRegister Empty & I2C BUSY
  I2C_SendData(I2C1, Register_Address);                                         // Send Register Address

  sendData("4");
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));             // Data Transmitted & I2C BUSY
  I2C_GenerateSTART(I2C1, ENABLE);                                              // Generate Start Condition

  sendData("5");
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C1, Slave_Address, I2C_Direction_Receiver);             // Send Address + Direction Bit(1)[Read]

  sendData("6");
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));        // Receiver Mode & I2C BUSY
  I2C_AcknowledgeConfig(I2C1, DISABLE);                                          // Ack Disable, Not Continue
  I2C_GenerateSTOP(I2C1, ENABLE);                                                // Generate Stop Condition

  sendData("7");
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));                 // Data Received & I2C BUSY
  return I2C_ReceiveData(I2C1);                                                  // Recive Data
}

uint8_t arduinoRead(uint8_t Slave_Address) {
  sendData("1");
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));                               // I2C1 BUSY
  I2C_GenerateSTART(I2C1, ENABLE);                                              // Generate Start Condition

  sendData("2");
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C1, Slave_Address, I2C_Direction_Receiver);             // Send Address + Direction Bit(1)[Read]

  sendData("3");
  // Receiver Mode & I2C BUSY
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
    recoveryCount++;
    if(recoveryCount > 9){
      recoveryCount = 0;
      break;
    }
  }
  I2C_AcknowledgeConfig(I2C1, DISABLE);                                          // Ack Disable, Not Continue
  I2C_GenerateSTOP(I2C1, ENABLE);                                                // Generate Stop Condition

  sendData("4");
  // Data Received & I2C BUSY
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)){
    recoveryCount++;
    if(recoveryCount > 9){
      recoveryCount = 0;
      sendData("arduinoI2C FAILED\r\n");
      return 0xFF;
      break;
    }
  }
  return I2C_ReceiveData(I2C1);                                                  // Recive Data
}

void i2c1Write(uint8_t Slave_Address, uint8_t Register_Address, uint8_t Data) {
  I2C_GenerateSTART(I2C1, ENABLE);                                              // Generate Start Condition

  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C1, Slave_Address, I2C_Direction_Transmitter);          // Send Address + Direction Bit(0)[Write]

  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    // Transmitter Mode & DataRegister Empty & I2C BUSY
  I2C_SendData(I2C1, Register_Address);                                         // Send Register Address

  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));             // Data Transmitted & I2C BUSY
  I2C_SendData(I2C1, Data);                                                     // Send Data

  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));             // Data Transmitted & I2C BUSY
  I2C_AcknowledgeConfig(I2C1, DISABLE);                                         // Ack Disable, Not Continue
  I2C_GenerateSTOP(I2C1, ENABLE);                                               // Generate Stop Condition
}

uint8_t i2c2Read(uint8_t Slave_Address, uint8_t Register_Address) {
  while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));                               // I2C2 BUSY
  I2C_GenerateSTART(I2C2, ENABLE);                                              // Generate Start Condition

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C2, Slave_Address, I2C_Direction_Transmitter);          // Send Address + Direction Bit(0)[Write]

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    // Transmitter Mode & DataRegister Empty & I2C BUSY
  I2C_SendData(I2C2, Register_Address);                                         // Send Register Address

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));             // Data Transmitted & I2C BUSY
  I2C_GenerateSTART(I2C2, ENABLE);                                              // Generate Start Condition

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C2, Slave_Address, I2C_Direction_Receiver);             // Send Address + Direction Bit(1)[Read]

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));        // Receiver Mode & I2C BUSY
  I2C_AcknowledgeConfig(I2C2, DISABLE);                                          // Ack Disable, Not Continue
  I2C_GenerateSTOP(I2C2, ENABLE);                                                // Generate Stop Condition

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));                 // Data Received & I2C BUSY
  return I2C_ReceiveData(I2C2);                                                  // Recive Data
}

void i2c2Write(uint8_t Slave_Address, uint8_t Register_Address, uint8_t Data) {
  I2C_GenerateSTART(I2C2, ENABLE);                                              // Generate Start Condition

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));                  // Master Mode & I2C BUSY
  I2C_Send7bitAddress(I2C2, Slave_Address, I2C_Direction_Transmitter);          // Send Address + Direction Bit(0)[Write]

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));    // Transmitter Mode & DataRegister Empty & I2C BUSY
  I2C_SendData(I2C2, Register_Address);                                         // Send Register Address

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));             // Data Transmitted & I2C BUSY
  I2C_SendData(I2C2, Data);                                                     // Send Data

  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));             // Data Transmitted & I2C BUSY
  I2C_AcknowledgeConfig(I2C2, DISABLE);                                         // Ack Disable, Not Continue
  I2C_GenerateSTOP(I2C2, ENABLE);                                               // Generate Stop Condition
}

/**
 ****************************************************************1***************************
 * System Configuration
 *******************************************************************************************
 */
// GPIO Default Configuration -> LED, WakeUp
void GPIO_Configuration(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // Clock Supply to Alternate functions and GPIOD
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);
  // JTAG Disable
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);    // DebugPort(PA13,PA14,PA15,PB3,PB4) release

  // GPIOD Configuration
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;               // PD4 : connect to LED1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        // Mode: Digital Output with pull-up
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       // Max switching speed : 50Hz
  GPIO_Init(GPIOD, &GPIO_InitStructure);                  // Init GPIOD

  GPIO_SetBits(GPIOD, GPIO_Pin_4);                        // PD4 HIGH
}

/**
 *******************************************************************************************
 * Timer Configuration
 *******************************************************************************************
 */
void SysTick_Configuration(void) {
  SysTick -> LOAD = 0xFFFFFF;
  SysTick -> CTRL = 0x000005;
}

void EXTI_Configuration(void) {
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOE , ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = PE0_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init( GPIOE , &GPIO_InitStructure );

  GPIO_EXTILineConfig( GPIO_PortSourceGPIOE , GPIO_PinSource0 );
  GPIO_EXTILineConfig( GPIO_PortSourceGPIOE , GPIO_PinSource1 );
  GPIO_EXTILineConfig( GPIO_PortSourceGPIOE , GPIO_PinSource2 );
  GPIO_EXTILineConfig( GPIO_PortSourceGPIOE , GPIO_PinSource3 );
  GPIO_EXTILineConfig( GPIO_PortSourceGPIOE , GPIO_PinSource4 );
  GPIO_EXTILineConfig( GPIO_PortSourceGPIOE , GPIO_PinSource5 );

  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_Init( &EXTI_InitStructure );

  EXTI_InitStructure.EXTI_Line = 0x003E;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_Init( &EXTI_InitStructure );

  EXTI_ClearITPendingBit(0x003F);

  NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI0_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init( &NVIC_InitStructure );

  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI1_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init( &NVIC_InitStructure );

  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI2_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init( &NVIC_InitStructure );

  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI3_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init( &NVIC_InitStructure );

  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI4_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init( &NVIC_InitStructure );

  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = EXTI9_5_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_Init( &NVIC_InitStructure );
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable DMA1 channel5 IRQ Channel */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIM1_Configuration(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOE, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = TIM1_CH1 | TIM1_CH2 | TIM1_CH3 | TIM1_CH4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 15000-1;
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_Pulse = 0;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);

  TIM_Cmd(TIM1, ENABLE);

  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void TIM2_Configuration(void) {
  /* NVIC Configuration */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;               // TIM2_IRQHandler();
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM2_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // Sub Priority Set
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               // Inturrupt Enable
  NVIC_Init(&NVIC_InitStructure);                               // NVIC Init

  // Clock Supply to TIM2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  // TIM2 general-purpose timer Configuration
  // PSC_frequency = 72MHz / Prescaler;
  // TIM2_Period = Period / PSC_frequency;
  // TIM2_Period = 144 * 50000 / 72*10^6 = 0.1[s] [10Hz]
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 50000-1;                   // Timer Update : count 50000
  TIM_TimeBaseStructure.TIM_Prescaler = 144-1;                   // Division ratio : 1/144
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;                  // Dead Time 0
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   // Counter Up
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);               // Init Time Base TIM2

  // TIM2 Enable
  TIM_Cmd(TIM2, ENABLE);
  // TIM2 interrupt Enable
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void TIM4_Configuration(void) {
  RCC_APB2PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB2Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = TIM4_CH1 | TIM4_CH2 | TIM4_CH3 | TIM4_CH4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 15000-1;
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable);

  TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

  TIM_Cmd(TIM4, ENABLE);
}

void TIM5_Configuration(void) {
  /* NVIC Configuration */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;               // TIM5_IRQHandler();
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM5_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // Sub Priority Set
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               // Inturrupt Enable
  NVIC_Init(&NVIC_InitStructure);                               // NVIC Init

  // Clock Supply to TIM5
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  // TIM5 general-purpose timer Configuration
  // PSC_frequency = 72MHz / Prescaler;
  // TIM5_Period = Period / PSC_frequency;
  // TIM5_Period = 72 * 20000 / 72*10^6 = 0.02[s] [50Hz]
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 20000-1;                   // Timer Update : count 20000
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;                   // Division ratio : 1/72
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;                  // Dead Time 0
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   // Counter Up
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);               // Init Time Base TIM5

  // TIM5 Enable
  TIM_Cmd(TIM5, ENABLE);
  // TIM5 interrupt Enable
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}

void TIM6_Configuration(void) {
  /* NVIC Configuration */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;               // TIM6_IRQHandler();
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM6_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // Sub Priority Set
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;               // Inturrupt Enable
  NVIC_Init(&NVIC_InitStructure);                               // NVIC Init

  // Clock Supply to TIM6
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  // TIM6 general-purpose timer Configuration
  // PSC_frequency = 72MHz / Prescaler;
  // TIM6_Period = Period / PSC_frequency;
  // TIM6_Period = 72 * 50000 / 72*10^6 = 0.05[s] [20Hz]
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 50000-1;                   // Timer Update : count 50000
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;                   // Division ratio : 1/72
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;                  // Dead Time 0
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   // Counter Up
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);               // Init Time Base TIM6

  // TIM6 Enable
  TIM_Cmd(TIM6, ENABLE);
  // TIM6 interrupt Enable
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
}

void TIM7_Configuration(uint16_t usec) {
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
  TIM_Cmd(TIM7, ENABLE);

  TIM_TimeBaseStructure.TIM_Period = usec;
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

  TIM_ClearFlag(TIM7, TIM_FLAG_Update);

  while(!TIM_GetFlagStatus(TIM7, TIM_FLAG_Update));
}

void TIM8_Configuration(void) {

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 10000-1;
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

  TIM_ClearITPendingBit(TIM8, TIM_IT_Update);

  TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);

  TIM_Cmd(TIM8, ENABLE);
  TIM8->CNT = 0;

  TIM_ClearITPendingBit(TIM8, TIM_IT_Update);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = 44;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM8_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
 *******************************************************************************************
 * Serial Configuration
 *******************************************************************************************
 */
void DMA1_Configuration(uint32_t Memory_Address, uint16_t Buffer_Size) {
  DMA_InitTypeDef DMA_InitStructure;

  // Clock Supply to AHB/DMA1
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  DMA_DeInit(DMA1_Channel4);                                              // DMA1 Initialize
  DMA_InitStructure.DMA_BufferSize = Buffer_Size;                         // Set Buffer size : string length
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // Data Transfer direction : Memory => Peripheral circuit
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // Memory to memory transfer : Disable
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Memory_Address;        // string[] initial address
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // Memory transmit data size : 1byte(8bits)
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // Memory address increment : Enable
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // DMA mode : Normal
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;       // Base address : USART1 => DR
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Peripheral circuit transmit data size : 1byte(8bits)
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // Peripheral circuit address increment : Disable
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // DMA Priority : Very HIGH
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);                            // Set Config DMA1
}

void USART1_Configuration(void) {
  // Clock Supply to USART1 and GPIOA, DMA1 Re
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // GPIOA Configuration, USART1 <TRANSMIT>
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                   // connect to USART1_TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;             // Mode: Digital Output with push-pull
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;           // Max switching speed : 50Hz
  GPIO_Init(GPIOA, &GPIO_InitStructure);                      // Init GPIOA
  // GPIOA Configuration, USART1 <RECEIVE>
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                  // connect to USART1_RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       // Mode : floating
  GPIO_Init(GPIOA, &GPIO_InitStructure);                      // Init GPIOA

  /* DMA channel for USART1(DMA1_Channel5) Config */
  DMA_DeInit(DMA1_Channel5);                                              // DMA1 Initialize
  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_BufferSize = RX_BUFFER;                           // Set Buffer size : 8bit(1byte)
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // Data Transfer direction : Memory => Peripheral circuit
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // Memory to memory transfer : Disable
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RxData;                // receive data initial address
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // Memory transmit data size : 1byte(8bits)
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // Memory address increment : Enable
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // DMA mode : Normal
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;       // Base address : USART1 => DR
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Peripheral circuit transmit data size : 1byte(8bits)
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // Peripheral circuit address increment : Disable
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // DMA Priority : Very HIGH
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);                            // Set Config DMA1
  /* Enable DMA1 Channel5 Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
  /* Enable USART1 DMA RX Channel */
  DMA_Cmd(DMA1_Channel5, ENABLE);

  // USART1 Configuration
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 115200;                      // BaudRate : 115200bps
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Hardware flow control : NONE
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;   // MODE : Rx & Tx
  USART_InitStructure.USART_Parity = USART_Parity_No ;              // Parity bit none
  USART_InitStructure.USART_StopBits = USART_StopBits_1;            // Stop bit : 1bit
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;       // Wordlength/(1 transmit and receive) : 8bits
  USART_Init(USART1, &USART_InitStructure);                         // Init USART1
  // DMA Request from USART1 : Transmit <ENABLE>
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  // USART1 Enable
  USART_Cmd(USART1, ENABLE);
}

void I2C1_Configuration(void) {
  // Clock Supply to I2C1 and GPIOB
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  // GPIOB Configuration
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;            // PB6,PB7 : connect to I2C1_SCL, I2C1_SDA
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;                   // Mode: Digital Output with open-drain
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                 // Max switching speed : 50Hz
  GPIO_Init(GPIOB, &GPIO_InitStructure);                            // Init GPIOB

  // I2C Configuration
  I2C_InitTypeDef I2C_InitStructure;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;                       // Acknowledge Enable
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // Acknowledg+e address length
  I2C_InitStructure.I2C_ClockSpeed = 400000;                        // I2C transmission speed
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;                // HIGH,LOW, DutyCycle
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                        // PinMode : I2C
  I2C_Init(I2C1, &I2C_InitStructure);                               // Init I2C1

  // I2C1 Enable
  I2C_Cmd(I2C1, ENABLE);
}

void I2C2_Configuration(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef I2C_InitStructure;

  // Clock Supply to I2C2 and GPIOB
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  // GPIOB Configuration
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;          // PB10 : connect to I2C2_SCL, I2C2_SDA
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;                   // Mode: Digital Output with open-drain
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                 // Max switching speed : 50Hz
  GPIO_Init(GPIOB, &GPIO_InitStructure);                            // Init GPIOB

  // I2C Configuration
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;                       // Acknowledge Enable
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // Acknowledg+e address length
  I2C_InitStructure.I2C_ClockSpeed = 400000;                        // I2C transmission speed
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;                // HIGH,LOW, DutyCycle
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                        // PinMode : I2C
  I2C_Init(I2C2, &I2C_InitStructure);                               // Init I2C2

  // I2C2 Enable
  I2C_Cmd(I2C2, ENABLE);
}

/**
 *****************************************************************************
 * PID
 *****************************************************************************
 */
void PID_Controller(void) {
  float T = SAMPLING_RATE;
  float FA = ((ITA_A * TD_A)/(T+ITA_A * TD_A));
  float FE = ((ITA_E * TD_E)/(T+ITA_E * TD_E));
  float FR = ((ITA_R * TD_R)/(T+ITA_R * TD_R));
  float GA = TD_A/(T+ITA_A * TD_A);
  float GE = TD_E/(T+ITA_E * TD_E);
  float GR = TD_R/(T+ITA_R * TD_R);
  float aileronP = 0;
  float elevatorP = 0;
  float rudderP = 0;
  float aileronI = 0;
  float elevatorI = 0;
  float rudderI = 0;
  float aileronD = 0;
  float elevatorD = 0;
  float rudderD = 0;

  // Error
  phi_e = phi_t - phi;
  theta_e = theta_t - theta;
  psi_e = psi_t - psi;

  //P_controller//
  aileronP = KP_A * (phi_e - phi_e_1);
  elevatorP = KP_E * (theta_e - theta_e_1);
  rudderP = KP_R * (psi_e - psi_e_1);

  //I_controller//
  aileronI = KP_A * ((T / TI_A) * phi_e);
  elevatorI = KP_E * ((T / TI_E) * theta_e) *0;
  rudderI = KP_R * ((T / TI_R) * psi_e) *0;

  //D_controller//
  Ud_a = FA * Ud_pa + GA * (phi_e - 2 * phi_e_1 + phi_e_2);
  Ud_e = FE * Ud_pe + GE * (theta_e - 2 * theta_e_1 + theta_e_2);
  Ud_r = FR * Ud_pr + GR * (psi_e - 2 * psi_e_1 + psi_e_2);
  aileronD = KP_A * Ud_a;
  elevatorD = KP_E * Ud_e;
  rudderD = KP_R * Ud_r;

  //PID_controller//
  du_a = aileronP + aileronI + aileronD;
  du_e = elevatorP + elevatorI + elevatorD;
  du_r = rudderP + rudderI + rudderD;
  U[0] = U_p[0] + du_a;
  U[1] = U_p[1] + du_e;
  U[2] = U_p[2] + du_r;

  //Antiwindup//
  antiWindUp();
  Aileron_u = aileronDeg2Duty(-1 * U[0]);
  Elevator_u = elevatorDeg2Duty(-1 * U[1]);
  Rudder_u = rudderDeg2Duty(U[2]);

  // Preservation : One step before and Two step before
  phi_e_2 = phi_e_1;
  theta_e_2 = theta_e_1;
  psi_e_2 = psi_e_1;

  phi_e_1 = phi_e;
  theta_e_1 = theta_e;
  psi_e_1 = psi_e;

  Ud_pa = Ud_a;
  Ud_pe = Ud_e;
  Ud_pr = Ud_r;

  U_p[0] = U[0];
  U_p[1] = U[1];
  U_p[2] = U[2];
}

void PI_D_Controller(void) {
  float T = SAMPLING_RATE;
  float FA = ((ITA_A * TD_A)/(T+ITA_A * TD_A));
  float FE = ((ITA_E * TD_E)/(T+ITA_E * TD_E));
  float FR = ((ITA_R * TD_R)/(T+ITA_R * TD_R));
  float GA = TD_A/(T+ITA_A * TD_A);
  float GE = TD_E/(T+ITA_E * TD_E);
  float GR = TD_R/(T+ITA_R * TD_R);
  float aileronP = 0;
  float elevatorP = 0;
  float rudderP = 0;
  float aileronI = 0;
  float elevatorI = 0;
  float rudderI = 0;
  float aileronD = 0;
  float elevatorD = 0;
  float rudderD = 0;

  // Error
  phi_e = phi_t - phi;
  theta_e = theta_t - theta;
  psi_e = psi_t - psi;

  //P_controller//
  aileronP = KP_A * (phi_e - phi_e_1);
  elevatorP = KP_E * (theta_e - theta_e_1);
  rudderP = KP_R * (psi_e - psi_e_1);

  //I_controller//
  aileronI = KP_A * ((T / TI_A) * phi_e) *0;
  elevatorI = KP_E * ((T / TI_E) * theta_e) *0;
  rudderI = KP_R * ((T / TI_R) * psi_e) *0;

  //D_controller//
  Ud_a = FA * Ud_pa + GA * (2*phi_1 - phi - phi_2);
  Ud_e = FE * Ud_pe + GE * (2*theta_1 - theta - theta_2);
  Ud_r = FR * Ud_pr + GR * (2*psi_1 - psi - psi_2);
  aileronD = KP_A * Ud_a;
  elevatorD = KP_E * Ud_e;
  rudderD = KP_R * Ud_r;

  //PID_controller//
  du_a = aileronP + aileronI + aileronD;
  du_e = elevatorP + elevatorI + elevatorD;
  du_r = rudderP + rudderI + rudderD;
  U[0] = U_p[0] + du_a;
  U[1] = U_p[1] + du_e;
  U[2] = U_p[2] + du_r;

  //Antiwindup//
  antiWindUp();
  Aileron_u = aileronDeg2Duty(-1 * U[0]);
  Elevator_u = elevatorDeg2Duty(U[1]);
  Rudder_u = rudderDeg2Duty(U[2]);

  // Preservation : One step before and Two step before
  phi_e_2 = phi_e_1;
  theta_e_2 = theta_e_1;
  psi_e_2 = psi_e_1;

  phi_e_1 = phi_e;
  theta_e_1 = theta_e;
  psi_e_1 = psi_e;

  phi_1 = phi;
  theta_1 = theta;
  psi_1 = psi;

  phi_2 = phi_1;
  theta_2 = theta_1;
  psi_2 = psi_1;

  Ud_pa = Ud_a;
  Ud_pe = Ud_e;
  Ud_pr = Ud_r;

  U_p[0] = U[0];
  U_p[1] = U[1];
  U_p[2] = U[2];
}

void I_PD_Controller(void) {
  float T = SAMPLING_RATE;
  float FA = ((ITA_A * TD_A)/(T+ITA_A * TD_A));
  float FE = ((ITA_E * TD_E)/(T+ITA_E * TD_E));
  float FR = ((ITA_R * TD_R)/(T+ITA_R * TD_R));
  float GA = TD_A/(T+ITA_A * TD_A);
  float GE = TD_E/(T+ITA_E * TD_E);
  float GR = TD_R/(T+ITA_R * TD_R);
  float aileronP = 0;
  float elevatorP = 0;
  float rudderP = 0;
  float aileronI = 0;
  float elevatorI = 0;
  float rudderI = 0;
  float aileronD = 0;
  float elevatorD = 0;
  float rudderD = 0;

  // Error
  phi_e = phi_t - phi;
  theta_e = theta_t - theta;
  psi_e = psi_t - psi;

  //P_controller//
  aileronP = KP_A * (phi_1 - phi);
  elevatorP = KP_E * (theta_1 - theta);
  rudderP = KP_R * (psi_1 - psi);

  //I_controller//
  aileronI = KP_A * ((T / TI_A) * phi_e) *0;
  elevatorI = KP_E * ((T / TI_E) * theta_e) *0;
  rudderI = KP_R * ((T / TI_R) * psi_e) *0;

  //D_controller//
  Ud_a = FA * Ud_pa + GA * (2*phi_1 - phi - phi_2);
  Ud_e = FE * Ud_pe + GE * (2*theta_1 - theta - theta_2);
  Ud_r = FR * Ud_pr + GR * (2*psi_1 - psi - psi_2);
  aileronD = KP_A * Ud_a;
  elevatorD = KP_E * Ud_e;
  rudderD = KP_R * Ud_r;

  //PID_controller//
  du_a = aileronP + aileronI + aileronD;
  du_e = elevatorP + elevatorI + elevatorD;
  du_r = rudderP + rudderI + rudderD;
  U[0] = U_p[0] + du_a;
  U[1] = U_p[1] + du_e;
  U[2] = U_p[2] + du_r;

  //Antiwindup//
  antiWindUp();
  Aileron_u = aileronDeg2Duty(-1 * U[0]);
  Elevator_u = elevatorDeg2Duty(U[1]);
  Rudder_u = rudderDeg2Duty(U[2]);

  // Preservation : One step before and Two step before
  phi_e_2 = phi_e_1;
  theta_e_2 = theta_e_1;
  psi_e_2 = psi_e_1;

  phi_e_1 = phi_e;
  theta_e_1 = theta_e;
  psi_e_1 = psi_e;

  phi_1 = phi;
  theta_1 = theta;
  psi_1 = psi;

  phi_2 = phi_1;
  theta_2 = theta_1;
  psi_2 = psi_1;

  Ud_pa = Ud_a;
  Ud_pe = Ud_e;
  Ud_pr = Ud_r;

  U_p[0] = U[0];
  U_p[1] = U[1];
  U_p[2] = U[2];
}
//rollrateの目標値rollrate_tは現状33.8deg/s//
//実用非干渉PID制御にてエルロンの入力をrollrateをフィードバックすることで決定//
//エレベータ、ラダーは通常の実用非干渉PID制御//
void PID_Roll_Controller(void) {
  float T = SAMPLING_RATE;
  float FA = ((ITA_ROLL * TD_ROLL)/(T+ITA_ROLL * TD_ROLL));
  float FE = ((ITA_E * TD_E)/(T+ITA_E * TD_E));
  float FR = ((ITA_R * TD_R)/(T+ITA_R * TD_R));
  float GA = TD_ROLL/(T+ITA_ROLL * TD_ROLL);
  float GE = TD_E/(T+ITA_E * TD_E);
  float GR = TD_R/(T+ITA_R * TD_R);
  float aileronP = 0;
  float elevatorP = 0;
  float rudderP = 0;
  float aileronI = 0;
  float elevatorI = 0;
  float rudderI = 0;
  float aileronD = 0;
  float elevatorD = 0;
  float rudderD = 0;

  //filter
  //gyroRollLPF = (1 - 0.2) * gyroRollLPF + 0.2 * gyroData[0];
  angVel = (phi - phi_1)/ 0.05;
  // Error
  rollrate_e = rollrate_t - gyroRollLPF;
  theta_e = theta_t - theta;
  psi_e = psi_t - psi;

  //P_controller//
  aileronP = KP_A * (rollrate_e - rollrate_e_1);
  elevatorP = KP_E * (theta_e - theta_e_1);
  rudderP = KP_R * (psi_e - psi_e_1);

  //I_controller//
  aileronI = KP_A * ((T / TI_A) * rollrate_e) *0;
  elevatorI = KP_E * ((T / TI_E) * theta_e) *0;
  rudderI = KP_R * ((T / TI_R) * psi_e) *0;

  //D_controller//
  Ud_a = FA * Ud_pa + GA * (rollrate_e - 2 * rollrate_e_1 + rollrate_e_2);
  Ud_e = FE * Ud_pe + GE * (theta_e - 2 * theta_e_1 + theta_e_2);
  Ud_r = FR * Ud_pr + GR * (psi_e - 2 * psi_e_1 + psi_e_2);
  aileronD = KP_A * Ud_a;
  elevatorD = KP_E * Ud_e;
  rudderD = KP_R * Ud_r;

  //PID_controller//
  du_a = aileronP + aileronI + aileronD;
  du_e = elevatorP + elevatorI + elevatorD;
  du_r = rudderP + rudderI + rudderD;
  U[0] = U_p[0] + du_a;
  U[1] = U_p[1] + du_e;
  U[2] = U_p[2] + du_r;

  //Antiwindup//
  antiWindUp();
  Aileron_u = aileronDeg2Duty(-1 * U[0]);
  Elevator_u = elevatorDeg2Duty(U[1]);
  Rudder_u = rudderDeg2Duty(U[2]);

  phi_1 = phi;

  // Preservation : One step before and Two step before
  rollrate_e_2 = rollrate_e_1;
  theta_e_2 = theta_e_1;
  psi_e_2 = psi_e_1;

  rollrate_e_1 = rollrate_e;
  theta_e_1 = theta_e;
  psi_e_1 = psi_e;

  Ud_pa = Ud_a;
  Ud_pe = Ud_e;
  Ud_pr = Ud_r;

  U_p[0] = U[0];
  U_p[1] = U[1];
  U_p[2] = U[2];
}
//測定値微分先行型PID制御にてエルロンの入力をrollrateをフィードバックすることで決定//
//エレベータ、ラダーも測定値微分先行型PID制御//
void PI_D_Roll_Controller(void) {
  float T = SAMPLING_RATE;
  float FA = ((ITA_ROLL * TD_ROLL)/(T+ITA_ROLL * TD_ROLL));
  float FE = ((ITA_E * TD_E)/(T+ITA_E * TD_E));
  float FR = ((ITA_R * TD_R)/(T+ITA_R * TD_R));
  float GA = TD_ROLL/(T+ITA_ROLL * TD_ROLL);
  float GE = TD_E/(T+ITA_E * TD_E);
  float GR = TD_R/(T+ITA_R * TD_R);
  float aileronP = 0;
  float elevatorP = 0;
  float rudderP = 0;
  float aileronI = 0;
  float elevatorI = 0;
  float rudderI = 0;
  float aileronD = 0;
  float elevatorD = 0;
  float rudderD = 0;

  // Error
  rollrate_e = rollrate_t - gyroData[0];
  theta_e = theta_t - theta;
  psi_e = psi_t - psi;

  //P_controller//
  aileronP = KP_A * (rollrate_e - rollrate_e_1);
  elevatorP = KP_E * (theta_e - theta_e_1);
  rudderP = KP_R * (psi_e - psi_e_1);

  //I_controller//
  aileronI = KP_A * ((T / TI_ROLL) * rollrate_e) *0;
  elevatorI = KP_E * ((T / TI_E) * theta_e) *0;
  rudderI = KP_R * ((T / TI_R) * psi_e) *0;

  //D_controller//
  Ud_a = FA * Ud_pa + GA * (2*rollrate_1 - gyroData[0] - rollrate_2);
  Ud_e = FE * Ud_pe + GE * (2*theta_1 - theta - theta_2);
  Ud_r = FR * Ud_pr + GR * (2*psi_1 - psi - psi_2);
  aileronD = KP_A * Ud_a;
  elevatorD = KP_E * Ud_e;
  rudderD = KP_R * Ud_r;

  //PID_controller//
  du_a = aileronP + aileronI + aileronD;
  du_e = elevatorP + elevatorI + elevatorD;
  du_r = rudderP + rudderI + rudderD;
  U[0] = U_p[0] + du_a;
  U[1] = U_p[1] + du_e;
  U[2] = U_p[2] + du_r;

  //Antiwindup//
  antiWindUp();
  Aileron_u = aileronDeg2Duty(-1 * U[0]);
  Elevator_u = elevatorDeg2Duty(U[1]);
  Rudder_u = rudderDeg2Duty(U[2]);

  // Preservation : One step before and Two step before
  rollrate_e_2 = rollrate_e_1;
  theta_e_2 = theta_e_1;
  psi_e_2 = psi_e_1;

  rollrate_e_1 = rollrate_e;
  theta_e_1 = theta_e;
  psi_e_1 = psi_e;

  rollrate_1 = gyroData[0];
  theta_1 = theta;
  psi_1 = psi;

  rollrate_2 = rollrate_1;
  theta_2 = theta_1;
  psi_2 = psi_1;

  Ud_pa = Ud_a;
  Ud_pe = Ud_e;
  Ud_pr = Ud_r;

  U_p[0] = U[0];
  U_p[1] = U[1];
  U_p[2] = U[2];
}
//測定値比例微分先行型PID制御にてエルロンの入力をrollrateをフィードバックすることで決定//
//エレベータ、ラダーも測定値比例微分先行型PID制御//
void I_PD_Roll_Controller(void) {
  float T = SAMPLING_RATE;
  float FA = ((ITA_ROLL * TD_ROLL)/(T+ITA_ROLL * TD_ROLL));
  float FE = ((ITA_E * TD_E)/(T+ITA_E * TD_E));
  float FR = ((ITA_R * TD_R)/(T+ITA_R * TD_R));
  float GA = TD_ROLL/(T+ITA_ROLL * TD_ROLL);
  float GE = TD_E/(T+ITA_E * TD_E);
  float GR = TD_R/(T+ITA_R * TD_R);
  float aileronP = 0;
  float elevatorP = 0;
  float rudderP = 0;
  float aileronI = 0;
  float elevatorI = 0;
  float rudderI = 0;
  float aileronD = 0;
  float elevatorD = 0;
  float rudderD = 0;

  // Error
  rollrate_e = rollrate_t - gyroData[0];
  theta_e = theta_t - theta;
  psi_e = psi_t - psi;

  //P_controller//
  aileronP = KP_A * (rollrate_1 - gyroData[0]);
  elevatorP = KP_E * (theta_1 - theta);
  rudderP = KP_R * (psi_1 - psi);

  //I_controller//
  aileronI = KP_A * ((T / TI_ROLL) * rollrate_e) *0;
  elevatorI = KP_E * ((T / TI_E) * theta_e) *0;
  rudderI = KP_R * ((T / TI_R) * psi_e) *0;

  //D_controller//
  Ud_a = FA * Ud_pa + GA * (2*rollrate_1 - gyroData[0] - rollrate_2);
  Ud_e = FE * Ud_pe + GE * (2*theta_1 - theta - theta_2);
  Ud_r = FR * Ud_pr + GR * (2*psi_1 - psi - psi_2);
  aileronD = KP_A * Ud_a;
  elevatorD = KP_E * Ud_e;
  rudderD = KP_R * Ud_r;

  //PID_controller//
  du_a = aileronP + aileronI + aileronD;
  du_e = elevatorP + elevatorI + elevatorD;
  du_r = rudderP + rudderI + rudderD;
  U[0] = U_p[0] + du_a;
  U[1] = U_p[1] + du_e;
  U[2] = U_p[2] + du_r;

  //Antiwindup//
  antiWindUp();
  Aileron_u = aileronDeg2Duty(-1 * U[0]);
  Elevator_u = elevatorDeg2Duty(U[1]);
  Rudder_u = rudderDeg2Duty(U[2]);

  // Preservation : One step before and Two step before
  rollrate_e_2 = rollrate_e_1;
  theta_e_2 = theta_e_1;
  psi_e_2 = psi_e_1;

  rollrate_e_1 = rollrate_e;
  theta_e_1 = theta_e;
  psi_e_1 = psi_e;

  rollrate_1 = gyroData[0];
  theta_1 = theta;
  psi_1 = psi;

  rollrate_2 = rollrate_1;
  theta_2 = theta_1;
  psi_2 = psi_1;

  Ud_pa = Ud_a;
  Ud_pe = Ud_e;
  Ud_pr = Ud_r;

  U_p[0] = U[0];
  U_p[1] = U[1];
  U_p[2] = U[2];
}

void antiWindUp(void) {
  if(U[0] > AILERON_MAX) {
    U[0] = AILERON_MAX;
  } else if (U[0] < AILERON_MIN) {
    U[0] = AILERON_MIN;
  }
  if(U[1] > ELEVATOR_MAX) {
    U[1] = ELEVATOR_MAX;
  } else if (U[1] < ELEVATOR_MIN) {
    U[1] = ELEVATOR_MIN;
  }
  if(U[2] > RUDDER_MAX) {
    U[2] = RUDDER_MAX;
  } else if (U[2] < RUDDER_MIN) {
    U[2] = RUDDER_MIN;
  }
}

int32_t aileronDeg2Duty(double u) {
  return normalize((int32_t)u, AILERON_MIN, AILERON_MAX, aileronDutyMin, aileronDutyMax);
}

int32_t elevatorDeg2Duty(double u) {
  return normalize((int32_t)u, ELEVATOR_MIN, ELEVATOR_MAX, elevatorDutyMin, elevatorDutyMax);
}

int32_t rudderDeg2Duty(double u) {
  return normalize((int32_t)u, RUDDER_MIN, RUDDER_MAX, rudderDutyMin, rudderDutyMax);
}

int32_t normalize(int32_t val, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax) {
  return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/**
 *******************************************************************************************
 * MadgwickAHRS
 *******************************************************************************************
 */

 void MadgwickAHRS(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz) {
   double recipNorm;
   double s0, s1, s2, s3;
   double qDot1, qDot2, qDot3, qDot4;
   double hx, hy;
   double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

   // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
   if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
     MadgwickAHRS_IMU(gx, gy, gz, ax, ay, az);
   }

   // Convert gyroscope degrees/sec to radians/sec
   gx *= 0.0174533;
   gy *= 0.0174533;
   gz *= 0.0174533;

   // Rate of change of quaternion from gyroscope
   qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
   qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
   qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
   qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

   // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
   if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
     // Normalise accelerometer measurement
     recipNorm = invSqrt(ax * ax + ay * ay + az * az);
     ax *= recipNorm;
     ay *= recipNorm;
     az *= recipNorm;

     // Normalise magnetometer measurement
     recipNorm = invSqrt(mx * mx + my * my + mz * mz);
     mx *= recipNorm;
     my *= recipNorm;
     mz *= recipNorm;

     // Auxiliary variables to avoid repeated arithmetic
     _2q0mx = 2.0 * q0 * mx;
     _2q0my = 2.0 * q0 * my;
     _2q0mz = 2.0 * q0 * mz;
     _2q1mx = 2.0 * q1 * mx;
     _2q0 = 2.0 * q0;
     _2q1 = 2.0 * q1;
     _2q2 = 2.0 * q2;
     _2q3 = 2.0 * q3;
     _2q0q2 = 2.0 * q0 * q2;
     _2q2q3 = 2.0 * q2 * q3;
     q0q0 = q0 * q0;
     q0q1 = q0 * q1;
     q0q2 = q0 * q2;
     q0q3 = q0 * q3;
     q1q1 = q1 * q1;
     q1q2 = q1 * q2;
     q1q3 = q1 * q3;
     q2q2 = q2 * q2;
     q2q3 = q2 * q3;
     q3q3 = q3 * q3;

     // Reference direction of Earth's magnetic field
     hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
     hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
     _2bx = sqrtf(hx * hx + hy * hy);
     _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
     _4bx = 2.0 * _2bx;
     _4bz = 2.0 * _2bz;

     // Gradient decent algorithm corrective step
     s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
     s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
     s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
     s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
     recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
     s0 *= recipNorm;
     s1 *= recipNorm;
     s2 *= recipNorm;
     s3 *= recipNorm;

     // Apply feedback step
     qDot1 -= beta * s0;
     qDot2 -= beta * s1;
     qDot3 -= beta * s2;
     qDot4 -= beta * s3;

      phi  = getPhi();
      theta = getTheta();
      psi   = getPsi();
   }

   // Integrate rate of change of quaternion to yield quaternion
   q0 += qDot1 * invSampleFreq;
   q1 += qDot2 * invSampleFreq;
   q2 += qDot3 * invSampleFreq;
   q3 += qDot4 * invSampleFreq;

   // Normalise quaternion
   recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
   q0 *= recipNorm;
   q1 *= recipNorm;
   q2 *= recipNorm;
   q3 *= recipNorm;
   anglesComputed = 0.0;
 }

 void MadgwickAHRS_IMU(double gx, double gy, double gz, double ax, double ay, double az) {
   double recipNorm;
   double s0, s1, s2, s3;
   double qDot1, qDot2, qDot3, qDot4;
   double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

   // Convert gyroscope degrees/sec to radians/sec
   gx *= 0.0174533;
   gy *= 0.0174533;
   gz *= 0.0174533;

   // Rate of change of quaternion from gyroscope
   qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
   qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
   qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
   qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

   // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
   if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

     // Normalise accelerometer measurement
     recipNorm = invSqrt(ax * ax + ay * ay + az * az);
     ax *= recipNorm;
     ay *= recipNorm;
     az *= recipNorm;

     // Auxiliary variables to avoid repeated arithmetic
     _2q0 = 2.0 * q0;
     _2q1 = 2.0 * q1;
     _2q2 = 2.0 * q2;
     _2q3 = 2.0 * q3;
     _4q0 = 4.0 * q0;
     _4q1 = 4.0 * q1;
     _4q2 = 4.0 * q2;
     _8q1 = 8.0 * q1;
     _8q2 = 8.0 * q2;
     q0q0 = q0 * q0;
     q1q1 = q1 * q1;
     q2q2 = q2 * q2;
     q3q3 = q3 * q3;

     // Gradient decent algorithm corrective step
     s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
     s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
     s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
     s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
     recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
     s0 *= recipNorm;
     s1 *= recipNorm;
     s2 *= recipNorm;
     s3 *= recipNorm;

     // Apply feedback step
     qDot1 -= beta * s0;
     qDot2 -= beta * s1;
     qDot3 -= beta * s2;
     qDot4 -= beta * s3;
   }

   // Integrate rate of change of quaternion to yield quaternion
   q0 += qDot1 * invSampleFreq;
   q1 += qDot2 * invSampleFreq;
   q2 += qDot3 * invSampleFreq;
   q3 += qDot4 * invSampleFreq;

   // Normalise quaternion
   recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
   q0 *= recipNorm;
   q1 *= recipNorm;
   q2 *= recipNorm;
   q3 *= recipNorm;
   anglesComputed = 0.0;
 }

 double invSqrt(double x) {
    return 1.0 / (double) sqrt(x);
 }

 double getPhi() {
    if (!anglesComputed) computeAngles();
    return phi * 57.29578;
 }
 double getTheta() {
    if (!anglesComputed) computeAngles();
    return theta * 57.29578;
 }
 double getPsi() {
    if (!anglesComputed) computeAngles();
    return psi * 57.29578;
 }
 double getPhiRadians() {
    if (!anglesComputed) computeAngles();
    return phi;
 }
 double getThetaRadians() {
    if (!anglesComputed) computeAngles();
    return theta;
 }
 double getPsiRadians() {
    if (!anglesComputed) computeAngles();
    return psi;
 }

 void computeAngles(){
    phi = atan2f(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2);
    theta = -1 * asinf(-2.0 * (q1*q3 - q0*q2));
    psi = atan2f(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3);
    anglesComputed = 1.0;
 }

/**
 *******************************************************************************************
 * General purpose function
 *******************************************************************************************
 */

void Toggle_LED(void) {
  if((GPIOD->ODR & 0x0010) >> 4) {
    GPIO_ResetBits(GPIOD, GPIO_Pin_4);
  } else {
    GPIO_SetBits(GPIOD, GPIO_Pin_4);
  }
}

void _delay_us(uint16_t usec) {
  if(usec == 0) {
    return;
  }
  TIM7->CNT = 0;
  TIM7_Configuration(usec);
}

void _delay_ms(uint16_t msec) {
  uint16_t count_1ms;
  if(msec == 0) {
    return;
  }
  for(count_1ms = 0; count_1ms < msec; count_1ms++) {
    _delay_us(995);
  }
}

/* Protocol */
void sendDataOption(void) {
  tsprintf(str, ""
#ifdef INDEX
      ",%d"
#endif
#ifdef CCNT
      ",M:[%d|%d]"
#endif
#ifdef ACCELE_X
      ",%d"
#endif
#ifdef ACCELE_Y
      ",%d"
#endif
#ifdef ACCELE_Z
      ",%d"
#endif
#ifdef GYRO_X
      ",%d"
#endif
#ifdef GYRO_Y
      ",%d"
#endif
#ifdef GYRO_Z
      ",%d"
#endif
#ifdef MAG_X
      ",%d"
#endif
#ifdef MAG_Y
      ",%d"
#endif
#ifdef MAG_Z
      ",%d"
#endif
#ifdef PHI_OP
      ",%d"
#endif
#ifdef THETA_OP
      ",%d"
#endif
#ifdef PSI_OP
      ",%d"
#endif
#ifdef RECEIVER_DATA
      ",%d,%d,%d,%d,%d,%d"
#endif
#ifdef TEMP
      ",%d"
#endif
#ifdef PRESS
      ",%d"
#endif
#ifdef ALTITUDE
      ",[AL:%d]"
#endif
      "\r\n"
#ifdef INDEX
      , count
#endif
#ifdef CCNT
      , ControlCounter,ControlMode
#endif
#ifdef ACCELE_X
      , -1*(int32_t)(accelData[0]*SIGNFIG)
#endif
#ifdef ACCELE_Y
      , (int32_t)(accelData[1]*SIGNFIG)
#endif
#ifdef ACCELE_Z
      , (int32_t)(accelData[2]*SIGNFIG)
#endif
#ifdef GYRO_X
      , (int32_t)(gyroData[0]*SIGNFIG)
#endif
#ifdef GYRO_Y
      , (int32_t)(gyroData[1]*SIGNFIG)
#endif
#ifdef GYRO_Z
      , (int32_t)(gyroData[2]*SIGNFIG)
#endif
#ifdef MAG_X
      , (int32_t)(magData[0]*SIGNFIG)
#endif
#ifdef MAG_Y
      , (int32_t)(magData[1]*SIGNFIG)
#endif
#ifdef MAG_Z
      , (int32_t)(magData[2]*SIGNFIG)
#endif
#ifdef PHI_OP
      , (int32_t)phi
#endif
#ifdef THETA_OP
      , (int32_t)theta
#endif
#ifdef PSI_OP
      , (int32_t)psi
#endif
#ifdef RECEIVER_DATA
      ,aileronDuty,elevatorDuty,throttleDuty,rudderDuty,modeDuty,vpp
#endif
#ifdef TEMP
      , (int32_t)(temperature)
#endif
#ifdef PRESS
      , (int32_t)(pressure)
#endif
#ifdef ALTITUDE
      , (int32_t)(altitude*SIGNFIG)
#endif
  );
  sendData(str);
}

void sendDataOffset(void) {
  tsprintf(str, "Offset: "
#ifdef ACCELE_OFFSET
      ",%d,%d,%d"
#endif
#ifdef MAG_OFFSET
      ",%d,%d,%d"
#endif
      "\r\n"
#ifdef ACCELE_OFFSET
      , accelOffset[0], accelOffset[1], accelOffset[2]
#endif
#ifdef MAG_OFFSET
      , magOffset[0], magOffset[1], magOffset[2]
#endif
  );
  sendData(str);
}

void sendDataCalibration(void) {
  tsprintf(str, "Calibration: "
#ifdef MAG_OFFSET
      ",%d,%d,%d"
#endif
      "\r\n"
#ifdef MAG_OFFSET
      , magCalib[0], magCalib[1], magCalib[2]
#endif
  );
  sendData(str);
}

void sendDataPID(void){
  tsprintf(str, "PID:"
#ifdef MODE
      "M[%d|%d]"
#endif
#ifdef PHI
      ",%d"
#endif
#ifdef THETA
      ",%d"
#endif
#ifdef PSI
      ",%d"
#endif
#ifdef U_AILERON
      ",%d"
#endif
#ifdef U_ELEVATOR
      ",%d"
#endif
#ifdef U_RUDDER
      ",%d"
#endif
#ifdef PHI_E
      ",%d"
#endif
#ifdef THETA_E
      ",%d"
#endif
#ifdef PSI_E
      ",%d"
#endif
#ifdef RECEIVER_DATA_PID
      ",%d,%d,%d,%d,%d,%d"
#endif
#ifdef TEMP
      ",%d"
#endif
#ifdef PRESS
      ",%d"
#endif
#ifdef ALTITUDE
      ",[AL:%d]"
#endif
#ifdef REYAW
      ",[rYAW:%d]"
#endif
#ifdef GYRO_X_PID
      ",[GY:%d]"
#endif
      "\r\n"
#ifdef MODE
      , ControlMode,modeView
#endif
#ifdef PHI
      , (int32_t)phi
#endif
#ifdef THETA
      , (int32_t)theta
#endif
#ifdef PSI
      , (int32_t)psi
#endif
#ifdef U_AILERON
      , (int32_t)(U[0])
#endif
#ifdef U_ELEVATOR
      , (int32_t)(U[1])
#endif
#ifdef U_RUDDER
      , (int32_t)(U[2])
#endif
#ifdef PHI_E
      , (int32_t)phi_e
#endif
#ifdef THETA_E
      , (int32_t)theta_e
#endif
#ifdef PSI_E
      , (int32_t)psi_e
#endif
#ifdef RECEIVER_DATA_PID
      ,Aileron_u,Elevator_u,ctrlThrottle,Rudder_u,modeDuty,vpp
#endif
#ifdef TEMP
      , (int32_t)(temperature)
#endif
#ifdef PRESS
      , (int32_t)(pressure)
#endif
#ifdef ALTITUDE
      , (int32_t)(altitude*SIGNFIG)
#endif
#ifdef REYAW
      , (int32_t)relativeYaw
#endif
#ifdef GYRO_X_PID
      , (int32_t)angVel
#endif
  );
  sendData(str);
}
