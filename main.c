/**
 * MODIFIED BY CHARLES ZALOOM - 9/13/18
 * IMPLEMENTATION OF EMBEDDEDML IN LEARNING THE XORAND GATE
 **/

/**
 ******************************************************************************
 * @file    DataLog/Src/main.c
 * @author  Central Labs
 * @version V1.1.1
 * @date    06-Dec-2016
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <string.h> /* strlen */
#include <stdio.h>  /* sprintf */
#include <math.h>   /* trunc */
#include "embeddedML.h"
#include "main.h"

#include "datalog_application.h"
#include "usbd_cdc_interface.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Data acquisition period [ms] */
#define DATA_PERIOD_MS (1000)
//#define NOT_DEBUGGING

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* SendOverUSB = 0  --> Save sensors data on SDCard (enable with double click) */
/* SendOverUSB = 1  --> Send sensors data via USB */
uint8_t SendOverUSB = 1;

USBD_HandleTypeDef USBD_Device;
static volatile uint8_t MEMSInterrupt = 0;
static volatile uint8_t no_H_HTS221 = 0;
static volatile uint8_t no_T_HTS221 = 0;
static volatile uint8_t no_GG = 0;

static RTC_HandleTypeDef RtcHandle;
static void *LSM6DSM_X_0_handle = NULL;
static void *LSM6DSM_G_0_handle = NULL;
static void *LSM303AGR_X_0_handle = NULL;
static void *LSM303AGR_M_0_handle = NULL;
static void *LPS22HB_P_0_handle = NULL;
static void *LPS22HB_T_0_handle = NULL;
static void *HTS221_H_0_handle = NULL;
static void *HTS221_T_0_handle = NULL;
static void *GG_handle = NULL;

/* Private function prototypes -----------------------------------------------*/

static void Error_Handler(void);
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
static void initializeAllSensors(void);

/* Private functions ---------------------------------------------------------*/

#define N_EPOCH   2000
#define N_REPORT  200

void generate_xorxor(float *x, float *y, int i) {
	int k;

	k = rand() % 8;

	switch (k) {
	case 0:
		x[0] = 0.0;
		x[1] = 0.0;
		x[2] = 1.0;
		y[0] = 0.0;
		y[1] = 1.0;
		break;
	case 1:
		x[0] = 0.0;
		x[1] = 1.0;
		x[2] = 1.0;
		y[0] = 1.0;
		y[1] = 0.0;
		break;
	case 2:
		x[0] = 1.0;
		x[1] = 0.0;
		x[2] = 1.0;
		y[0] = 1.0;
		y[1] = 1.0;
		break;
	case 3:
		x[0] = 1.0;
		x[1] = 1.0;
		x[2] = 1.0;
		y[0] = 0.0;
		y[1] = 0.0;
		break;
	case 4:
		x[0] = 0.0;
		x[1] = 0.0;
		x[2] = 0.0;
		y[0] = 0.0;
		y[1] = 0.0;
		break;
	case 5:
		x[0] = 0.0;
		x[1] = 1.0;
		x[2] = 0.0;
		y[0] = 1.0;
		y[1] = 1.0;
		break;
	case 6:
		x[0] = 1.0;
		x[1] = 0.0;
		x[2] = 0.0;
		y[0] = 1.0;
		y[1] = 0.0;
		break;
	case 7:
		x[0] = 1.0;
		x[1] = 1.0;
		x[2] = 0.0;
		y[0] = 0.0;
		y[1] = 1.0;
		break;
	default:
		x[0] = 0.0;
		x[1] = 0.0;
		x[2] = 1.0;
		y[0] = 0.0;
		y[1] = 1.0;
		break;
	}

	/*
	 * The XOR-XOR system supplies three input values and
	 * two Ground Truth output values.
	 *
	 * Since 6 input neurons form the neural network, the
	 * remaining four Ground Truth values are set to zero.
	 *
	 */

	y[2] = 0.0;
	y[3] = 0.0;
	y[4] = 0.0;
	y[5] = 0.0;
}

void Output_Error(int size, ANN *net, float * ground_truth, float *error) {

	int i;
	int32_t d1, d2;
	char msg[128];

	/*
	 * Compute error as mean squared difference between
	 * output and Ground Truth
	 */

	*error = 0;
	for (i = 0; i < size; i++) {
		*error = *error
				+ (net->output[i] - ground_truth[i]) * (net->output[i] - ground_truth[i]);
	}
	*error = *error / size;

	floatToInt(net->output[0], &d1, &d2, 3);
	sprintf(msg, "\n\rOutput: %d.%03d", (int) d1, (int) d2);
	CDC_Fill_Buffer((uint8_t *) msg, strlen(msg));
	floatToInt(net->output[1], &d1, &d2, 3);
	sprintf(msg, "\t%d.%03d", (int) d1, (int) d2);
	CDC_Fill_Buffer((uint8_t *) msg, strlen(msg));
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
	uint32_t msTick, msTickPrev = 0;
	uint8_t doubleTap = 0;
	unsigned int i;
	char epochOut[128];
	char Report_Message[128];

	float ground_truth[2];
	float error, net_error;
	int32_t d1, d2;
	float weights[81];

	/* STM32L4xx HAL library initialization:
	 - Configure the Flash prefetch, instruction and Data caches
	 - Configure the Systick to generate an interrupt each 1 msec
	 - Set NVIC Group Priority to 4
	 - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	if (SendOverUSB) {
		/* Initialize LED */
		BSP_LED_Init(LED1);
		BSP_LED_On(LED1);
	}
#ifdef NOT_DEBUGGING     
	else
	{
		/* Initialize LEDSWD: Cannot be used during debug because it overrides SWDCLK pin configuration */
		BSP_LED_Init(LEDSWD);
		BSP_LED_Off(LEDSWD);
	}
#endif

	/* Initialize RTC */
	RTC_Config();
	RTC_TimeStampConfig();

	/* enable USB power on Pwrctrl CR2 register */
	HAL_PWREx_EnableVddUSB();

	if (SendOverUSB) /* Configure the USB */
	{
		/*** USB CDC Configuration ***/
		/* Init Device Library */
		USBD_Init(&USBD_Device, &VCP_Desc, 0);
		/* Add Supported Class */
		USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
		/* Add Interface callbacks for AUDIO and CDC Class */
		USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
		/* Start Device Process */
		USBD_Start(&USBD_Device);
	} else /* Configure the SDCard */
	{
		DATALOG_SD_Init();
	}
	HAL_Delay(200);

	/* Configure and disable all the Chip Select pins */
	Sensor_IO_SPI_CS_Init_All();

	/* Initialize and Enable the available sensors */
	initializeAllSensors();
	enableAllSensors();

	//---EMBEDDED ANN---

	float weights_initial[81] = { 0.680700, 0.324900, 0.607300, 0.365800, 0.693000,
			0.527200, 0.754400, 0.287800, 0.592300, 0.570900, 0.644000,
			0.416500, 0.249200, 0.704200, 0.598700, 0.250300, 0.632700,
			0.372900, 0.684000, 0.661200, 0.230300, 0.516900, 0.770900,
			0.315700, 0.756000, 0.293300, 0.509900, 0.627800, 0.781600,
			0.733500, 0.509700, 0.382600, 0.551200, 0.326700, 0.781000,
			0.563300, 0.297900, 0.714900, 0.257900, 0.682100, 0.596700,
			0.467200, 0.339300, 0.533600, 0.548500, 0.374500, 0.722800,
			0.209100, 0.619400, 0.635700, 0.300100, 0.715300, 0.670800,
			0.794400, 0.766800, 0.349000, 0.412400, 0.619600, 0.353000,
			0.690300, 0.772200, 0.666600, 0.254900, 0.402400, 0.780100,
			0.285300, 0.697700, 0.540800, 0.222800, 0.693300, 0.229800,
			0.698100, 0.463500, 0.201300, 0.786500, 0.581400, 0.706300,
			0.653600, 0.542500, 0.766900, 0.411500 };

	float dedw[81];
	float bias[15];
	unsigned int network_topology[3] = { 3, 9, 6 };
	float output[6];
	float x[3];
	float y[6];

	for (i = 0; i < 15; i++){
		bias[i] = 0.5;
	}
	for (i = 0; i < 6; i++){
		output[i] = 0.0;
	}
	for (i = 0; i < 81; i++){
		weights[i] = weights_initial[i];
		dedw[i] = 0.0;
	}

	ANN net;
	net.weights = weights;
	net.dedw = dedw;
	net.bias = bias;
	net.topology = network_topology;
	net.n_layers = 3;
	net.n_weights = 81;
	net.n_bias = 15;
	net.output = output;

	//OPTIONS
	net.eta = 0.05;     //Learning Rate
	net.beta = 0.01;    //Bias Learning Rate
	net.alpha = 0.25;   //Momentum Coefficient
	net.output_activation_function = &relu;
	net.hidden_activation_function = &relu;;

	/*
	 * Initialize neural network
	 */

	init_ann(&net);

	//---------------------

	/*
	 * XOR-XOR Input Values corresponding to output Ground Truth values
	 * The neural network includes 3 input neurons
	 *
	 */

	float x0[3] = { 0.0, 0.0, 0.0 };  // Corresponds to output 0 0
	float x1[3] = { 0.0, 1.0, 0.0 };  // Corresponds to output 1 1
	float x2[3] = { 0.0, 1.0, 1.0 };  // Corresponds to output 1 0
	float x3[3] = { 1.0, 1.0, 1.0 };  // Corresponds to output 0 0
	float x4[3] = { 0.0, 0.0, 1.0 };  // Corresponds to output 0 1
	float x5[3] = { 1.0, 0.0, 0.0 };  // Corresponds to output 1 0
	float x6[3] = { 1.0, 0.0, 1.0 };  // Corresponds to output 1 1
	float x7[3] = { 1.0, 1.0, 0.0 };  // Corresponds to output 0 1


	while ( 1 ){

		sprintf(epochOut, "\n\rEmbedded ML XOR-XOR System\n\r");
		CDC_Fill_Buffer((uint8_t *) epochOut, strlen(epochOut));

		sprintf(epochOut, "\n\rTrain and Test Start in 5 seconds");
		CDC_Fill_Buffer((uint8_t *) epochOut, strlen(epochOut));
		HAL_Delay(5000);

		/*
		 * Initialize weight, output, and bias values
		 */
		for (i = 0; i < 15; i++){
			bias[i] = 0.5;
		}
		for (i = 0; i < 6; i++){
			output[i] = 0.0;
		}
		for (i = 0; i < 81; i++){
			weights[i] = weights_initial[i];
			dedw[i] = 0.0;
		}

		/*
		 * Initialize neural network data structure
		 */

		net.weights = weights;
		net.dedw = dedw;
		net.bias = bias;
		net.output = output;

		init_ann(&net);

		/*
		 * Initiate train and test cycles
		 */

		for (i = 0; i < N_EPOCH; i++) {

			/*
			 * Compute input and output ground truth
			 */

			generate_xorxor(x, y, i);

			/*
			 * Train network on input and ground truth
			 */

			train_ann(&net, x, y);

			/*
			 * After completion of a number of epochs, N_REPORT
			 * perform neural network test execution, display
			 * resulting outputs and output error
			 */

			if (i % N_REPORT == 0 || i == 0) {
				sprintf(epochOut, "\n\rEpoch: %i", i);
				CDC_Fill_Buffer((uint8_t *) epochOut, strlen(epochOut));

				net_error = 0;

				/*
				 * Execute trained network and compute Output Error
				 * with Ground Truth supplied in ground_truth array
				 */

				run_ann(&net, x0);
				ground_truth[0] = 0.0;
				ground_truth[1] = 0.0;
				Output_Error(2, &net, ground_truth, &error);
				net_error = net_error + error;

				run_ann(&net, x1);
				ground_truth[0] = 1.0;
				ground_truth[1] = 1.0;
				Output_Error(2, &net, ground_truth, &error);
				net_error = net_error + error;

				run_ann(&net, x2);
				ground_truth[0] = 1.0;
				ground_truth[1] = 0.0;
				Output_Error(2, &net, ground_truth, &error);
				net_error = net_error + error;

				run_ann(&net, x3);
				ground_truth[0] = 0.0;
				ground_truth[1] = 0.0;
				Output_Error(2, &net, ground_truth, &error);
				net_error = net_error + error;

				run_ann(&net, x4);
				ground_truth[0] = 0.0;
				ground_truth[1] = 1.0;
				Output_Error(2, &net, ground_truth, &error);
				net_error = net_error + error;

				run_ann(&net, x5);
				ground_truth[0] = 1.0;
				ground_truth[1] = 0.0;
				Output_Error(2, &net, ground_truth, &error);
				net_error = net_error + error;

				run_ann(&net, x6);
				ground_truth[0] = 1.0;
				ground_truth[1] = 1.0;
				Output_Error(2, &net, ground_truth, &error);
				net_error = net_error + error;

				run_ann(&net, x7);
				ground_truth[0] = 0.0;
				ground_truth[1] = 1.0;
				Output_Error(2, &net, ground_truth, &error);
				net_error = net_error + error;

				floatToInt(net_error, &d1, &d2, 5);
				sprintf(Report_Message, "\r\nTotal Mean Squared Error: %d.%05d", (int)d1, (int)d2);
				CDC_Fill_Buffer((uint8_t *) Report_Message, strlen(Report_Message));
			}
		}

		sprintf(Report_Message, "\n\rTrain and Test Complete at Epoch: %d\n", i);
		CDC_Fill_Buffer((uint8_t *) Report_Message, strlen(Report_Message));
		/*
		 * Delay of 5 seconds to permit user to view results
		 */
		HAL_Delay(5000);
	}



	while (1) {
		// Go to Sleep
		__WFI();
	}

	/*
	 * Source code below is that of Datalog application.  Ths is retained
	 * but not required in the XOR-XOR neural network system
	 */

	while (1) {

		// Get sysTick value and check if it's time to execute the task
		msTick = HAL_GetTick();
		if (msTick % DATA_PERIOD_MS == 0 && msTickPrev != msTick) {
			msTickPrev = msTick;
			if (SendOverUSB) {
				BSP_LED_On(LED1);
			}

#ifdef NOT_DEBUGGING
			else if (SD_Log_Enabled)
			{
				BSP_LED_On(LEDSWD);
			}
#endif
			RTC_Handler(&RtcHandle);

			Accelero_Sensor_Handler(LSM6DSM_X_0_handle);

			Gyro_Sensor_Handler(LSM6DSM_G_0_handle);

			Magneto_Sensor_Handler(LSM303AGR_M_0_handle);

			Pressure_Sensor_Handler(LPS22HB_P_0_handle);

			if (!no_T_HTS221) {
				Temperature_Sensor_Handler(HTS221_T_0_handle);
			}
			if (!no_H_HTS221) {
				Humidity_Sensor_Handler(HTS221_H_0_handle);
			}

			if (!no_GG) {
				Gas_Gauge_Handler(GG_handle);
			}

			if (SD_Log_Enabled) // Write data to the file on the SDCard
			{
				DATALOG_SD_NewLine();
			}

			if (SendOverUSB) {
				BSP_LED_Off(LED1);
			}
#ifdef NOT_DEBUGGING
			else if (SD_Log_Enabled)
			{
				BSP_LED_Off(LEDSWD);
			}
#endif

		}

		// Check LSM6DSM Double Tap Event
		if (MEMSInterrupt) {
			MEMSInterrupt = 0;
			BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(LSM6DSM_X_0_handle,
					&doubleTap);
			if (doubleTap) { // Double Tap event

			}
		}

		// Go to Sleep
		__WFI();
	}
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors(void) {
	if (BSP_ACCELERO_Init(LSM6DSM_X_0, &LSM6DSM_X_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_GYRO_Init(LSM6DSM_G_0, &LSM6DSM_G_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_ACCELERO_Init(LSM303AGR_X_0, &LSM303AGR_X_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_MAGNETO_Init(LSM303AGR_M_0, &LSM303AGR_M_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_PRESSURE_Init(LPS22HB_P_0, &LPS22HB_P_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_TEMPERATURE_Init(LPS22HB_T_0, &LPS22HB_T_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_TEMPERATURE_Init(HTS221_T_0, &HTS221_T_0_handle)
			== COMPONENT_ERROR) {
		no_T_HTS221 = 1;
	}

	if (BSP_HUMIDITY_Init(HTS221_H_0, &HTS221_H_0_handle) == COMPONENT_ERROR) {
		no_H_HTS221 = 1;
	}

	/* Inialize the Gas Gauge if the battery is present */
	if (BSP_GG_Init(&GG_handle) == COMPONENT_ERROR) {
		no_GG = 1;
	}

	if (!SendOverUSB) {
		/* Enable HW Double Tap detection */
		BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(LSM6DSM_X_0_handle);
		BSP_ACCELERO_Set_Tap_Threshold_Ext(LSM6DSM_X_0_handle,
				LSM6DSM_TAP_THRESHOLD_MID);
	}

}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void enableAllSensors(void) {
	BSP_ACCELERO_Sensor_Enable(LSM6DSM_X_0_handle);
	BSP_GYRO_Sensor_Enable(LSM6DSM_G_0_handle);
	BSP_ACCELERO_Sensor_Enable(LSM303AGR_X_0_handle);
	BSP_MAGNETO_Sensor_Enable(LSM303AGR_M_0_handle);
	BSP_PRESSURE_Sensor_Enable(LPS22HB_P_0_handle);
	BSP_TEMPERATURE_Sensor_Enable(LPS22HB_T_0_handle);
	if (!no_T_HTS221) {
		BSP_TEMPERATURE_Sensor_Enable(HTS221_T_0_handle);
		BSP_HUMIDITY_Sensor_Enable(HTS221_H_0_handle);
	}

}

/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
void disableAllSensors(void) {
	BSP_ACCELERO_Sensor_Disable(LSM6DSM_X_0_handle);
	BSP_ACCELERO_Sensor_Disable(LSM303AGR_X_0_handle);
	BSP_GYRO_Sensor_Disable(LSM6DSM_G_0_handle);
	BSP_MAGNETO_Sensor_Disable(LSM303AGR_M_0_handle);
	BSP_HUMIDITY_Sensor_Disable(HTS221_H_0_handle);
	BSP_TEMPERATURE_Sensor_Disable(HTS221_T_0_handle);
	BSP_TEMPERATURE_Sensor_Disable(LPS22HB_T_0_handle);
	BSP_PRESSURE_Sensor_Disable(LPS22HB_P_0_handle);
}

/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void) {
	/*##-1- Configure the RTC peripheral #######################################*/
	RtcHandle.Instance = RTC;

	/* Configure RTC prescaler and RTC data registers */
	/* RTC configured as follow:
	 - Hour Format    = Format 12
	 - Asynch Prediv  = Value according to source clock
	 - Synch Prediv   = Value according to source clock
	 - OutPut         = Output Disable
	 - OutPutPolarity = High Polarity
	 - OutPutType     = Open Drain */
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
	RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
	RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
	RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

	if (HAL_RTC_Init(&RtcHandle) != HAL_OK) {

		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig(void) {

	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;

	/*##-3- Configure the Date using BCD format ################################*/
	/* Set Date: Monday January 1st 2000 */
	sdatestructure.Year = 0x00;
	sdatestructure.Month = RTC_MONTH_JANUARY;
	sdatestructure.Date = 0x01;
	sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

	if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK) {

		/* Initialization Error */
		Error_Handler();
	}

	/*##-4- Configure the Time using BCD format#################################*/
	/* Set Time: 00:00:00 */
	stimestructure.Hours = 0x00;
	stimestructure.Minutes = 0x00;
	stimestructure.Seconds = 0x00;
	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss) {

	RTC_TimeTypeDef stimestructure;

	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.Hours = hh;
	stimestructure.Minutes = mm;
	stimestructure.Seconds = ss;
	stimestructure.SubSeconds = 0;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	MEMSInterrupt = 1;
}

/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
static void Error_Handler(void) {

	while (1) {
	}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{

	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	while (1)
	{}
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
