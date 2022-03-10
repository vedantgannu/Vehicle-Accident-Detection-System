#include "init.h"
#include "usbh_conf.h"
#include "usbh_core.h"
#include "ff_gen_drv.h"
#include "usbh_diskio.h"
#include "GPS.h"
#include "GPSConfig.h"

ADC_HandleTypeDef hadc1; //ADC Handle
DMA_HandleTypeDef hdma1; //DMA Handle

int32_t rawValue[3]; //Array to store ADC channel measurements
int conv_complete = 0;//Flag to indicate if ADC conversion is complete

int32_t xsample=0; //Calibrated x value
int32_t ysample=0; //Calibrated y value
int32_t zsample=0; //Calibrated z value

int32_t x = 0; //Measure x direction acceleration
int32_t y = 0; //Measure y direction acceleration
int32_t z = 0; //Measure z direction acceleration

char crash_output[100]; //File write buffer

#define samples 200 //ADC samples taken to calibrate
#define minVal -500 //Threshold for Accident
#define MaxVal 500  //Threshold for Accident


FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
FIL MyFile;                   /* File object */
char USBDISKPath[4];          /* USB Host logical drive path */
USBH_HandleTypeDef husbh;     /* USB Host handle */

typedef enum {
	APPLICATION_IDLE = 0,
	APPLICATION_START,
	APPLICATION_RUNNING,
}MSC_ApplicationTypeDef;

MSC_ApplicationTypeDef AppliState = APPLICATION_IDLE;

//Private function prototypes -----------------------------------------------
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
static void MSC_Application(char* crash_output);
void configureADC();

int main(void) {

  Sys_Init();

  printf("\033[2J\033[;H"); // Erase screen & move cursor to home position
  fflush(stdout); // Need to flush stdout after using printf that doesn't end in \n

  if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)//Link the USB Host disk I/O driver
  {
      USBH_Init(&husbh, USBH_UserProcess, 0);//Init Host Library
      USBH_RegisterClass(&husbh, USBH_MSC_CLASS);//Add Supported Class
      USBH_Start(&husbh);//Start Host Process
  }

  configureADC(); //Configure ADC

  HAL_ADC_Start_DMA(&hadc1, (int32_t *)rawValue, 3); //Start reading ADC
  HAL_Delay(1000);
  HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);//Set USART6 Priority
  HAL_NVIC_EnableIRQ(USART6_IRQn);//Enable USART6 NVIC IRQ vector
  //TX6: PC6, RX6: PC7
  initUart(&_GPS_USART, (uint32_t) 9600, USART6);//Initialize the UART6's USART6 configurations for GPS unit.

  //Sampling the Accelerometer first
  for(int i=0;i<samples;i++)
  {
	  while(!conv_complete);
      xsample+=rawValue[0];
      ysample+=rawValue[1];
      zsample+=rawValue[2];
      conv_complete = 0;
  }
  xsample /= samples;
  ysample /= samples;
  zsample /= samples;

  //Initialize the GPS
  GPS_Init();
  //Ensure that program starts with a valid GPS location saved to GPS struct
  while (GPS.GPRMC.status == 'V'){
	  GPS_Process();
  }
  printf("GPS Initialized\r\n");
  HAL_Delay(1000);


  printf("\033[2J\033[;H"); // Erase screen & move cursor to home position
  printf("\033[0;0GPS"); //print on row 0
  printf("\033[11;0H"); //go to row 11
  printf("Accelerometer Readings\r\n");
  printf("\033[s"); //save spot
  printf("\033[12;24r");//enable scrolling lines 12 to 24

  //Accident detection
  while(1)
  {
	  while(!conv_complete){//While conversion is happening, try to get a location update
		  GPS_Process();
	  }
	  x=xsample-rawValue[0];
	  y=ysample-rawValue[1];
	  z=zsample-rawValue[2];

	  printf("\033[uX:%ld Y:%ld Z:%ld\r\n",x,y,z);
	  printf("\033[s");//Save cursor position
	  printf("\033[1;H\"%d:%d:%d (UTC)\", \"%d-%d-%d (UTC)\", \"%f %c\", \"%f %c\"\r\n",GPS.GPRMC.UTC_Hour, GPS.GPRMC.UTC_Min, GPS.GPRMC.UTC_Sec, GPS.GPRMC.UTC_Month, GPS.GPRMC.UTC_Day, GPS.GPRMC.UTC_Year, GPS.GPRMC.LatitudeDecimal, GPS.GPRMC.NS_Indicator, GPS.GPRMC.LongitudeDecimal, GPS.GPRMC.EW_Indicator);

	  //If accident is detected, break out of loop
	  if((x<minVal)||(x>MaxVal)||(y<minVal)||(y>MaxVal)||(z<minVal)||(z>MaxVal))
	  {
		  printf("\033[2J\033[;H"); // Erase screen & move cursor to home position
		  printf("ACCIDENT DETECTED\r\n");//Indicate to user that an accident has occurred

		  //Print GPS coordinates of the crash
		  printf("\"%d:%d:%d (UTC)\", \"%d-%d-%d (UTC)\", \"%f %c\", \"%f %c\"\r\n",GPS.GPRMC.UTC_Hour, GPS.GPRMC.UTC_Min, GPS.GPRMC.UTC_Sec, GPS.GPRMC.UTC_Month, GPS.GPRMC.UTC_Day, GPS.GPRMC.UTC_Year, GPS.GPRMC.LatitudeDecimal, GPS.GPRMC.NS_Indicator, GPS.GPRMC.LongitudeDecimal, GPS.GPRMC.EW_Indicator);
		  sprintf(crash_output, "\"%d:%d:%d (UTC)\", \"%d-%d-%d (UTC)\", \"%f %c\", \"%f %c\"",GPS.GPRMC.UTC_Hour, GPS.GPRMC.UTC_Min, GPS.GPRMC.UTC_Sec, GPS.GPRMC.UTC_Month, GPS.GPRMC.UTC_Day, GPS.GPRMC.UTC_Year, GPS.GPRMC.LatitudeDecimal, GPS.GPRMC.NS_Indicator, GPS.GPRMC.LongitudeDecimal, GPS.GPRMC.EW_Indicator);


		  //Run Application (Blocking mode)
		  while (1)
		  {
			  //USB Host Background task
			  USBH_Process(&husbh);

			  //Mass Storage Application State Machine
			  switch(AppliState)
			  {
			  	  case APPLICATION_START:
			  		  MSC_Application(crash_output);
			  		  AppliState = APPLICATION_IDLE;
			  		  break;

			  	  case APPLICATION_IDLE:
			  	  default:
			  		  break;
			  }
		  }

		  break;

	  }

	  conv_complete = 0; //Reset Flag
  }
}

void configureADC() {
  __HAL_RCC_ADC1_CLK_ENABLE(); //Enable ADC1 Clock.

  //Configure ADC
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  HAL_ADC_Init(&hadc1);

  ADC_ChannelConfTypeDef sConfig;

  //PA6 -> ADC1_IN6
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  //PA4 -> ADC1_IN4
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  //PC2 -> ADC1_IN12
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}


void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {

	__HAL_RCC_GPIOC_CLK_ENABLE(); //Enabling GPIOC clock
	__HAL_RCC_GPIOA_CLK_ENABLE(); //Enabling GPIOA clock

	//PA6 initialization
	GPIO_InitTypeDef  A0;
	A0.Pin = GPIO_PIN_6;
	A0.Mode = GPIO_MODE_ANALOG;
	A0.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &A0);//Initializing port PA6

	//PA4 initialization
	GPIO_InitTypeDef  A1;
	A1.Pin = GPIO_PIN_4;
	A1.Mode = GPIO_MODE_ANALOG;
	A1.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &A1);//Initializing port PA4

	//PC2 initialization
	GPIO_InitTypeDef  A2;
	A2.Pin = GPIO_PIN_2;
	A2.Mode = GPIO_MODE_ANALOG;
	A2.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &A2);//Initializing port PC2

	/* Peripheral clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();

	/* Peripheral DMA init*/
	hdma1.Instance = DMA2_Stream0;
	hdma1.Init.Channel = DMA_CHANNEL_0;
	hdma1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma1.Init.MemInc = DMA_MINC_ENABLE;
	hdma1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma1.Init.Mode = DMA_CIRCULAR;
	hdma1.Init.Priority = DMA_PRIORITY_LOW;
	HAL_DMA_Init(&hdma1);

	//Link DMA to ADC
	__HAL_LINKDMA(hadc,DMA_Handle,hdma1);

	//Enable NVICs
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

//DMA Stream IRQ
void DMA2_Stream0_IRQHandler()
{
	conv_complete = 1;
	HAL_DMA_IRQHandler(&hdma1);
}

//File Operations
static void MSC_Application(char* crash_output)
{
	FRESULT res;                             /* FatFs function common result code */
	uint32_t byteswritten;                     /* File write/read counts */
	uint8_t wtext[] = "ACCIDENT DETECTED \r\n" ; /* File write buffer */
	uint8_t* output = (uint8_t *)crash_output;/* File write buffer with crash coordinates */

	while(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK);/* Register the file system object to the FatFs module */

	printf("Creating report file\r\n");
	while(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK); /* Create and Open a new text file object with write access */

	printf("Starting to write report\r\n");
	res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);//Write data to the text file
	while((byteswritten == 0) || (res != FR_OK));  //'STM32.TXT' file Write or EOF Error
	res = f_write(&MyFile, output, strlen(output)*sizeof(uint8_t), (void *)&byteswritten); //Write data to the text file
	while((byteswritten == 0) || (res != FR_OK));  //'STM32.TXT' file Write or EOF Error

	printf("Report Written\r\n");

	f_close(&MyFile);/* Close the open text file */

    FATFS_UnLinkDriver(USBDISKPath);/* Unlink the USB disk I/O driver */
}

//Determines if a USB device is connected
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
	switch(id)
	{
		case HOST_USER_SELECT_CONFIGURATION:
			break;

		case HOST_USER_DISCONNECTION:
			AppliState = APPLICATION_IDLE;
			f_mount(NULL, (TCHAR const*)"", 0);
			break;

		case HOST_USER_CLASS_ACTIVE:
			AppliState = APPLICATION_START;
			break;

		default:
			break;
	}
}

void USART6_IRQHandler() {
	HAL_UART_IRQHandler(&_GPS_USART);
}

//Callback function for handling character reception on the RX lines of USART6
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart){
	if (huart->Instance == USART6){
		GPS_CallBack();
	}
}
