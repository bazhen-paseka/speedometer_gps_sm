/**
* \version 1.0
* \author bazhen.levkovets
* \date 31 July 2020. Friday.
*************************************************************************************
* \copyright	Bazhen Levkovets
* \copyright	Brovary
* \copyright	Ukraine
*************************************************************************************
*/

/*
**************************************************************************
*							INCLUDE FILES
**************************************************************************
*/

	#include "speedometer_gps_sm.h"

/*
**************************************************************************
*							LOCAL DEFINES
**************************************************************************
*/

	#define RX_BUFFER_SIZE 			0xFF

/*
**************************************************************************
*							LOCAL CONSTANTS
**************************************************************************
*/


/*
**************************************************************************
*						    LOCAL DATA TYPES
**************************************************************************
*/


/*
**************************************************************************
*							  LOCAL TABLES
**************************************************************************
*/

/*
**************************************************************************
*								 MACRO'S
**************************************************************************
*/


/*
**************************************************************************
*						 LOCAL GLOBAL VARIABLES
**************************************************************************
*/
typedef struct	{
		UART_HandleTypeDef * uart_debug	;
	} Debug_struct 						;

	Debug_struct 			Debug_ch				= { 0 }	;

	#define	DEBUG_STRING_SIZE		300

	volatile uint32_t counter_u32 = 0;

	 tm1637_struct h1_tm1637 =
	  {
		 .clk_pin  = GPIO_PIN_14,
		 .clk_port = GPIOB,
		 .dio_pin  = GPIO_PIN_15,
		 .dio_port = GPIOB
	  };

	 volatile uint8_t IRQ_flag = 0;

		extern DMA_HandleTypeDef		hdma_usart2_rx ;
		RingBuffer_DMA 			rx_buffer	= { 0 }	;
		uint8_t		rx_circular_buffer[RX_BUFFER_SIZE] ;

		char gps_string[RX_BUFFER_SIZE] = { 0 } ;
		int gps_length_int = 0 ;
		HAL_StatusTypeDef	status_res = {0} ;

/*
**************************************************************************
*                        LOCAL FUNCTION PROTOTYPES
**************************************************************************
*/

/*
**************************************************************************
*                           GLOBAL FUNCTIONS
**************************************************************************
*/

void Speedometer_GPS_Init (void) {
	Debug_init(&huart1);

	char debugString[DEBUG_STRING_SIZE];
	sprintf(debugString," START\r\n GPS-speedometer; tm1637;\r\n") ;
	Debug_print( debugString ) ;

	int		soft_version_arr_int[3] = {0} ;
	soft_version_arr_int[0] 	= ((SOFT_VERSION) / 100) %10 ;
	soft_version_arr_int[1] 	= ((SOFT_VERSION) /  10) %10 ;
	soft_version_arr_int[2] 	= ((SOFT_VERSION)      ) %10 ;
	sprintf (	debugString						,
			" 2020-June-31 v%d.%d.%d. \r\n"	,
			soft_version_arr_int[0]			,
			soft_version_arr_int[1]			,
			soft_version_arr_int[2]			) ;
	Debug_print( debugString ) ;

	tm1637_Init(&h1_tm1637);
	tm1637_Set_Brightness(&h1_tm1637, bright_45percent);
	tm1637_Display_Decimal(&h1_tm1637, 8765, double_dot);

	RingBuffer_DMA_Init (	&rx_buffer			,
							&hdma_usart2_rx		,
							rx_circular_buffer	,
							RX_BUFFER_SIZE		) ;  	// Start UART receive

	status_res = HAL_UART_Receive_DMA(&huart2, rx_circular_buffer, RX_BUFFER_SIZE ) ;  	// how many bytes in buffer
	sprintf (	debugString						,
				"UART_DMA status:%d\r\n"	,
				(int)	status_res	 					) ;
	Debug_print( debugString ) ;

	HAL_Delay(1000);
}
//***************************************************************************

void Speedometer_GPS_Main (void) {
	char debugString[DEBUG_STRING_SIZE] ;
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin) ;

	uint32_t speed_u32 = counter_u32++;

//	sprintf(debugString,"%04d\r\n", (int)speed_u32) ;
//	Debug_print( debugString ) ;

	tm1637_Display_Decimal(&h1_tm1637, speed_u32, no_double_dot) ;

	uint32_t 	rx_count = RingBuffer_DMA_Count ( &rx_buffer ) ;
	sprintf(debugString,"RingBuffer_DMA_Count = %d\r\n", (int)rx_count) ;
	Debug_print( debugString ) ;

	while ( rx_count-- ) {
		gps_string[gps_length_int] = RingBuffer_DMA_GetByte ( &rx_buffer ) ;
		gps_length_int++ ;
		if ( gps_length_int > 100 ) {
			return ;
		}
	}

	if ( rx_count > 2 ) {
		for (int i=0; i<gps_length_int; i++ ) {
			sprintf(debugString,"%c", gps_string[i]) ;
			Debug_print( debugString ) ;
		}
		gps_length_int = 0;

		sprintf(debugString,"\r\n") ;
		Debug_print( debugString ) ;
	}

	HAL_Delay(1000);
}
//***************************************************************************

//***********************************************************

void Debug_print( char * _string ) {
	size_t len_szt = strlen( _string );
	if (len_szt > DEBUG_STRING_SIZE) {
		len_szt = DEBUG_STRING_SIZE ;
	}
	HAL_UART_Transmit(	Debug_ch.uart_debug		,
						(uint8_t * ) _string	,
						len_szt					,
						100						) ;
}
//***********************************************************

void Debug_init( UART_HandleTypeDef * _huart ) {
	Debug_ch.uart_debug = _huart ;
}
//*********************************************************

/*
**************************************************************************
*                           LOCAL FUNCTIONS
**************************************************************************
*/
