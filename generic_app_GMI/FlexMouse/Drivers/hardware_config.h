/**
  ********************************************************************************************************************************
  * @file    hardware_config.h 
  * @author  Satya Akkina
  * @brief   Header of port and pin assignments
  * @details This file is used to configure ports and pins based on hardware
  ********************************************************************************************************************************
  */

#define HARDWARE_VERSION_BULLRUNNER 0
#define HARDWARE_VERSION_1p3KW 1
#define HARDWARE_VERSION_4KW 2
#define HARDWARE_VERSION_SYMAX_SRI 30

#define HARDWARE_VERSION HARDWARE_VERSION_SYMAX_SRI // set this to choose the HW verison

// ADC inputs ports/pins
#define ANALOG_0_10V_Pin LL_GPIO_PIN_0
#define ANALOG_0_10V_GPIO_Port GPIOA

#if HARDWARE_VERSION == HARDWARE_VERSION_SYMAX_SRI
#define ANALOG_4_20MA_Pin LL_GPIO_PIN_0
#else
#define ANALOG_4_20MA_Pin LL_GPIO_PIN_1
#endif
#define ANALOG_4_20MA_GPIO_Port GPIOA

/****
// GPIO Input Ports/Pins
#if HARDWARE_VERSION == HARDWARE_VERSION_1p3KW
#define DIN1_Pin LL_GPIO_PIN_0
#define DIN1_GPIO_Port GPIOB
#define DIN2_Pin LL_GPIO_PIN_1
#define DIN2_GPIO_Port GPIOB
#define DIN3_Pin LL_GPIO_PIN_2
#define DIN3_GPIO_Port GPIOB

#define PWM_IN_Pin LL_GPIO_PIN_8
#define PWM_IN_GPIO_Port GPIOA
#define PWM_IN_EXTI_CONFIG_Port LL_EXTI_CONFIG_PORTA
#define PWN_IN_EXTI_CONFIG_Line LL_EXTI_CONFIG_LINE8
#define PWM_IN_EXTI_Line LL_EXTI_LINE_8
#endif
****/

/****
#if HARDWARE_VERSION == HARDWARE_VERSION_4KW
#define DIN1_Pin LL_GPIO_PIN_4
#define DIN1_GPIO_Port GPIOA
#define DIN2_Pin LL_GPIO_PIN_5
#define DIN2_GPIO_Port GPIOA
#define DIN3_Pin LL_GPIO_PIN_6
#define DIN3_GPIO_Port GPIOA

#define PWM_IN_Pin LL_GPIO_PIN_8
#define PWM_IN_GPIO_Port GPIOA
#define PWM_IN_EXTI_CONFIG_Port LL_EXTI_CONFIG_PORTA
#define PWN_IN_EXTI_CONFIG_Line LL_EXTI_CONFIG_LINE8
#define PWM_IN_EXTI_Line LL_EXTI_LINE_8
#endif
****/

#if HARDWARE_VERSION == HARDWARE_VERSION_SYMAX_SRI
#define DIN1_Pin 			LL_GPIO_PIN_15
#define DIN1_GPIO_Port 		GPIOC
#define DIN2_Pin 			LL_GPIO_PIN_14
#define DIN2_GPIO_Port 		GPIOC
#define DIN3_Pin 			LL_GPIO_PIN_9
#define DIN3_GPIO_Port 		GPIOB
#define DIN4_Pin 			LL_GPIO_PIN_8
#define DIN4_GPIO_Port 		GPIOA
#define DIR_ROT_Pin 		LL_GPIO_PIN_3
#define DIR_ROT_GPIO_Port 	GPIOB

#define PWM_IN_Pin LL_GPIO_PIN_8
#define PWM_IN_GPIO_Port GPIOA
#define PWM_IN_EXTI_CONFIG_Port LL_EXTI_CONFIG_PORTA
#define PWN_IN_EXTI_CONFIG_Line LL_EXTI_CONFIG_LINE8
#define PWM_IN_EXTI_Line LL_EXTI_LINE_8

/*#define PWM_IN_Pin 				LL_GPIO_PIN_6
#define PWM_IN_GPIO_Port 		GPIOB
#define PWM_IN_EXTI_CONFIG_Port LL_EXTI_CONFIG_PORTB
#define PWN_IN_EXTI_CONFIG_Line LL_EXTI_CONFIG_LINE6
#define PWM_IN_EXTI_Line 		LL_EXTI_LINE_6*/

#endif

/****
// GPIO Output Ports/Pins
#if HARDWARE_VERSION == HARDWARE_VERSION_1p3KW
#define DOUT1_Pin LL_GPIO_PIN_9
#define DOUT1_GPIO_Port GPIOB
#define DOUT1_IOP_GRP1_PERIPH_Port LL_IOP_GRP1_PERIPH_GPIOB
#define DOUT1_LL_GPIO_AF LL_GPIO_AF_2

#define LED_OUT_Pin LL_GPIO_PIN_6
#define LED_OUT_GPIO_Port GPIOB
#define RELAY_OUT_Pin LL_GPIO_PIN_7
#define RELAY_OUT_GPIO_Port GPIOB
#endif
****/

/****
#if HARDWARE_VERSION == HARDWARE_VERSION_4KW
#define DOUT1_Pin LL_GPIO_PIN_7
#define DOUT1_GPIO_Port GPIOA
#define DOUT1_IOP_GRP1_PERIPH_Port LL_IOP_GRP1_PERIPH_GPIOA
#define DOUT1_LL_GPIO_AF LL_GPIO_AF_5
#define LED_OUT_Pin LL_GPIO_PIN_0
#define LED_OUT_GPIO_Port GPIOB
#define RELAY_OUT_Pin LL_GPIO_PIN_9
#define RELAY_OUT_GPIO_Port GPIOB
#endif
****/

#if HARDWARE_VERSION == HARDWARE_VERSION_SYMAX_SRI
#define DOUT1_Pin 					LL_GPIO_PIN_5
#define DOUT1_GPIO_Port 			GPIOB
#define DOUT1_IOP_GRP1_PERIPH_Port 	LL_IOP_GRP1_PERIPH_GPIOB
#define DOUT1_LL_GPIO_AF 			GPIO_AF1_TIM3//LL_GPIO_AF_2

#define LED_OUT_Pin 				LL_GPIO_PIN_1
#define LED_OUT_GPIO_Port 			GPIOB
#define RELAY_OUT_Pin 				LL_GPIO_PIN_4
#define RELAY_OUT_GPIO_Port 		GPIOB
#endif

#define LED_ONBOARD_Pin 			LL_GPIO_PIN_0
#define LED_ONBOARD_GPIO_Port 		GPIOB

// USART ports/pins
#define USART_TX_Pin 				LL_GPIO_PIN_2
#define USART_TX_GPIO_Port 			GPIOA
#define USART_RX_Pin 				LL_GPIO_PIN_3
#define USART_RX_GPIO_Port 			GPIOA
#define USART_GPIO_AF 				LL_GPIO_AF_1
#define USART_IOP_GRP1_PERIPH_PORT 	LL_IOP_GRP1_PERIPH_GPIOA

// MODBUS ports/pins
#define MODBUS_TX_Pin 				LL_GPIO_PIN_9
#define MODBUS_TX_GPIO_Port 		GPIOA
#define MODBUS_RX_Pin 				LL_GPIO_PIN_10
#define MODBUS_RX_GPIO_Port 		GPIOA
#define MODBUS_EN_Pin 				LL_GPIO_PIN_12
#define MODBUS_EN_GPIO_Port 		GPIOA
#define MODBUS_GPIO_Port 			GPIOA
#define MODBUS_LL_GPIO_AF 			LL_GPIO_AF_1
#define MODBUS_IOP_GRP1_PERIPH_Port LL_IOP_GRP1_PERIPH_GPIOA

/****
// SPI ports/pins
#if HARDWARE_VERSION == HARDWARE_VERSION_1p3KW
#define SPI1_MISO_Pin LL_GPIO_PIN_6
#define SPI1_MISO_Port GPIOA
#define SPI1_MOSI_Pin LL_GPIO_PIN_7
#define SPI1_MOSI_Port GPIOA
#endif
****/

/****
#if HARDWARE_VERSION == HARDWARE_VERSION_4KW
#define SPI1_MISO_Pin LL_GPIO_PIN_4
#define SPI1_MISO_Port GPIOB
#define SPI1_MOSI_Pin LL_GPIO_PIN_5
#define SPI1_MOSI_Port GPIOB
#endif
****/

// SPI ports/pins
#if HARDWARE_VERSION == HARDWARE_VERSION_SYMAX_SRI
#define SPI1_MISO_Pin 		LL_GPIO_PIN_6
#define SPI1_MISO_Port 		GPIOA
#define SPI1_MOSI_Pin 		LL_GPIO_PIN_7
#define SPI1_MOSI_Port 		GPIOA
#endif

#define SPI1_GPIO_AF 		LL_GPIO_AF_0
#define SPI1_SCK_Pin 		LL_GPIO_PIN_3
#define SPI1_SCK_Port 		GPIOB
