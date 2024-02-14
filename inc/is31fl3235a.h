/***************************************************************************/
/*	Filename: is31fl3235a.h                                                */
/*	Feature:  LED DRIVER                                                   */
/*	Module:	  IS31FL3235A                                                  */
/*	Author:	  Busra Tuzcu  busraatuzcu@gmail.com                           */
/***************************************************************************/

#ifndef _IS31FL3235A_H_
#define _IS31FL3235A_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stm32h7xx_hal.h> // Include ST's HAL library for the STM32. The correct library for the platform has to be chosen.

// Definitions of values for updating and changing settings of the IC. These values have been retrieved from the IS31FL3235A's datasheet.

#define IS31FL3235A_I2C_AD_TO_GND				      0x00
#define IS31FL3235A_I2C_AD_TO_SCL				      0x02
#define IS31FL3235A_I2C_AD_TO_SDA				      0x04
#define IS31FL3235A_I2C_AD_TO_VCC				      0x06
#define IS31FL3235A_GET_I2C_ADDR(AD_conn)		  (0x78 | AD_conn) // Retrieves the I2C address of the IS31FL3235A chip by specifying the connection of the AD pin.

#define IS31FL3235A_MAX_CHANNELS				      28 // The IS31FL3235A chip has 28 controllable PWM channels.

#define IS31FL3235A_REGISTER_SHUTDOWN			    0x00 // Shutdown register: The LSBit of this register determine-s if the chip is in shutdown. Write 0x00 for shutdown, 0x01 for normal operation.
#define IS31FL3235A_REGISTER_PWM				      0x05 // Add the PWM channel index (range from 0 to 27) to IS31FL3235A_REGISTER_PWM to address different channels; Write pwm value for channel in this register.
#define IS31FL3235A_REGISTER_LED_CTRL			    0x2a // LED control register: LED control state (on or off) and maximum current setting. Refer to datasheet for detailed information.
#define IS31FL3235A_REGISTER_UPDATE				    0x25 // Update register: Write 0x00 to this register to update PWM and LED control values.
#define IS31FL3235A_REGISTER_GLOBAL_CTRL		  0x4a // Global control register: The LSBit of this register sets all led states: 0 on, 1 off.
#define IS31FL3235A_REGISTER_OUTPUT_FREQUENCY	0X4b // Output frequency setting register: Set all channels operating frequency.
#define IS31FL3235A_REGISTER_RESET				    0x4f // Reset register: Write 0x00 to reset all registers of the chip.

#define IS31FL3235A_LED_CURRENT_MAX				    0x00 // LED current is set to maximum current specified by external resistor. (refer to datasheet) [RESET DEFAULT]
#define IS31FL3235A_LED_CURRENT_MAX_DIV_2		  0x02 // LED current is set to maximum current specified by external resistor divided by 2. (refer to datasheet)
#define IS31FL3235A_LED_CURRENT_MAX_DIV_3		  0x04 // LED current is set to maximum current specified by external resistor divided by 3. (refer to datasheet)
#define IS31FL3235A_LED_CURRENT_MAX_DIV_4		  0x06 // LED current is set to maximum current specified by external resistor divided by 4. (refer to datasheet)

#define IS31FL3235A_LED_STATE_OFF				      0x00 // LED is off. [RESET DEFAULT]
#define IS31FL3235A_LED_STATE_ON				      0x01 // LED is controlled by PWM registers.

#define IS31FL3235A_FREQUENCY_OUTPUT_3KHZ		  0X00 // Output frequency setting is 3kHz.
#define IS31FL3235A_FREQUENCY_OUTPUT_22KHZ		0X01 // Output frequency setting is 22kHz.

#define IS31FL3235A_SOFTWARE_SHUTDOWN_ENABLED	  0x00 // Chip software shutdown mode enabled. [RESET DEFAULT]
#define IS31FL3235A_SOFTWARE_SHUTDOWN_DISABLED	0x01 // Chip software shutdown mode disabled.

#define IS31FL3235A_CHIP_DISABLED				GPIO_PIN_RESET // Chip disabled state.
#define IS31FL3235A_CHIP_ENABLED				GPIO_PIN_SET // Chip enabled state.

// Initialization struct typedef.
typedef struct
{
	I2C_HandleTypeDef*	I2C_Bus;
	uint8_t 			      I2C_Device_Address;
	uint32_t 			      I2C_Transmit_Timeout_Milliseconds;
	GPIO_TypeDef* 		  Chip_Enable_Signal_Port;
	uint16_t 			      Chip_Enable_Signal_Pin;
} IS31FL3235A_InitTypeDef;

// Device-handle struct typedef.
typedef struct
{
	IS31FL3235A_InitTypeDef Init;
} IS31FL3235A_HandleTypeDef;

void IS31FL3235A_SetChipEnable(IS31FL3235A_HandleTypeDef* handle, uint8_t enable_state);

HAL_StatusTypeDef IS31FL3235A_Init(IS31FL3235A_HandleTypeDef* handle);
HAL_StatusTypeDef IS31FL3235A_WriteRegister(IS31FL3235A_HandleTypeDef* handle, uint8_t register_address, uint8_t value);
HAL_StatusTypeDef IS31FL3235A_Reset(IS31FL3235A_HandleTypeDef* handle);
HAL_StatusTypeDef IS31FL3235A_Update(IS31FL3235A_HandleTypeDef* handle);

HAL_StatusTypeDef IS31FL3235A_WriteLEDControl(IS31FL3235A_HandleTypeDef* handle, uint8_t channel,uint8_t current_set,uint8_t led_state);
HAL_StatusTypeDef IS31FL3235A_WriteGlobalLEDControl(IS31FL3235A_HandleTypeDef* handle,uint8_t current_set, uint8_t led_state);
HAL_StatusTypeDef IS31FL3235A_WritePWM(IS31FL3235A_HandleTypeDef* handle, uint8_t channel, uint8_t pwm_value);
HAL_StatusTypeDef IS31FL3235A_SetSoftwareShutdown(IS31FL3235A_HandleTypeDef* handle, uint8_t software_shutdown_mode);
HAL_StatusTypeDef IS31FL3235A_AllLEDControl(IS31FL3235A_HandleTypeDef* handle, uint8_t current_set, uint8_t led_state);


#ifdef __cplusplus
}
#endif

#endif /* _IS31FL3235A_H_ */
