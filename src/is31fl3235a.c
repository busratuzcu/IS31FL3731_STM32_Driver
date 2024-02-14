/****************************************************************************/
/*  Filename:  is31fl3235a.c                                                */
/*  Feature:   LED DRIVER                                                   */
/*	Module:    IS31FL3235A                                                  */
/*	Author:    Busra Tuzcu  busraatuzcu@gmail.com                           */
/****************************************************************************/

#include <is31fl3235a.h>

/*
 * Initialises the given handle.
 */
HAL_StatusTypeDef IS31FL3235A_Init(IS31FL3235A_HandleTypeDef* handle)
{
	HAL_StatusTypeDef status  = 0;
	status = IS31FL3235A_Reset(handle);
	status = IS31FL3235A_SetSoftwareShutdown(handle, IS31FL3235A_SOFTWARE_SHUTDOWN_DISABLED);
	IS31FL3235A_SetChipEnable(handle, IS31FL3235A_CHIP_ENABLED);
	status = IS31FL3235A_AllLEDControl(handle, 0x00, 0x00);
	return status;
}

/*
 * Changes the chip enable state of the specified chip.
 */
void IS31FL3235A_SetChipEnable(IS31FL3235A_HandleTypeDef* handle, uint8_t enable_state)
{
	HAL_GPIO_WritePin(handle->Init.Chip_Enable_Signal_Port, handle->Init.Chip_Enable_Signal_Pin, enable_state);
}

/*
 * Writes a given value to a given register.
 */
HAL_StatusTypeDef IS31FL3235A_WriteRegister(IS31FL3235A_HandleTypeDef* handle, uint8_t register_address, uint8_t value)
{
	HAL_StatusTypeDef status  = 0;
	uint8_t buf[2];
	buf[0] = register_address;
	buf[1] = value;
	status = HAL_I2C_Master_Transmit(handle->Init.I2C_Bus, handle->Init.I2C_Device_Address, buf, 2, handle->Init.I2C_Transmit_Timeout_Milliseconds);
	return status;
}

/*
 * Resets the specified chip. (Through I2C)
 */
HAL_StatusTypeDef IS31FL3235A_Reset(IS31FL3235A_HandleTypeDef* handle)
{
	HAL_StatusTypeDef status  = 0;
	status  = IS31FL3235A_WriteRegister(handle, IS31FL3235A_REGISTER_RESET, 0x00);
	return status;
}

/*
 * Latches the written PWM and led control values in the chip to the led control circuit.
 */
HAL_StatusTypeDef IS31FL3235A_Update(IS31FL3235A_HandleTypeDef* handle)
{
	HAL_StatusTypeDef status  = 0;
	status  = IS31FL3235A_WriteRegister(handle, IS31FL3235A_REGISTER_UPDATE, 0x00);
	return status;
}

/*
 * Sets the LED control for the specified chip / channel.
 */
HAL_StatusTypeDef IS31FL3235A_WriteLEDControl(IS31FL3235A_HandleTypeDef* handle, uint8_t channel,uint8_t current_set, uint8_t led_state)
{
	HAL_StatusTypeDef status  = 0;
	if (channel < IS31FL3235A_MAX_CHANNELS)
	{
		switch(led_state)
		{
			case IS31FL3235A_LED_STATE_OFF:
				IS31FL3235A_WritePWM(handle, channel, 0x00);
				break;

			case IS31FL3235A_LED_STATE_ON:
				IS31FL3235A_WritePWM(handle, channel, 0xFF);
				break;
			default:
				break;
		}
		status  = IS31FL3235A_WriteRegister(handle, IS31FL3235A_REGISTER_LED_CTRL + channel, (current_set | 0x01));
		status	= IS31FL3235A_Update(handle);
	}
	return status;
}

/*
 * Writes an led control word to all channels of a specified chip.
 */
HAL_StatusTypeDef IS31FL3235A_WriteGlobalLEDControl(IS31FL3235A_HandleTypeDef* handle, uint8_t current_set, uint8_t led_state)
{
	HAL_StatusTypeDef status  = 0;
	for (uint8_t i = 0; i < IS31FL3235A_MAX_CHANNELS; i++)
	{
		status  = IS31FL3235A_WriteLEDControl(handle, i,current_set, led_state);
	}
	return status;
}

/*
 * Writes the given PWM value (0 - 255) to the specified chip / channel.
 */
HAL_StatusTypeDef IS31FL3235A_WritePWM(IS31FL3235A_HandleTypeDef* handle, uint8_t channel, uint8_t pwm_value)
{
	HAL_StatusTypeDef status  = 0;
	if (channel < IS31FL3235A_MAX_CHANNELS)
		status  = IS31FL3235A_WriteRegister(handle, IS31FL3235A_REGISTER_PWM + channel, pwm_value);
	return status;
}

/*
 * Sets the software shutdown mode of the specified chip.
 */
HAL_StatusTypeDef IS31FL3235A_SetSoftwareShutdown(IS31FL3235A_HandleTypeDef* handle, uint8_t software_shutdown_mode)
{
	HAL_StatusTypeDef status  = 0;
	status  = IS31FL3235A_WriteRegister(handle, IS31FL3235A_REGISTER_SHUTDOWN, software_shutdown_mode);
	return status;
}

/*
 * Sets the LED control for the all chip / channel.
 */
HAL_StatusTypeDef IS31FL3235A_AllLEDControl(IS31FL3235A_HandleTypeDef* handle, uint8_t current_set, uint8_t led_state)
{
	HAL_StatusTypeDef status  = 0;
	for(uint8_t channel = 0x00; channel < 0x1C; channel++)
	{
		status  = IS31FL3235A_WriteLEDControl(handle, channel, current_set, led_state);
	}

	return status;
}
