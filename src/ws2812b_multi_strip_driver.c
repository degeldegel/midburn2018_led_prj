#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "ws2812b_multi_strip_driver.h"

uint16_t GPIO_strips_mask[MAX_SUPPORTED_LEDS_IN_STRIP * BITS_TO_CONFIGURE_ONE_LED]; /*contains masks of all bits that are zero*/
uint16_t GPIO_all_strips_mask;
uint8_t  LED_strips[MAX_SUPPORTED_NUM_OF_STRIPS][MAX_SUPPORTED_LEDS_IN_STRIP][NUM_OF_CFG_BYTES_PER_LED];

void drive_port_strips(void)
{
	volatile int idx;
	int i, curr_led_bit_idx;
	uint16_t curr_zero_mask;

	for (curr_led_bit_idx=0; curr_led_bit_idx < MAX_LEDS_IN_STRIP * BITS_TO_CONFIGURE_ONE_LED; curr_led_bit_idx++)
	{
		//raise all Strips up
		GPIOB->ODR |=  GPIO_all_strips_mask;
		//wait for strips with bit value zero
		for (i=0; i < 10; i++) {idx=i;}
		//lower all strips with bit value zero
		curr_zero_mask = GPIO_strips_mask[curr_led_bit_idx];
		GPIOB->ODR &= ~curr_zero_mask;
		//wait for strips with bit value one
		for (i=0; i < 18; i++) {idx=i;}
		//lower all strips with bit value zero
		GPIOB->ODR &= ~(curr_zero_mask ^ GPIO_all_strips_mask);
		//finish bit configuration cycle ~1.25 msec
		for (i=0; i < 9; i++) {idx=i;}
	}
}

void update_GPIO_all_strips_mask(uint16_t update_mask)
{
	GPIO_all_strips_mask = update_mask;
}

void zero_all_driver_masks(void)
{
	int curr_led_bit_idx;
	for (curr_led_bit_idx=0; curr_led_bit_idx < MAX_LEDS_IN_STRIP * BITS_TO_CONFIGURE_ONE_LED; curr_led_bit_idx++)
	{
		GPIO_strips_mask[curr_led_bit_idx] = 0;
	}
}

void update_driver_mask(int strip_idx)
{
	//TODO - need to add hash table to convert LED_STRIP index to actual GPIO port and number
	//Right now prepared for one strip.
	int led_idx, rgb_idx;
	zero_all_driver_masks();
	for (led_idx=0; led_idx < MAX_LEDS_IN_STRIP; led_idx ++)
	{
		for (rgb_idx=0; rgb_idx < NUM_OF_CFG_BYTES_PER_LED; rgb_idx ++)
		{
			uint8_t cur_rgb_led;
			cur_rgb_led = LED_strips[strip_idx][led_idx][rgb_idx];
			int base_bit_idx = led_idx*BITS_TO_CONFIGURE_ONE_LED + rgb_idx*BITS_IN_BYTE;
			GPIO_strips_mask[base_bit_idx + 0] |= ((cur_rgb_led>>0) & 0x1) ? 0 : 1 << 10;
			GPIO_strips_mask[base_bit_idx + 1] |= ((cur_rgb_led>>1) & 0x1) ? 0 : 1 << 10;
			GPIO_strips_mask[base_bit_idx + 2] |= ((cur_rgb_led>>2) & 0x1) ? 0 : 1 << 10;
			GPIO_strips_mask[base_bit_idx + 3] |= ((cur_rgb_led>>3) & 0x1) ? 0 : 1 << 10;
			GPIO_strips_mask[base_bit_idx + 4] |= ((cur_rgb_led>>4) & 0x1) ? 0 : 1 << 10;
			GPIO_strips_mask[base_bit_idx + 5] |= ((cur_rgb_led>>5) & 0x1) ? 0 : 1 << 10;
			GPIO_strips_mask[base_bit_idx + 6] |= ((cur_rgb_led>>6) & 0x1) ? 0 : 1 << 10;
			GPIO_strips_mask[base_bit_idx + 7] |= ((cur_rgb_led>>7) & 0x1) ? 0 : 1 << 10;
		}
	}
}

