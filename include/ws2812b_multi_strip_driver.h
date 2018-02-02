#ifndef _WS2812B_MULTI_STRIP_DRIVER_H
#define _WS2812B_MULTI_STRIP_DRIVER_H

#define MAX_SUPPORTED_NUM_OF_STRIPS (20)
#define MAX_SUPPORTED_LEDS_IN_STRIP (600)
#define NUM_OF_CFG_BYTES_PER_LED (3)
#define BITS_TO_CONFIGURE_ONE_LED (24)
#define MAX_LEDS_IN_STRIP (300)
#define BITS_IN_BYTE (8)
#define GPIOB_PORT 0
#define GPIOC_PORT 1
enum {
	GREEN_BYTE 	= 0,
	RED_BYTE 	= 1,
	BLUE_BYTE 	= 2
};

#define GET_STRIP_PORT(strip) (strip<10 ? GPIOB_PORT : GPIOC_PORT)

void drive_port_strips(void);
void update_driver_mask(int strip_idx);
void update_GPIO_all_strips_mask(uint16_t update_mask);
#endif  /* _WS2812B_MULTI_STRIP_DRIVER_H */
