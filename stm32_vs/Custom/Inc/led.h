
#ifndef INC_LED_H_
#define INC_LED_H_



typedef enum {
	GREEN = 12,
	ORANGE = 13,
	RED = 14,
	BLUE = 15
}led_color;


typedef enum {
	ON = 1,
	OFF = 0,
}led_stage;

void led_control(led_color led,led_stage stage);
void led_init();



#endif /* INC_LED_H_ */

