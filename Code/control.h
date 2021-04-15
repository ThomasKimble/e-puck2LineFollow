#ifndef CONTROL_H
#define CONTROL_H

//start the speed regulator thread
void speed_regulator_start(void);
void set_intersect(void);
void clr_intersect(void);
uint8_t get_intersect(void);
void rotate_left(void);
void rotate_right(void);
void skip_stop(void);
void stay_stop(void);
void u_turn(void);
uint16_t get_speed(void);
void turn_signal(uint8_t side);

#endif /* CONTROL_H */
