#ifndef __SERIAL_H_
#define __SERIAL_H_
#include <stdbool.h>
#include <stdint.h>
#define SIZE 	18
extern bool	g_bDataAvailable;
extern uint8_t g_data[6];
extern uint8_t g_cmd[1];
void writeCom(uint8_t* cmd, uint8_t* data);
#endif
