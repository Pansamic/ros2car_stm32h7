/**
 * @file buzzer.h
 * @author pansamic (pansamic@foxmail.com)
 * @brief 
 * @version 0.1
 * @date 2025-03-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef __BUZZER_H__
#define __BUZZER_H__

#include <stdint.h>

void buzzer_on(void);
void buzzer_off(void);
void buzzer_set_frequency(uint16_t frequency);

#endif // __BUZZER_H__