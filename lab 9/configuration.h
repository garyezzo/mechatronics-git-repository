

#ifndef configuration_H
#define	configuration_H

    #define f_clk  16000000UL
    #define baud_rate 19200UL

    #define line_terminator 0x0A      // ASCII Line Feed

    #define LED_PIN 13
    #define motor_direction 10
    #define motor_PWM 11
    
    #define mot_feedback A0
    
    #define Kp 2
    #define Ki 4/100
    #define Kd 5

#endif	
