/*
 * Arduino PID template.
 *
 * Copyright 2013 Aaron P. Dahlen
 *
 *     APDahlen@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

// AVR Libc includes
    #include <avr/io.h>
    #include <avr/interrupt.h>
    #include <stdint.h>
    #include <string.h>

// Project specific includes
    #include "configuration.h"
    #include "USART.c"

// Global variables

    volatile uint8_t start_flag = 0;

    float P;
    float I;
    int16_t D;
    
    int32_t accumulator;
    int16_t error;
    int16_t feedback;
    int16_t plant_drive;
    
void setup(){

    pinMode(LED_PIN, OUTPUT);    
    pinMode(motor_direction, OUTPUT);
    pinMode(motor_PWM, OUTPUT);
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS22);                                                //FIXME: the motor hum is annoying - change PWM prescaler...
    OCR2A = 0;
   
    init_timer_1_CTC(100);          // Enable the ISR
    init_USART( );                  // Enable USART ISR
}



/*********************************************************************************
 *  ______  ____   _____   ______  _____  _____    ____   _    _  _   _  _____
 * |  ____|/ __ \ |  __ \ |  ____|/ ____||  __ \  / __ \ | |  | || \ | ||  __ \
 * | |__  | |  | || |__) || |__  | |  __ | |__) || |  | || |  | ||  \| || |  | |
 * |  __| | |  | ||  _  / |  __| | | |_ ||  _  / | |  | || |  | || . ` || |  | |
 * | |    | |__| || | \ \ | |____| |__| || | \ \ | |__| || |__| || |\  || |__| |
 * |_|     \____/ |_|  \_\|______|\_____||_|  \_\ \____/  \____/ |_| \_||_____/
 *
 ********************************************************************************/

ISR(USART_RX_vect){
/*
 * @brief This Interrupt Service Routine is called when a new character is received 
 * on the serial port.  As quickly as possible, the AVR transfers the character to 
 * a circular buffer.  The main loop code then retrieves data from this buffer.  
 * Observe that this mechanism allows data to be received at a high rate of speed 
 * independent of the main loop.
 * 
 * @todo This function should be in the USART.c file but there is a error 
 * "multiple definition of `  vector_18".  Apparently Arduino detects when an ISR
 * is in the main sketch.  If you place it somewhere else it is missed and replaced 
 * with the Arduino handler.  This is the source of the multiple definitions
 * error -  * see discussion @ http://forum.arduino.cc/index.php?topic=42153.0
 * Note that the three variables used in this function are all global is scope.  
 * This was done so that this function could be included in the projects main page.
 * 
 * @note The vector name is not "USART_RXC" as indicated in the data sheet.  
 * Instead, the ATMega328p "iom328p.h" file identified the name as "USART_RX_vect".

 * @note From the ATMEL data sheet "When interrupt driven data reception is used, the 
 * receive complete routine must read the received data from UDRn in order to
 * clear the RXCn Flag, otherwise a new interrupt will occur once the interrupt
 * routine terminates.
 *
 */

   ISR_main_link_circ_bufer[circ_buf_head] = UDR0;
   circ_buf_head++;
   circ_buf_head &= 0b00011111;
}



ISR(TIMER1_COMPA_vect){

/* @brief This Interrupt Service Routine (ISR) serves as the 100 Hz heartbeat 
 * for the Arduino.  See the companion function init_timer_1_CTC for additional 
 * information.
 *
 * @ Note:
 *    1) Compiler generated code pushes status register and any used registers to stack.
 *    2) Calling a subroutine from the ISR causes compiler to save all 32 registers; a 
 *       slow operation (fact check). 
 *    3) Status and used registers are popped by compiler generated code.
 */

    static uint8_t cnt;
    static uint16_t time_in_state = 0;
    static uint16_t setpoint = 0;
    static uint8_t state = 0;
    
    int16_t PWM_DC;
    static int16_t last_feedback;
    
    char buf[50];

    digitalWrite(LED_PIN, HIGH);        // Connect oscilloscope to verify cooperative scheduling is functioning correctly.
                                        // Which is to say, the LED should not be on with a 100% duty cycle...
    
    D = analogRead(A0) - feedback;
    //D = (analogRead(A0) - feedback) * Kd;
    feedback = (analogRead(A0));
    error = setpoint - feedback;
    //P = (float)error * Kp;
    
    if (abs(error) < 20)
    {
      D *= 2;
    }
    else
    {
      D *= 8;
    }
    
    P = (float)error * Kp;
    
    if (abs (error) < 100)
    {
      accumulator += error;
      I = accumulator * Ki;                              // Add your PID code here... 
    }
    /*
    else
    {
      I = 0;
    }
    */
    
    //accumulator += error;
    //I = accumulator * Ki; 
    
    plant_drive = (int)P + (int)I - D;
    
    PWM_DC = plant_drive;

    
    
// Limit the duty cycle to +/- 240     
    if (PWM_DC >= 240){
        PWM_DC = 240;
    }
    else if (PWM_DC <= -240){
        PWM_DC = -240;  
    }



// Drive the Cytron MD10C
    if (PWM_DC >=0){
        digitalWrite(10, LOW);
        OCR2A = PWM_DC;                     
    }
    else{
        digitalWrite(10, HIGH);
        OCR2A = -PWM_DC;
    }




    
    //sprintf(buf, "%d, %d, %d, %d\n",feedback, (int)P, (int)I, D);
    sprintf(buf, "%d, %d, %d\n", (int) P, (int)I, D);
    //sprintf(buf, "%d\n", (int)P);
    USART_puts(buf);
  
  
  
  
  
  
    if (++time_in_state == 200){
    
        time_in_state = 0;
      
       if (state){
           setpoint = 600;
           state = 0;
       }
       else{
           setpoint = 300;
           state = 1;
       }
    }
    
     digitalWrite(LED_PIN, LOW);

}





/*********************************************************************************
 *  ____            _____  _  __ _____  _____    ____   _    _  _   _  _____
 * |  _ \    /\    / ____|| |/ // ____||  __ \  / __ \ | |  | || \ | ||  __ \
 * | |_) |  /  \  | |     | ' /| |  __ | |__) || |  | || |  | ||  \| || |  | |
 * |  _ <  / /\ \ | |     |  < | | |_ ||  _  / | |  | || |  | || . ` || |  | |
 * | |_) |/ ____ \| |____ | . \| |__| || | \ \ | |__| || |__| || |\  || |__| |
 * |____//_/    \_\\_____||_|\_\\_____||_|  \_\ \____/  \____/ |_| \_||_____/
 *
 ********************************************************************************/






void loop(){
    // Yup, the main loop does nothing...
}







void init_timer_1_CTC(long desired_ISR_freq){
/*
 * @brief Configure timer #1 to operate in Clear Timer on Capture Match (CTC Mode)
 *
 *      desired_ISR_freq = (f_clk / prescale value) /  Output Compare Registers
 *
 *   For example:
 *        Given a Arduino Uno: F_clk = 16 MHz
 *        let prescale = 64
 *        let desired ISR heartbeat = 100 Hz
 *
 *        if follows that OCR1A = 2500
 *
 * @param desired_ISR_freq is the desired operating frequency of the ISR
 * @param f_clk must be defined globally e.g., #define f_clk 16000000L
 *
 * @return void
 *
 * @note The prescale value is set manually in this function.  Refer to ATMEL ATmega48A/PA/88A/PA/168A/PA/328/P datasheet for specific settings.
 *
 * @warning There are no checks on the desired_ISR_freq parameter.  Use this function with caution.
 *
 * @warning Use of this code will break the Arduino Servo() library.
 */
    cli();                                          // Disable global
    TCCR1A = 0;                                     // Clear timer counter control registers.  The initial value is zero but it appears Arduino code modifies them on startup...
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12);                         // Timer #1 using CTC mode  
    TIMSK1 |= (1 << OCIE1A);                        // Enable CTC interrupt
    TCCR1B |= (1 << CS10)|(1 << CS11);              // Prescale: divide by f_clk by 64.  Note SC12 cleared earlier
    OCR1A = (f_clk / 64L) / desired_ISR_freq;       // Interrupt when TCNT1 equals the top value of the counter specified by OCR
    sei();                                          // Enable global
}
