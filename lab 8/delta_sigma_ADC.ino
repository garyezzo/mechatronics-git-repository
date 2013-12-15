/*
 * An Arduino based Sigma_delta
 *
 */

// AVR Libc includes
   #include <avr/io.h>

/* Identify ADC pins */

    #define LED 13

/* Global variables to reduce clutter */


void setup() {
    //opens the serial comm
    Serial.begin(57600);
    pinMode(7, INPUT);
    pinMode(6, INPUT);
    pinMode(2, OUTPUT);
    ACSR &= ~(1<<ACD);               //switch on the AC
    delay(100);
}



void loop() {
    
    //ACSR &= ~(1<<ACD);             //switch on the AC
    
    uint16_t accumulator;
    float voltage = 0.0;
    accumulator = 0;
    uint16_t i;
    for (i = 0; i <= 65534; i++)
    {
      //Serial.println (i);
      ACSR &= ~(1<<ACD);  
      if ((ACSR && (1<<ACO)) == 1) // if analog comparator is high 
      { 
          accumulator++;
          digitalWrite(2, HIGH);
          //digitalWrite(6, HIGH);
          //digitalWrite(LED, HIGH);
      }
      else
      {
          //digitalWrite(LED, LOW);
          digitalWrite(2, LOW);
          //digitalWrite(6, LOW);
      }
    }
    
    voltage = 3.4 - (5.00 * ((float) accumulator / 65534));
    //Serial.println (voltage);
    //Serial.print ("    ");
    
    Serial.println (accumulator);
    
    //Serial.println ("gary");
}



uint32_t move_avg (uint16_t new_value){
// Code adapted from class textbook: Carryer, J. Edward., R. Matthew. Ohline, and Thomas William. Kenny. Introduction to Mechatronic Design. Upper Saddle River: Prentice Hall, 2011. Print.
    uint32_t run_sum = 0;  
    uint32_t buf[16] = {0,0,0,0,0,0,0,0,    0,0,0,0,0,0,0,0};
    uint8_t newest = 0;
    uint8_t oldest = 1;
    
    run_sum = run_sum - buf[oldest] + new_value;
    buf[newest] = new_value;
    newest = (newest + 1) & 0x0F;
    oldest = (oldest + 1) & 0x0F;
    return(run_sum >> 4);
}

