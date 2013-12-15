void setup ()
{
  Serial.begin (9600);
}

void loop ()
{
  /*
  Serial.println (analogRead (3));
  int x = analogRead (3);
  delay (1000);
  */
  
  
  float Vout = ((float) analogRead (3) / 1023.00) * 5.00;
  float Rf = (Vout - 1.00) * (6300);
  //Serial.print (Rf);
  //Serial.print ("  ");
  
  // ****************** CALCULATE TEMPERATURE BY REVERSE TRANSFER FUNCTION ****************** 
  float temperature = 1 / ((1 / 3984.00) * (float) log (Rf / 10000.00) + (1 / 298.00));
  temperature -= 272.15;
  float temperatureFarenheit = ((temperature * 9) / 5) + 32;
  
  Serial.print ("Temp in Celsius: ");
  Serial.print (temperature);
  Serial.print ("  Temp in Farenheit: ");
  Serial.println (temperatureFarenheit);
  delay (200);
  
}
