void setup ()
{
  Serial.begin (9600);
  pinMode(11, OUTPUT);
}

void loop ()
{
  
  digitalWrite (11, HIGH);
  pinMode(10, OUTPUT);
  digitalWrite (10, LOW);
  delay (22);
  pinMode (10, INPUT);
  int startTime = millis ();
  digitalWrite (11, LOW);
  while (digitalRead (9) == 0);
  
  int stopTime = millis ();
  float voltage = .008 * (stopTime - startTime);
  
  Serial.println (voltage);
  /*
  int accumulator = 0;
  float voltage = 0;
  for (float i = 0; i <= pow(2, 16); i++)
  {
    if (digitalRead(6) == HIGH)
    {
      accumulator++;
    }
  }
  voltage = 5.00 * (accumulator / (pow(2, 16) - 1));
  Serial.println (voltage);
  */
}
