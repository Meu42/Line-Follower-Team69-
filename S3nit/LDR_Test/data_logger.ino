#define LDR_PIN A0

void setup() {
  Serial.begin(9600); 
  pinMode(LDR_PIN, INPUT);
  Serial.println("System Initialized...");
}

void loop() {                                                                                                  
  // Store the reading in a variable
  int lightValue = analogRead(LDR_PIN);  
  Serial.println(lightValue);
  
  delay(1000);
}
