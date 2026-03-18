#include <Adafruit_NeoPixel.h>

#define LED_PIN 48 
#define NUMPIXELS 1 

#define b_PIN 15

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup(){
  pixels.begin(); 
  pixels.setBrightness(100);
  pinMode(b_PIN, OUTPUT);

  startup();
}

void loop(){

}

void startup(){
  communication(200, 2, 255, 0, 255);
}

void communication(int t, int r, int R, int G, int B){ // t - time delay, r - repeatitions
  for(int i=0; i<r; i++){
    pixels.clear();
    digitalWrite(b_PIN, HIGH);
    pixels.setPixelColor(0, pixels.Color(R, G, B));
    pixels.show();
    delay(t);
    digitalWrite(b_PIN, LOW);
    pixels.clear();
    pixels.show();
    delay(t);
  }
}
