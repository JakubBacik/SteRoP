#include <Arduino.h>
#define Btn1_GPIO 35

#define RXD2 16
#define TXD2 17

volatile int interruptFlag=0;

/*
 * Przerawnie które powoduje wywołanie funkcji odpowiedzialnej za czytanie z UART
 **/
void IRAM_ATTR readFromPort(){
  interruptFlag=1;
}


void readFromUart(){
  Serial.print(Serial2.readString());
}

void setup() {
  Serial.begin(115200);
  pinMode(Btn1_GPIO, INPUT_PULLUP);
  attachInterrupt(Btn1_GPIO, readFromPort, FALLING);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
  if(interruptFlag == 1){
    readFromUart();
    interruptFlag = 0;
  }
}