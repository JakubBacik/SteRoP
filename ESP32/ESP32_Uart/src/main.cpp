#include <Arduino.h>
#include <CRC16.h>
#define Btn1_GPIO 35
CRC16 crc;

#define RXD2 16
#define TXD2 17

volatile int interruptFlag=0;

/*
 * Przerawnie które powoduje wywołanie funkcji odpowiedzialnej za czytanie z UART
 **/
void IRAM_ATTR readFromPort(){
  interruptFlag=1;
}

unsigned short MakeFrame(String toCRC16){
  crc.setPolynome(0x1021);
  for(unsigned int i=0; i< toCRC16.length()-1; i++){
    crc.add(toCRC16[i]);
  }

  unsigned short toReturn = crc.getCRC();
  crc.restart();
  return toReturn;
}

void readFromUart(){
  String toReturn = Serial2.readString();
  Serial.print(toReturn);
  Serial.println(MakeFrame(toReturn), HEX);
}



void setup() {
  Serial.begin(115200);
  pinMode(Btn1_GPIO, INPUT_PULLUP);
  attachInterrupt(Btn1_GPIO, readFromPort, FALLING);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
  String test = "X 1001 1001 1005 3550 ";
  unsigned int tak = MakeFrame(test);
  Serial.println(tak, HEX);
  delay(1000);
  if(interruptFlag == 1){
    readFromUart();
    interruptFlag = 0;
  }
}