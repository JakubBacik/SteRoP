#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "NTPClient.h"
#include <WiFiUdp.h>
#include <sqlite3.h>
#include <SPI.h>
#include "SPIFFS.h"
#include "SD.h"
#include <CRC16.h>
#include "../inc/password.hh"

#define Btn1_GPIO 35

#define RXD2 16
#define TXD2 17
CRC16 crc;

WebServer server(80);

volatile int interruptFlag=0;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

int dataToDatabase[3] = {0,0,0};

void dataFromSTM(int *dataFromTabel); 

/*
 * Przerawnie które powoduje wywołanie funkcji odpowiedzialnej za czytanie z UART
 **/
void IRAM_ATTR readFromPort(){
  interruptFlag=1;
}

/*
 * Funkcja odpowiedzialna za obliczenie sumy kontrolnej CRC16
 */ 
unsigned short MakeFrame(String toCRC16){
  crc.setPolynome(0x1021);
  for(unsigned int i=0; i< toCRC16.length()-1; i++){
    crc.add(toCRC16[i]);
  }

  unsigned short toReturn = crc.getCRC();
  crc.restart();

  return toReturn;
}

/*
 * Funkcja odpowiedzialna za sprawdzenie poprawności danych
 * oraz parsowanie ich do tablicy.
 */

bool CheckData(String str, int* toReturn){
  
  if(str[0] != 'X'){
   return false; 
  }
  String wordToAdd = "";
  String stringToCRC16 ="X ";
  int number = 0;
  String toCompareCRC16;

  for(int i=2; i<(int)str.length()+1; i++){
    if(str[i] != ' ' && str[i] != '\n'){
      wordToAdd += str[i];
    }else{
      if(number < 3){
        stringToCRC16 +=wordToAdd;
        stringToCRC16 +=" ";

        toReturn[number] = wordToAdd.toInt();
        wordToAdd="";
      }
      if(number == 3){
        toCompareCRC16 = wordToAdd.substring(0,4);
      }
      number++;
    }
  }

  if(number != 4){
    return false;    
  }

  stringToCRC16 += " ";
  unsigned int frame = MakeFrame(stringToCRC16);
  String anotherOne = String(frame, HEX);
  anotherOne.toUpperCase();

  if(toCompareCRC16 == anotherOne){
    return true;
  }else{
    return false; 
     
  }
}
/*
 * Funkcja odpowiedzialana za czytanie danych z UARTu.
 */
void readFromUart(int* dataTab){
  String dataFromUart = Serial2.readString();
  if(CheckData(dataFromUart, dataTab)){
    dataFromSTM(dataTab);
  }
}


/* Funkcja odpowiedzialna za pobranie z SPIFFS pliku .html 
 * strony głównej
 */ 
String getPage()
{
  String strona;
  File file = SPIFFS.open("/index.html", "r");
  while(file.available()) strona+=(char)file.read();
  file.close();
  return strona;
} 

/* Funkcja odpowiedzialna za pobranie z SPIFFS pliku .js
 * skrypt
 */ 
String getScript()
{
  String strona;
  File file = SPIFFS.open("/skrypty.js", "r");
  while(file.available()) strona+=(char)file.read();
  file.close();
  return strona;
} 

/* Funkcja odpowiedzialna za pobranie z SPIFFS pliku .html 
 * strony z tabelą stworzoną na podstawie danych z dwoch dni
 */ 
String getDataBase()
{
  String strona;
  File file = SPIFFS.open("/Database.html", "r");
  while(file.available()) strona+=(char)file.read();
  file.close();
  return strona;
} 

/* Funkcja odpowiedzialna za pobranie z SPIFFS pliku .html 
 * strony z wykresem stworzonym na podstawie danych z dwoch dni
 */ 
String getPlot()
{
  String strona;
  File file = SPIFFS.open("/Plot.html", "r");
  while(file.available()) strona+=(char)file.read();
  file.close();
  return strona;
} 

sqlite3 *db1;
int rc;
sqlite3_stmt *res;
int rec_count = 0;
const char *tail;
char current_db[255];

/* Funkcja odpowiedzialna za otworzenie bazy danych
 * ustawia ona nazwe obeznej bazy danych, jeśli nazwa
 * bazy danych nie ulegla zmianie to ją zamyka. Następnie
 * otwiera, opatrzona jest w komunikat w przypadku poprawnego
 * otwarcia bazy danych lub błędu.
 */ 
int openDb(const char *filename) {
  if (strncmp(filename, current_db, sizeof(current_db)) == 0)
    return 0;
  else
    sqlite3_close(db1);
  int rc = sqlite3_open(filename, &db1);
  if (rc) {
      Serial.printf("Can't open database: %s\n", sqlite3_errmsg(db1));
      memset(current_db, '\0', sizeof(current_db));
      return rc;
  } else {
      Serial.printf("Opened database successfully\n");
      strcpy(current_db, filename);
  }
  return rc;
}

/* 
 * Funkcja odpowiedzialna za utworzenie nowego rekordu w bazie danych
 * z wygenerowanych losowo danych. Dodatkowo pobiera aktualną date za
 * pomocą bilbioteki NTPClient.
 **/
void dataFromSTM(int *dataFromTabel) 
{
  String db_name = "/sd/DataBase.db";
  String sql = "INSERT INTO WeatherStation (Pressure, Humidity, Temperature, Data, Time) VALUES (?, ?, ?, ?, ?)";
  String resp;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  openDb(db_name.c_str());
  rc = sqlite3_prepare_v2(db1, sql.c_str(), 1000, &res, &tail);
  if (rc != SQLITE_OK) {
    String resp = "Failed to fetch data: ";
    resp += sqlite3_errmsg(db1);
  }

  timeClient.forceUpdate();

  String formattedDate = timeClient.getFormattedDate();
  int splitT = formattedDate.indexOf("T");
  String dayStamp = formattedDate.substring(0, splitT);
  String timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-4);

  sqlite3_bind_int(res, 1, dataFromTabel[2]/100);
  sqlite3_bind_int(res, 2, dataFromTabel[1]);
  sqlite3_bind_int(res, 3, dataFromTabel[0]);
  sqlite3_bind_text(res, 4, dayStamp.c_str(), strlen(dayStamp.c_str()), SQLITE_STATIC);
  sqlite3_bind_text(res, 5, timeStamp.c_str(), strlen(timeStamp.c_str()), SQLITE_STATIC);

  Serial.println("New record");

  if (sqlite3_step(res) != SQLITE_DONE) {
    Serial.printf("ERROR executing stmt: %s\n", sqlite3_errmsg(db1));
    sqlite3_close(db1);
    return;
  }

  sqlite3_clear_bindings(res);
  rc = sqlite3_reset(res);
  if (rc != SQLITE_OK) {
    sqlite3_close(db1);
    return;
  }
  sqlite3_finalize(res);

}

/*
 * Funkcja odpowiedzialna za wysłanie odpowiednich danych do klienta 
 * na odpowiednie żądanie. Wysła stronę główną
 */
void handleRoot()
{ 
  server.send (200, "text/html", getPage()); 
}

/*
 * Funkcja odpowiedzialna za wysłanie odpowiednich danych do klienta 
 * na odpowiednie żądanie. Wysła skrypty w JavaScript
 */
void handleSkrypty()
{
  server.send(200, "text/javascript", getScript()); //skrypty
}

/*
 * Funkcja odpowiedzialna za wysłanie odpowiednich danych do klienta 
 * na odpowiednie żądanie. Wysła plik .html z tabela, która zostanie
 * wypełniona danymi z bazy danych
 */
void handleDataBase(){
  server.send(200, "text/html", getDataBase()); //skrypty
}

/*
 * Funkcja odpowiedzialna za wysłanie odpowiednich danych do klienta 
 * na odpowiednie żądanie. Wysła plik .html z wykresem, który zostanie
 * wypełniony danymi z bazy danych
 */
void handlePlot(){
  server.send(200, "text/html", getPlot()); //skrypty
}

/*
 * Funkcja odpowiedzialna za odczytanie ostanich 287 danych z bazy danych
 * danych co odpowiada informacją pogodowym z dwóch dni.
 * */
void handleData()
{ 
  String db_name = "/sd/DataBase.db";
  String sql = "SELECT * FROM WeatherStation ORDER BY Id DESC LIMIT 287";
  String resp;
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  openDb(db_name.c_str());

  rc = sqlite3_prepare_v2(db1, sql.c_str(), 1000, &res, &tail);
  if (rc != SQLITE_OK) {
    String resp = "Failed to fetch data: ";
    resp += sqlite3_errmsg(db1);
  }

  rec_count = 0;
  while (sqlite3_step(res) == SQLITE_ROW) {
    int count = sqlite3_column_count(res);
    for (int i = 0; i<count; i++) {
      resp += (const char *) sqlite3_column_text(res, i);
      resp += "!";
    }
    resp += "\n";      
  }
    
  sqlite3_finalize(res);
  server.send (200, "text/html", resp);
}

/*
 * Funkcja odpowiedzialna za odczytanie najnowszego rekordu z 
 * bazy danych
 */
void handleCurrentData(){
  String db_name = "/sd/DataBase.db";
  String sql = "SELECT * FROM WeatherStation ORDER BY Id DESC LIMIT 1";
  String resp;
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  openDb(db_name.c_str());

  rc = sqlite3_prepare_v2(db1, sql.c_str(), 1000, &res, &tail);
  if (rc != SQLITE_OK) {
    String resp = "Failed to fetch data: ";
    resp += sqlite3_errmsg(db1);
  }

  rec_count = 0;
  while (sqlite3_step(res) == SQLITE_ROW) {
    int count = sqlite3_column_count(res);
    for (int i = 0; i<count; i++) {
      resp += (const char *) sqlite3_column_text(res, i);
       resp += "!";
    }
    resp += "\n";     
  }
    
  sqlite3_finalize(res);
  server.send (200, "text/html", resp);
}

void setup()
{
  Serial.begin(115200);
  pinMode(Btn1_GPIO, INPUT_PULLUP);
  attachInterrupt(Btn1_GPIO, readFromPort, FALLING);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  SPIFFS.begin();
  //Połączenie z WIFI
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) 
  {
    delay ( 500 );
    Serial.print ( "." );
  }
  

  Serial.println ( "" );
  Serial.print ( "Connected to " );
  Serial.println ( ssid );
  Serial.print ( "IP address: " );
  Serial.println ( WiFi.localIP() );

  if ( MDNS.begin ( "esp32" ) ) {
      Serial.println ( "MDNS responder started" );
  }
  timeClient.begin();
  timeClient.setTimeOffset(3600);

  memset(current_db, '\0', sizeof(current_db));
  SPI.begin();
  SD.begin();
  sqlite3_initialize();
  //Handlery dla odpowiednich akcji
  server.on("/", handleRoot);
  server.on("/index.html", handleRoot);
  server.on("/dates", handleData);
  server.on("/currentdates", handleCurrentData);
  server.on("/skrypty.js", handleSkrypty);
  server.on("/Database.html", handleDataBase);
  server.on("/Plot.html", handlePlot);
  server.uri();
  server.begin(); 
  Serial.println ( "HTTP server started" );
  
  timeClient.begin();
  timeClient.setTimeOffset(7200);

}

void loop()
{
  server.handleClient();
  if(interruptFlag == 1){
    readFromUart(dataToDatabase);
    interruptFlag = 0;
  } 
}
