#define DEBUG
//#define DEBUG_WIFI
#define AP_SSID "iotA"
#define AP_PASS "iotA1234"
#define SERVER_NAME "10.10.14.89"
#define SERVER_PORT 5000
#define LOGID "CAR01_ARD"
#define PASSWD "PASSWD"

#define WIFITX 7  //7:TX -->ESP8266 RX
#define WIFIRX 6  //6:RX-->ESP8266 TX
#define LED_TEST_PIN 12
#define LED_BUILTIN_PIN 13
#define TACT_PIN 2

#define CMD_SIZE 50
#define ARR_CNT 5

// LCD Header
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Wifi Header
#include "WiFiEsp.h"
#include "SoftwareSerial.h"

#include <TimerOne.h>

char sendBuf[CMD_SIZE];
bool timerIsrFlag = false;

volatile int lcdIndex = 1; 
bool tactState = false; 
bool tactPressed = false; 

bool distanceAlert = false; 
unsigned long prevBlinkTime = 0; 
bool ledState = false;           

unsigned long secCount;
int sensorTime;

SoftwareSerial wifiSerial(WIFIRX, WIFITX);
WiFiEspClient client;

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,16,2);
char buf[17]; // LCD1602

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_TEST_PIN, OUTPUT);     //D12
  pinMode(LED_BUILTIN_PIN, OUTPUT);  //D13
  pinMode(TACT_PIN, INPUT_PULLUP);   //D2
  attachInterrupt(digitalPinToInterrupt(TACT_PIN), tactISR, FALLING); // 눌릴 때 ISR 호출

  Serial.begin(115200);              //DEBUG
  wifi_Setup();
  Timer1.initialize(1000000);         //1000000uS ==> 1Sec
  Timer1.attachInterrupt(timerIsr);  // timerIsr to run every 1 seconds

  lcd.init();                      // initialize the lcd
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Hello!");
  lcd.setCursor(0,1);
  lcd.print("Smart Car!");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (client.available()) {
    socketEvent();
    
  }
  if (timerIsrFlag) {
    timerIsrFlag = false;
    if (!(secCount % 5)) {
      if (!client.connected()) {
        server_Connect();
      }
    }
  }
  // distance <= 10이면 LED_BUILTIN_PIN 깜빡임
  if (distanceAlert) {
    unsigned long currentMillis = millis();
    if (currentMillis - prevBlinkTime >= 500) {
      prevBlinkTime = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN_PIN, ledState);
    }
  }
}
void socketEvent() {
  int i = 0;
  char* pToken;
  char* pArray[ARR_CNT] = { 0 };
  char recvBuf[CMD_SIZE] = { 0 };
  int len;

  len = client.readBytesUntil('\n', recvBuf, CMD_SIZE);
  recvBuf[len] = '\0';

  pToken = strtok(recvBuf, "[@]");
  while (pToken != NULL) {
    pArray[i] = pToken;
    if (++i >= ARR_CNT)
      break;
    pToken = strtok(NULL, "[@]");
  }

  for (int i = 0; pArray[lcdIndex] && pArray[lcdIndex][i]; i++) {
    if (pArray[lcdIndex][i] == '\r' || pArray[lcdIndex][i] == '\n') {
        pArray[lcdIndex][i] = '\0';
        break;
    }
}
  
#ifdef DEBUG
  Serial.print("recv : 1. ");
  Serial.print(pArray[1]);
  Serial.print(" 2. ");
  Serial.print(pArray[2]);
  Serial.print(" 3. ");
  Serial.print(pArray[3]);

  lcd.setCursor(0,0);
  lcd.print(pArray[0]);
  lcd.print("       ");

  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  if (pArray[lcdIndex] != NULL) {
    if (lcdIndex == 1){
      snprintf(buf, sizeof(buf), "Speed: %s", pArray[lcdIndex]);
    }
    else if (lcdIndex == 2) {
      snprintf(buf, sizeof(buf), "distance: %s", pArray[lcdIndex]);
    }
    else if (lcdIndex == 3) {
      snprintf(buf, sizeof(buf), "PWM: %s", pArray[lcdIndex]);  
    }
  } else {
    snprintf(buf, sizeof(buf), "No data");
  }
  int distValue = atoi(pArray[2]);
  if (distValue < 10) {
    distanceAlert = true;
  } else {
    distanceAlert = false;
    digitalWrite(LED_BUILTIN_PIN, LOW);
  }  
  
  lcd.print(buf);
#endif
  if (!strncmp(pArray[1], " New connected", 4))  // New Connected
  {
    Serial.write('\n');
    return;
  } else if (!strncmp(pArray[1], " Alr", 4))  //Already logged
  {
    Serial.write('\n');
    client.stop();
    server_Connect();
    return;
  } 

#ifdef DEBUG
  Serial.print(", send : ");
  Serial.print(sendBuf);
  Serial.print("\n");;
#endif
}
void timerIsr() {
  //  digitalWrite(LED_BUILTIN_PIN,!digitalRead(LED_BUILTIN_PIN));
  timerIsrFlag = true;
  secCount++;
}
void wifi_Setup() {
  wifiSerial.begin(38400);
  wifi_Init();
  server_Connect();
}
void wifi_Init() {
  do {
    WiFi.init(&wifiSerial);
    if (WiFi.status() == WL_NO_SHIELD) {
#ifdef DEBUG_WIFI
      Serial.println("WiFi shield not present");
#endif
    } else
      break;
  } while (1);

#ifdef DEBUG_WIFI
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(AP_SSID);
#endif
  while (WiFi.begin(AP_SSID, AP_PASS) != WL_CONNECTED) {
#ifdef DEBUG_WIFI
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(AP_SSID);
#endif
  }
#ifdef DEBUG_WIFI
  Serial.println("You're connected to the network");
  printWifiStatus();
#endif
}
int server_Connect() {
#ifdef DEBUG_WIFI
  Serial.println("Starting connection to server...");
#endif

  if (client.connect(SERVER_NAME, SERVER_PORT)) {
#ifdef DEBUG_WIFI
    Serial.println("Connected to server");
#endif
    client.print("[" LOGID ":" PASSWD "]");
  } else {
#ifdef DEBUG_WIFI
    Serial.println("server connection failure");
#endif
  }
}
void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void tactISR() {
  lcdIndex++;
  if (lcdIndex > 3) lcdIndex = 1;
}
