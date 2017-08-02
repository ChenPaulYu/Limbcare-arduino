#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <Wire.h>
#include <Adafruit_MLX90614.h>



// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2      // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();


String buffer = "";
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;
const int  hotOrCold_PIN = 8;
const int temperature_PIN = 7;
const int touchPIN = 6;
bool touch_State = false;
bool last_touch_State = false;
bool hotOrCold_State = false;
int  temperature_Value = 255;
float temperature_ref = 0;
float temperature_current = 0;
float temperature_target = 0;



void setup(void)
{
  Serial.begin(9600);
  //  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));
  pinMode(touchPIN, INPUT);
  BTLEserial.setDeviceName("LimbCare"); //更改裝置名稱
  BTLEserial.begin();
  mlx.begin();
}




void loop()
{
  BTLEserial.pollACI(); //BLE模組初始化
  aci_evt_opcode_t status = BTLEserial.getState(); //取得目前狀態
  laststatus = checkingState(status); //確認目前狀態


  //確認是否在連接狀態
  if (status == ACI_EVT_CONNECTED) {
    getData(); //取得傳過的資料
    temperature_current = getTemp();
    Serial.println(abs(temperature_current - temperature_ref));
    if((abs(temperature_current - temperature_ref))>0.1){
      temperature_ref =  temperature_current;
      writeData(temperature_ref);
    }
  }

  touchSignal();
  

  if (hotOrCold_State) {
    digitalWrite(hotOrCold_PIN, HIGH);
  } else {
    digitalWrite(hotOrCold_PIN, LOW);
  }

  analogWrite(temperature_PIN, temperature_Value);

  //確認輸出Port是否有文字
//  if (Serial.available()) {
//    writeData(test); //傳輸文字於手機端
//  }
  
}

aci_evt_opcode_t checkingState(aci_evt_opcode_t status) {
  
  if (status != laststatus) {
    if (status == ACI_EVT_DEVICE_STARTED) {
      Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
      Serial.println(F("* Connected!"));
      
    }
    if (status == ACI_EVT_DISCONNECTED) {
      Serial.println(F("* Disconnected or advertising timed out"));
    }
  }
  return status;
}

void touchSignal() {
  
  temperature_Value = 100;
  touch_State = digitalRead(touchPIN);
  
  if ((touch_State) && (touch_State != last_touch_State)) {
    hotOrCold_State = !hotOrCold_State;
  }
  last_touch_State = touch_State;
}


void getData() {
  if (BTLEserial.available()) {
    Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
  }
  while (BTLEserial.available()) {
    char c = BTLEserial.read();
    buffer.concat(c);
  }
  if (buffer != "") {
    String State = buffer.substring(0, 1);
    Serial.println(State);
    if (State == "T") {
      String Temp = buffer.substring(1);
      Serial.println(Temp);
      temperature_Value = Temp.toInt();
    } else if (State = "B") {
      int buttonState = buffer.substring(1).toInt();
      if (buttonState == 1 ) {
        hotOrCold_State = true;
      } else {
        hotOrCold_State = false;
      }
    }
    buffer = "";
  }
}

void writeData(float temp) {
  Serial.setTimeout(100); // 100 millisecond timeout
  String s = String(temp);
  Serial.println(s);
  uint8_t sendbuffer[20];
//  s.getBytes(sendbuffer, 20);
  s.getBytes(sendbuffer,20);
  char sendbuffersize = min(20, s.length());
  Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");
  BTLEserial.write(sendbuffer, sendbuffersize);
}

float getTemp() {
    
    return mlx.readObjectTempC();    
}










