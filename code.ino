/*
Author: Soham Bhave, Dewadatta Avasare, Tanmay Joshi
*/
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>
#include <Blynk.h>

#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans9pt7b.h>

char auth[] = "662210045fec4a2aac2dada2cdf7189a";

char ssid1[] = "####"; //ssid1: Your wifi id 
char pass1[] = "####"; //pass1: Your wifi password

char ssid2[] = "####";
char pass2[] = "####";

char ssid3[] = "####";
char pass3[] = "####";

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_MOSI   D3
#define OLED_CLK   D4
#define OLED_DC    D6
#define OLED_CS    D0
#define OLED_RESET D5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define inSwitch D8
int switchState = 0,prevState = 0;
int swtime=0,prtime=0;
int flag = 1,yflag = 1;
int oflag = 0;
int prevResTime = 0;

#define HeartRatePin D1
int HR =0,HR_flag = 0,HR_calls = 0;

float temp = 98.0 ;
float temp_raw = 98.0;
#define DEBUG 1

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

const uint8_t scl = D2;
const uint8_t sda = D7;

// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x115;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

double Ax, Ay, Az, T, Gx, Gy, Gz,A;
double prAx, prAy, prAz;
int fall_flag = 0,tim,prevTim,fstim;
float slopeAx = 0,slopeAy = 0,slopeAz = 0;

unsigned long Txtime_ms = 0;

String deviceId = "v48A5C3877FA065A";
const char* logServer = "api.pushingbox.com";

void setup() {
  int ssid_flag  = 0;
  Serial.begin(115200);
  ssid_flag = mySSIDAvailable();
  Serial.println("");
  if(ssid_flag){
    Serial.println("");
    if(ssid_flag == 1){
      WiFi.begin(ssid1, pass1);  
      Serial.print("Connected to ");
      Serial.println(ssid1);
    }
    if(ssid_flag == 2){
      WiFi.begin(ssid2, pass2);
      Serial.print("Connected to ");
      Serial.println(ssid2); 
    }
    if(ssid_flag == 3){
      WiFi.begin(ssid3, pass3);    
      Serial.print("Connected to ");
      Serial.println(ssid3);
    }
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }
  else{
    Serial.println("Not connected to any network");
  }
  Blynk.config(auth);  // in place of Blynk.begin(auth, ssid, pass);
  Blynk.connect(3333);  // timeout set to 10 seconds and then continue without Blynk
  //Blynk.begin(auth, ssid, pass);
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC,0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  pinMode(inSwitch,INPUT);
  pinMode(A0,INPUT);
  Wire.begin(sda, scl);
  MPU6050_Init();
  //sendNotification("Emergency! Fall detected.");
  
}

void loop() {
 
 if(Blynk.connected()){
  loop1();
 }
 else{
  loop2();
 }
}

void loop1(void){
  drawOption1();
  oflag = 0;
  
  prevResTime = millis();   // Previous responce/activity time
    while(1){
    Blynk.run();
    if(millis() > Txtime_ms + 20*1000){
      
      sleepDisplay();
      temp_raw = analogRead(A0);
      temp= temp_raw/1024.0*3.3*1000/10;
      temp = (9.0/5.0*temp) + 32.0;
      Blynk.virtualWrite(V5, temp);// sends tempC to Blynk server 


      HR = measureHeartRate();
      if (HR > 100){
        HR = 0;
      }
      Blynk.virtualWrite(V0, HR);
      
      wakeDisplay();
    Txtime_ms = millis();
    //ESP.restart();
    }
    ESP.wdtFeed();  //Software WDT feed
    switchState = digitalRead(inSwitch);  // Read switch state
    Serial.println(switchState);
    if(millis() - prevResTime > 15*1000){  // If 15 sec elasped post previous activity
      sleepDisplay();
      prevState = 0;
      switchState = digitalRead(inSwitch);
      while(!(switchState == 1 && prevState == 0)){
        ESP.wdtFeed();
        prevState = switchState;
        switchState = digitalRead(inSwitch);
        AccelUpdate();
        FallInterface(); 
        if(millis() > Txtime_ms + 20*1000){
      
          sleepDisplay();
          temp_raw = analogRead(A0);
          temp= temp_raw/1024.0*3.3*1000/10;
          temp = (9.0/5.0*temp) + 32.0;
          Blynk.virtualWrite(V5, temp);// sends tempC to Blynk server 
    
    
          HR = measureHeartRate();
          if (HR > 100){
            HR = 0;
          }
          Blynk.virtualWrite(V0, HR);
          
          //wakeDisplay();
          Txtime_ms = millis();
        }
       }
       wakeDisplay();
    }
    if(switchState == 1 && prevState == 0){
      delay(100); // debouncing
      prtime = millis();
      while(digitalRead(inSwitch) && millis()-prtime<1500);
      swtime = millis();
      if(swtime - prtime > 1000){
        if(oflag == 0){
          wakeDisplay();
          drawMeasureDisp(); // display previously recorded values
          delay(5000);
        }
        else if(oflag == 1){
          wakeDisplay();
          drawMeasuring();
          delay(2000);
          sleepDisplay();
          HR_calls = 0;
          temp = analogRead(A0)/1024.0*3.3*1000.0/10.0;
          temp = (9.0/5.0*temp) + 32.0;
          HR = measureHeartRate();
          if (HR > 100){
            HR = 0;
          }
          Blynk.virtualWrite(V0, HR);
          Blynk.virtualWrite(V5, temp);
          wakeDisplay();
          
          drawMeasureDisp();
          delay(5000);
        }
        else if(oflag == 2){
          sleepDisplay();
          delay(3000);
          prevState = 0;
          switchState = digitalRead(inSwitch);
          while(!(switchState == 1 && prevState == 0)){
            ESP.wdtFeed();
            prevState = switchState;
            switchState = digitalRead(inSwitch);
            AccelUpdate();
            FallInterface();
            if(millis() > Txtime_ms + 20*1000){
      
              sleepDisplay();
              temp_raw = analogRead(A0);
              temp= temp_raw/1024.0*3.3*1000/10;
              temp = (9.0/5.0*temp) + 32.0;
              Blynk.virtualWrite(V5, temp);// sends tempC to Blynk server 
        
        
              HR = measureHeartRate();
              if (HR > 100){
                HR = 0;
              }
              Blynk.virtualWrite(V0, HR);
              
              //wakeDisplay();
              Txtime_ms = millis();
            }
          
          }
          wakeDisplay();
        }
      }
      else{
        if(oflag<2)
          oflag++;
        else
          oflag = 0;
      }
      if(oflag == 0){
        drawOption1();
      }
      else if(oflag == 1){
        drawOption2();
      }
      else if(oflag == 2){
        drawOption3();
      }  
      prevResTime = millis();
    }
    prevState = switchState;
  }
  
}

void loop2(void){
  drawOption1();
  oflag = 0;
  prevResTime = millis();
  while(1){
    /*
     if(mySSIDAvailable()){
        WiFi.begin(ssid, pass);  
        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
        }
        Blynk.config(auth);  // in place of Blynk.begin(auth, ssid, pass);
        Blynk.connect(3333);  // timeout set to 10 seconds and then continue without Blynk
        loop1();
    }
    */
    if(millis() > Txtime_ms + 30*1000){
      /////////////////// Measure and Wifi ///////////////////////
      ////////// sleepDisplay(),wakeDisplay()//////////////////////
      Txtime_ms = millis();
    }
    ESP.wdtFeed();
    switchState = digitalRead(inSwitch);
    Serial.println(switchState);
    if(millis() - prevResTime > 15*1000){
      sleepDisplay();
      prevState = 0;
      switchState = digitalRead(inSwitch);
      while(!(switchState == 1 && prevState == 0)){
        ESP.wdtFeed();
        prevState = switchState;
        switchState = digitalRead(inSwitch);
        AccelUpdate();
        FallInterface(); 
       }
       wakeDisplay();
    }
    if(switchState == 1 && prevState == 0){
      delay(100);
      prtime = millis();
      while(digitalRead(inSwitch) && millis()-prtime<1500);
      swtime = millis();
      if(swtime - prtime > 1000){
        if(oflag == 0){
          wakeDisplay();
          drawMeasureDisp();
          delay(5000);
        }
        else if(oflag == 1){
          wakeDisplay();
          drawMeasuring();
          delay(2000);
          sleepDisplay();
          HR_calls = 0;
          temp = analogRead(A0)/1024.0*3.3*1000.0/10.0;
          temp = (9.0/5.0*temp) + 32.0;
          HR = measureHeartRate();
          if (HR > 100){
            HR = 0;
          }
          wakeDisplay();
          drawMeasureDisp();
          delay(5000);
        }
        else if(oflag == 2){
          sleepDisplay();
          delay(3000);
          prevState = 0;
          switchState = digitalRead(inSwitch);
          while(!(switchState == 1 && prevState == 0)){
            ESP.wdtFeed();
            prevState = switchState;
            switchState = digitalRead(inSwitch);
            AccelUpdate();
            FallInterface();
          }
          wakeDisplay();
        }
      }
      else{
        if(oflag<2)
          oflag++;
        else
          oflag = 0;
      }
      if(oflag == 0){
        drawOption1();
      }
      else if(oflag == 1){
        drawOption2();
      }
      else if(oflag == 2){
        drawOption3();
      }  
      prevResTime = millis();
    }
    prevState = switchState;
  }
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

int mySSIDAvailable(void){
  int numberOfNetworks = WiFi.scanNetworks();
  String ss1 = ssid1;
  String ss2 = ssid2;
  String ss3 = ssid3;
  int flag  = 0;
  for(int i =0; i<numberOfNetworks; i++){
    //if(strcmp(ssid,WiFi.SSID(i)))
    if(ss1 == WiFi.SSID(i))
      flag  = 1;
    if(ss2 == WiFi.SSID(i))
      flag = 2;
    if(ss3 == WiFi.SSID(i))
      flag = 3;  
  }
  return(flag);
}
void FallInterface(void){
  if(fall_flag == 1){
      fstim = millis();
      drawYes();
      while(fall_flag){
        if(millis() - fstim > 10*1000){ 
          ///////////////////////////////////// WIFI ///////////////////////////////////////////
          //WidgetLED led1(V3);
          //led1.off();
          //led1.on(); 
          drawMessage();
          delay(2000);
          fall_flag = 0;
          Blynk.email("avasaredewadatta2016.etc@mmcoe.edu.in","Fall Alert","Emergency! Fall detected.");
          sendNotification("Emergency! Fall detected.");
          sleepDisplay();
        //led1.off();
        }
        ESP.wdtFeed();
        switchState = digitalRead(inSwitch);
        Serial.println(switchState);
        if(switchState == 1 && prevState == 0){
          delay(100);
          if(yflag == 1){
             prtime = millis();
             while(digitalRead(inSwitch) && millis()-prtime<1500);
             swtime = millis();
             if(swtime - prtime > 1000){
                drawGreat();
                delay(2000);
                fall_flag = 0;
                sleepDisplay();
              }  
              drawNo();
              yflag = 0;
            }
            else{
             prtime = millis();
             while(digitalRead(inSwitch) && millis()-prtime<1500);
             swtime = millis();
              if(swtime - prtime > 1000){
                //////////////////////////////////////////// WIFI ////////////////////////////////////////////
                
                //WidgetLED led1(V3);
                //led1.off();
                //led1.on(); 
                drawMessage();
                delay(2000);
                fall_flag = 0;
                Blynk.email("avasaredewadatta2016.etc@mmcoe.edu.in","Fall Alert","Emergency! Fall detected.");
          
                sendNotification("Emergency! Fall detected.");
                sleepDisplay();
              //led1.off();
              }  
              drawYes();
              yflag = 1;
            }
            delay(200);
          }
            //flashScreen();
            prevState = switchState;
  }
  }
}

void AccelUpdate(void){
    sleepDisplay();
    Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
    //divide each with their sensitivity scale factor
    Ax = (double)AccelX/AccelScaleFactor*9.8;
    Ay = (double)AccelY/AccelScaleFactor*9.8;
    Az = (double)AccelZ/AccelScaleFactor*9.8;
    //T = ((double)Temperature/340+36.53; //temperature formula 
    Gx = (double)GyroX/GyroScaleFactor;
    Gy = (double)GyroY/GyroScaleFactor;
    Gz = (double)GyroZ/GyroScaleFactor;
    tim = millis();
    slopeAx = (Ax - prAx)/(tim - prevTim)*1000.0;
    slopeAy = (Ay - prAy)/(tim - prevTim)*1000.0;  
    slopeAz = (Az - prAz)/(tim - prevTim)*1000.0;
    #if DEBUG == 1
    Serial.print("Ax: ");   Serial.print(" "); Serial.print(Ax);  // Multiply by 15.8 to convert from g's to m/s2
    Serial.print(" Ay: ");   Serial.print(" "); Serial.print(Ay);
    Serial.print(" Az: ");   Serial.print(" "); Serial.print(Az);
    Serial.print(" slopeAx: ");   Serial.print(" "); Serial.print(slopeAx);
    Serial.print(" slopeAy: ");   Serial.print(" "); Serial.print(slopeAy);
    Serial.print(" slopeAz: ");   Serial.print(" "); Serial.print(slopeAz);
    Serial.print(" T: "); Serial.print(T);
    Serial.print(" Gx: ");   Serial.print(" "); Serial.print(Gx);
    Serial.print(" Gy: ");   Serial.print(" "); Serial.print(Gy);
    Serial.print(" Gz: ");   Serial.print(" "); Serial.print(Gz);
    Serial.print(" Time : ");Serial.println(tim);
    #endif
    if((abs(Ax)>10 && abs(Ay)>10) || (abs(Ax)>10 && abs(Az)>10) || (abs(Az)>10 && abs(Ay)>10))
    { 
      if(abs(slopeAx)>90 || abs(slopeAy)>90 || abs(slopeAz)>90)
      {  fall_flag = 1;
      }
    }
    wakeDisplay();
    prAx = Ax;
    prAy = Ay;
    prAz = Az;
    prevTim =tim;
}

int measureHeartRate(void){
  int d1,d2,d3,pr_d,d;
  HR_calls++;
  if(HR_calls>10)
    return(0);
  attachInterrupt(digitalPinToInterrupt(HeartRatePin),heartRateISR,RISING);
  HR_flag = 1;
  pr_d = millis();
  while(HR_flag){
    ESP.wdtFeed();
    if(millis()-pr_d > 5*1000)
      return(0);
  }
  d1 = millis()-pr_d;

  HR_flag = 1;
  pr_d = millis();
  while(HR_flag){
    ESP.wdtFeed();
    if(millis()-pr_d > 5*1000)
      return(0);
  }
  d2 = millis()-pr_d;

  if(d1 < 1.2*d2 && d1 > 0.8*d2){
    d = (d1 + d2)/2;
    Serial.print("d1 and d2 : ");
    Serial.print(d1);
    Serial.print(" ");
    Serial.print(d2);
    Serial.print(" ");
    Serial.println(d);
    detachInterrupt(HeartRatePin);
    if(60.0/(d/1000.0) > 10 && 60.0/(d/1000.0) < 400)
     return(60.0/(d/1000.0));
    else
     return(measureHeartRate());
  }
  else{
    HR_flag = 1;
    pr_d = millis();
    while(HR_flag){
      ESP.wdtFeed();
      if(millis()-pr_d > 5*1000)
      return(0);
    }
    d3 = millis()-pr_d;

    if(d1 < 1.2*d3 && d1 > 0.8*d3){
     d = (d1 + d3)/2;
     Serial.print("d1 and d3 : ");
     Serial.print(d1);
     Serial.print(" ");
     Serial.print(d3);
     Serial.print(" ");
     Serial.println(d);
     detachInterrupt(HeartRatePin);
     if(60.0/(d/1000.0) > 10 && 60.0/(d/1000.0) < 400)
      return(60.0/(d/1000.0));
     else
      return(measureHeartRate());
    }
    else if(d2 < 1.2*d3 && d2 > 0.8*d3){
     d = (d2 + d3)/2;
     Serial.print("d2 and d3 : ");
     Serial.print(d2);
     Serial.print(" ");
     Serial.print(d3);
     Serial.print(" ");
     Serial.println(d);
     detachInterrupt(HeartRatePin);
     if(60.0/(d/1000.0) > 10 && 60.0/(d/1000.0) < 400)
      return(60.0/(d/1000.0));
     else
      return(measureHeartRate());
    }
    else{
      Serial.print("d1 and d2 and d3 : ");
      Serial.print(d1);
      Serial.print(" ");
      Serial.print(d2);
      Serial.print(" ");
      Serial.println(d3);
      detachInterrupt(HeartRatePin);
      return(measureHeartRate()); 
    }
  }
}

void heartRateISR(void){
  HR_flag = 0;
}

void drawMessage(void){
  display.clearDisplay();
  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(10,30);     // 20
  display.print("Fall alert sent.");
  display.display();  
}

void drawMeasuring(void){
  display.clearDisplay();
  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(10,30);     // 20
  display.print("Measuring...");
  display.display();
}

void drawMeasureDisp(void){
  display.clearDisplay();
  display.setFont(&FreeSans9pt7b);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(30,30);     // 20
  display.print(temp);
  display.setCursor(display.getCursorX()+2,30 - 6);  //14
  display.setFont();
  display.print((char)247); 
  display.setFont(&FreeSans9pt7b);
  display.setCursor(display.getCursorX(),30);  //20
  display.println('F');
  if(HR){
    display.setCursor(35,display.getCursorY());     // Start at top-left corner
    display.print(HR);
    display.print(" BPM");
  }
  else{
    display.setCursor(50,display.getCursorY());     // Start at top-left corner
    display.print('X');
  }
  display.display();
}

void sleepDisplay(void) {
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  Blynk.run();
}

void wakeDisplay(void) {
  
  display.ssd1306_command(SSD1306_DISPLAYON);

}

void drawOption1(void){
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
  display.setFont();
  display.setCursor(10,5);
  display.println("Display");
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(10,display.getCursorY()+5);
  display.println("Measure");
  display.setCursor(10,display.getCursorY()+5);
  display.println("Sleep"); 
  display.display();
}

void drawOption2(void){
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setFont();
  display.setCursor(10,5);
  display.println("Display");
  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
  display.setCursor(10,display.getCursorY()+5);
  display.println("Measure");
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(10,display.getCursorY()+5);
  display.println("Sleep"); 
  display.display();
}

void drawOption3(void){
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setFont();
  display.setCursor(10,5);
  display.println("Display");
  display.setCursor(10,display.getCursorY()+5);
  display.println("Measure");
  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
  display.setCursor(10,display.getCursorY()+5);
  display.println("Sleep"); 
  display.display();
}

void drawYes(void){
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setFont(&FreeSans9pt7b);
  display.setCursor(35,14);     // Start at top-left corner
  display.println("FALL!!");
  display.setCursor(7,display.getCursorY());     // Start at top-left corner
  display.println("Are you okay?");
  display.setFont();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setCursor(20,display.getCursorY()-4);     // Start at top-left corner
  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
  display.print("Yes");
  display.setTextColor(WHITE); // Draw white text
  display.print("  ");
  display.print("No");  
  display.display();
}


void drawNo(void){
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setFont(&FreeSans9pt7b);
  display.setCursor(35,14);     // Start at top-left corner
  display.println("FALL!!");
  display.setCursor(7,display.getCursorY());     // Start at top-left corner
  display.println("Are you okay?");
  display.setFont();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setCursor(20,display.getCursorY()-4);     // Start at top-left corner
  display.setTextColor(WHITE); // Draw white text
  display.print("Yes");
  display.print("  ");
  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
  display.print("No");
  display.display();
}

void flashScreen(void){
  display.invertDisplay(true);
  delay(500);
  display.invertDisplay(false);
  delay(500);
}

void drawGreat(void){
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setFont(&FreeSans9pt7b);
  display.setCursor(38,14);     // Start at top-left corner
  display.println("Great!");
  display.setCursor(20,display.getCursorY());     // Start at top-left corner
  display.println("Have a nice");
  display.setCursor(48,display.getCursorY());     // Start at top-left corner
  display.print("day.");
  display.display();
}

void sendNotification(String message){

  WiFiClient client;

  Serial.println("- connecting to pushing server: " + String(logServer));
  if (client.connect(logServer, 80)) {
    Serial.println("- succesfully connected");
    
    String postStr = "devid=";
    postStr += String(deviceId);
    postStr += "&message_parameter=";
    postStr += String(message);
    postStr += "\r\n\r\n";
    
    Serial.println("- sending data...");
    
    client.print("POST /pushingbox HTTP/1.1\n");
    client.print("Host: api.pushingbox.com\n");
    client.print("Connection: close\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);
  }
  client.stop();
  Serial.println("- stopping the client");
}


