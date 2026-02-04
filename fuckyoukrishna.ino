#include <Arduino.h>         
#include <Wire.h>            
#include <vl53l4cd_class.h>  

// Define the pins used for the I2C bus on the ACEBOTT ESP32
#define SDA_PIN 21           
#define SCL_PIN 22           
#define MUX_ADDR 0x70        

// adress for motor 
#define INT1 16
#define INT2 17
#define INT3 18
#define INT4 19

// --- NEW SPEED SETTING ---
// 100 on USB (5V) is roughly equal to 70 on Battery (7.4V)
// This makes the motors spin at the "USB Speed" even on battery.
#define MOTOR_SPEED 70  

VL53L4CD sensor(&Wire, -1); 

void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);  
  Wire.write(1 << channel);          
  Wire.endTransmission();            
  delay(5);                          
}

void setup() {
  // 1. FIX STARTUP GLITCH
  delay(3000);                       

  Serial.begin(115200);              
  delay(2000);                       

  Wire.begin(SDA_PIN, SCL_PIN);      
  Wire.setClock(100000);             

  for (uint8_t i = 0; i < 3; i++) {
    selectMuxChannel(i);             
    if (sensor.begin() == 0 && sensor.InitSensor() == 0) {
      sensor.VL53L4CD_StartRanging(); 
    }
  }
  pinMode(INT1, OUTPUT); 
  pinMode(INT2, OUTPUT); 
  pinMode(INT3, OUTPUT); 
  pinMode(INT4, OUTPUT); 
}

void loop() {
  uint16_t dists[3] = {0, 0, 0};     

  for (uint8_t i = 0; i < 3; i++) {
    selectMuxChannel(i);             
    
    uint8_t isReady = 0;             
    VL53L4CD_Result_t results;       
    
    sensor.VL53L4CD_CheckForDataReady(&isReady); 

    if (isReady) {
      sensor.VL53L4CD_GetResult(&results);  
      dists[i] = results.distance_mm;       
      sensor.VL53L4CD_ClearInterrupt();     
    }
  }
  
  // PRINTING DATA 
  Serial.print("Left_S0:");          
  Serial.print(dists[2]);            
  Serial.print(",");                 
  
  Serial.print("Center_S1:");        
  Serial.print(dists[1]);            
  Serial.print(",");                 
  
  Serial.print("Right_S2:");         
  Serial.print(dists[0]);            
  
  Serial.println();                  
  delay(30);                         

  // --- LOGIC (KEPT EXACTLY AS YOU HAD IT) ---
  if (dists[1] > 100 || 50 >= dists[1]) {
    forward();
    delay(50);
  }
  else if (dists[0] > dists[2] || 50 >= dists[0]) {
    right();
  }
  else if (dists[2] > dists[0] || 50 >= dists[2]){
    left();
  }
  else {
    stop();
  }
}

// --- COMMANDS (UPDATED WITH "MOTOR_SPEED" INSTEAD OF 100) ---

void forward() {
  analogWrite(INT1, MOTOR_SPEED); 
  analogWrite(INT2, 0); 
  analogWrite(INT3, 0);
  analogWrite(INT4, MOTOR_SPEED);
}

void backward() {
  analogWrite(INT1, 0);
  analogWrite(INT2, MOTOR_SPEED);
  analogWrite(INT3, MOTOR_SPEED);
  analogWrite(INT4, 0);
}

void stop() {
  analogWrite(INT1, 0);
  analogWrite(INT2, 0);
  analogWrite(INT3, 0);
  analogWrite(INT4, 0);
}

void right() { 
  analogWrite(INT1, MOTOR_SPEED); 
  analogWrite(INT2, 0); 
  analogWrite(INT3, MOTOR_SPEED); 
  analogWrite(INT4, 0);
  delay(1000);
}

void left() {  
  analogWrite(INT1, 0); 
  analogWrite(INT2, MOTOR_SPEED); 
  analogWrite(INT3, 0); 
  analogWrite(INT4, MOTOR_SPEED); 
  delay(1000);
}
