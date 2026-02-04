#include <Arduino.h>         
#include <Wire.h>            
#include <vl53l4cd_class.h>  

#define SDA_PIN 21           
#define SCL_PIN 22           
#define MUX_ADDR 0x70        

// --- MATCH USB SPEED ---
// 70 on Battery (7.4V) feels like 100 on USB (5V)
#define MOTOR_SPEED 70

// Motor Pins
#define INT1 16
#define INT2 17
#define INT3 18
#define INT4 19

VL53L4CD sensor(&Wire, -1); 

// Helper to switch Mux
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);  
  Wire.write(1 << channel);          
  Wire.endTransmission();            
  delay(10); // Increased delay slightly for stability
}

// MOVEMENT FUNCTIONS (Defined early so Setup can use them)
void stop() {
  analogWrite(INT1, 0); analogWrite(INT2, 0);
  analogWrite(INT3, 0); analogWrite(INT4, 0);
}

void forward() {
  analogWrite(INT1, MOTOR_SPEED); analogWrite(INT2, 0); 
  analogWrite(INT3, 0);           analogWrite(INT4, MOTOR_SPEED);
}

void right() { 
  analogWrite(INT1, MOTOR_SPEED); analogWrite(INT2, 0); 
  analogWrite(INT3, MOTOR_SPEED); analogWrite(INT4, 0);
  delay(600); // Tuned for battery speed
}

void left() {  
  analogWrite(INT1, 0);           analogWrite(INT2, MOTOR_SPEED); 
  analogWrite(INT3, 0);           analogWrite(INT4, MOTOR_SPEED); 
  delay(600); // Tuned for battery speed
}

void setup() {
  // 1. CRITICAL: STOP MOTORS FIRST
  // This prevents voltage drops from killing the sensors during boot
  pinMode(INT1, OUTPUT); pinMode(INT2, OUTPUT); 
  pinMode(INT3, OUTPUT); pinMode(INT4, OUTPUT); 
  stop(); 

  // 2. WAIT FOR BATTERY TO STABILIZE
  delay(3000);                       

  Serial.begin(115200);              
  Wire.begin(SDA_PIN, SCL_PIN);      
  Wire.setClock(100000);             

  // 3. ROBUST SENSOR INIT (RETRY LOGIC)
  for (uint8_t i = 0; i < 3; i++) {
    selectMuxChannel(i);             
    
    // Try to start the sensor 5 times before giving up
    for (int attempt = 0; attempt < 5; attempt++) {
      if (sensor.begin() == 0 && sensor.InitSensor() == 0) {
        sensor.VL53L4CD_StartRanging(); 
        break; // It worked! Move to next sensor
      }
      delay(50); // Wait a bit and try again
    }
  }
}

void loop() {
  uint16_t dists[3] = {0, 0, 0};     

  // 1. READ DATA
  for (uint8_t i = 0; i < 3; i++) {
    selectMuxChannel(i);             
    
    uint8_t isReady = 0;             
    VL53L4CD_Result_t results;       
    
    sensor.VL53L4CD_CheckForDataReady(&isReady); 

    if (isReady) {
      sensor.VL53L4CD_GetResult(&results);  
      // FILTER: If sensor fails (Status != 0), pretend it's open space (2000mm)
      // This prevents the "Run away" bug if a sensor disconnects
      if (results.range_status == 0) {
        dists[i] = results.distance_mm;
      } else {
        dists[i] = 2000; 
      }
      sensor.VL53L4CD_ClearInterrupt();     
    }
  }
  
  // 2. DEBUG PRINTING
  Serial.print("Left_S0:"); Serial.print(dists[2]); Serial.print(",");
  Serial.print("Center_S1:"); Serial.print(dists[1]); Serial.print(",");
  Serial.print("Right_S2:"); Serial.print(dists[0]); Serial.println();
  
  delay(30);                         

  // 3. YOUR ORIGINAL LOGIC
  // (Note: If sensors fail now, they return 2000, so it won't run away)
  
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
