#include <Arduino.h>         // Base library for Arduino functions
#include <Wire.h>            // Library for I2C communication (SDA/SCL)
#include <vl53l4cd_class.h>  // Specific library for the VL53L4CD distance sensors

// Define the pins used for the I2C bus on the ACEBOTT ESP32
#define SDA_PIN 21           
#define SCL_PIN 22           
#define MUX_ADDR 0x70        // The I2C address of your Multiplexer board

// adress for motor 
#define INT1 16
#define INT2 17
#define INT3 18
#define INT4 19

// Create an instance of the sensor named 'sensor' using the Wire bus
VL53L4CD sensor(&Wire, -1); 

// Function to tell the Multiplexer which "gate" (channel) to open
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);  // Start talking to the Multiplexer
  Wire.write(1 << channel);          // Send a bitmask to select the specific channel (0, 1, or 2)
  Wire.endTransmission();            // Finish command
  delay(5);                          // Short pause to let the hardware switch electrically
}

void setup() {
  // --- FIX ADDED HERE ---
  delay(3000);                       // Wait 3 seconds for sensors to power up!
  // ----------------------

  Serial.begin(115200);              // Start Serial communication at high speed
  delay(2000);                       // Wait 2 seconds to give you time to open the Plotter

  Wire.begin(SDA_PIN, SCL_PIN);      // Join the I2C bus using our defined pins
  Wire.setClock(100000);             // Set the speed of the I2C bus (100kHz is standard)

  // Loop through our 3 sensors to turn them on one by one
  for (uint8_t i = 0; i < 3; i++) {
    selectMuxChannel(i);             // Open the gate for sensor 'i'
    
    // Check if the sensor is actually there and initialize its internal settings
    if (sensor.begin() == 0 && sensor.InitSensor() == 0) {
      sensor.VL53L4CD_StartRanging(); // Tell the sensor to start firing its laser
    }
  }
  pinMode(INT1, OUTPUT); // controls left wheel forward
  pinMode(INT2, OUTPUT); // controls left wheel backwards 
  pinMode(INT3, OUTPUT); // controls right wheel backwards
  pinMode(INT4, OUTPUT); // controls right wheel forwards
}

void loop() {
  uint16_t dists[3] = {0, 0, 0};     // Create a temporary array to hold the 3 distance values

  // Loop to collect data from each sensor
  for (uint8_t i = 0; i < 3; i++) {
    selectMuxChannel(i);             // Switch the Multiplexer to the current sensor
    
    uint8_t isReady = 0;             // Variable to check if a measurement is finished
    VL53L4CD_Result_t results;       // Structure to hold the sensor's data (distance, status, etc.)
    
    // Ask the sensor if it has a new distance reading ready
    sensor.VL53L4CD_CheckForDataReady(&isReady); 

    if (isReady) {
      sensor.VL53L4CD_GetResult(&results);  // Download the measurement into 'results'
      dists[i] = results.distance_mm;       // Save only the distance (in mm) into our array
      sensor.VL53L4CD_ClearInterrupt();     // Tell the sensor it's okay to start the next measurement
    }
  }
  // PRINTING DATA FOR THE SERIAL PLOTTER:
  // The Plotter looks for "Label:Value" separated by commas to draw different colored lines
  
  Serial.print("Left_S0:");          // Label for the first line
  Serial.print(dists[2]);            // The actual number for sensor 0
  Serial.print(",");                 // Comma tells Plotter to stay on the same horizontal point
  
  Serial.print("Center_S1:");        // Label for the second line
  Serial.print(dists[1]);            // The actual number for sensor 1
  Serial.print(",");                 // Another comma
  
  Serial.print("Right_S2:");         // Label for the third line
  Serial.print(dists[0]);            // The actual number for sensor 2
  
  Serial.println();                  // Newline tells the Plotter to move to the next point in time
  
  delay(30);                         // Small pause to keep the graph moving smoothly

  // command to move 
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

// commands for moving 
void forward() {
  analogWrite(INT1, 100); // left forwards at max speed which is 100
  analogWrite(INT2, 0); // so left wont move back wards 
  analogWrite(INT3, 0);
  analogWrite(INT4, 100);
}

void backward() {
  analogWrite(INT1, 0);
  analogWrite(INT2, 100);
  analogWrite(INT3, 100);
  analogWrite(INT4, 0);
}

void stop() {
  analogWrite(INT1, 0);
  analogWrite(INT2, 0);
  analogWrite(INT3, 0);
  analogWrite(INT4, 0);
}
 void right() { // makes right turn in place 
  analogWrite(INT1, 100); // left wheel forward
  analogWrite(INT2, 0); 
  analogWrite(INT3, 100); // right wheel backwards
  analogWrite(INT4, 0);
  delay(1000);
}

void left() {  // makes left turn in place 
  analogWrite(INT1, 0); 
  analogWrite(INT2, 100); // left wheel backwords 
  analogWrite(INT3, 0); 
  analogWrite(INT4, 100); // right wheel forwards
  delay(1000);
}

