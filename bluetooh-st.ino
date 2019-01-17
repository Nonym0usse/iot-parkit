#include <Wire.h>
#include <vl53l0x_class.h>

TwoWire WIRE1(PB11, PB10);  //SDA=PB11 & SCL=PB10
VL53L0X sensor_vl53l0x(&WIRE1, PC6, PC7); //XSHUT=PC6 & INT=PC7
 int prise = 0;
  unsigned long time1;
  unsigned long timestamp;

void setup() {

  float starttime = 0;  //Variables to do the math 
 float endtime=0; 
 float resulttime=0, oldresulttime=0; 
  int status;
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  Serial.begin(9600);

  // Initialize I2C bus.
  WIRE1.begin();

  sensor_vl53l0x.VL53L0X_Off();

  status = sensor_vl53l0x.InitSensor(0x10);
  if(status)
  {
    Serial.println("Init sensor_vl53l0x failed...");
  }else{
        Serial.println("OK.");
  }
}


/* Loop ----------------------------------------------------------------------*/

void loop() {

  
  // Led blinking.
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  uint32_t distance;
  int status;
  status = sensor_vl53l0x.GetDistance(&distance);

 if (status == VL53L0X_ERROR_NONE)
  {
    // Output data.
    char report[64];
    snprintf(report, sizeof(report), "| Distance [mm]: %ld |", distance);

    if(distance < 1000){
       time1 = millis();
       timestamp = time1;
       prise = 1;
       
      while(true){
        time1 = millis();
        sensor_vl53l0x.GetDistance(&distance);
        if(distance >= 1000){break;}
      }
      
    }
    int passed_time = time1 - timestamp;
   if(prise == 1 && distance > 1000)
   {
        Serial.println(passed_time);
  prise = 0;
   }

   
  }
  
}
