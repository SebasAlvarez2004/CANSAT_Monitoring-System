//CódeCansat
//Autor : Sebastian Alvarez Palacio
//UNIVERSIDAD EIA 2023

#include <Wire.h>  // Wire library - used for I2C communication
#include <SPI.h>  
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_BMP280.h>
#include <AHT10.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <DFRobot_MICS.h>
#define CALIBRATION_TIME   3  // Default calibration time is three minutes for MICS5524

//Buzzer
const int tonePin = 27; //From buzzer
int available = 0; //from active buzzer
//General
String str_datos; //Data_String_type
String strTemperature, strHumidity, strPressure, strDustDensity, straccelerometer_x, straccelerometer_y, straccelerometer_z, strgyro_x, strgyro_y, strgyro_z;
//AHT20 + BMP280
Adafruit_BMP280 bmp;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);
float temperature;
float humidity;
float pressure;
float altitude;
float P0; //Calc Atitude level floor
//MPU6050
Adafruit_MPU6050 mpu;
float accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
float gyro_x, gyro_y, gyro_z; // variables for gyro raw data
//NRF42L01
#define CE_PIN 4
#define CSN_PIN 5
RF24 nrf24(CE_PIN, CSN_PIN); 
byte direccion[5] ={'c','a','n','a','l'}; //Direccion of channel comunication
//PM2.5
int measurePin = 34;  
int ledPower = 32;
int samplingtime = 280;
int deltatime = 40;
int sleeptime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity;
//GasConcentration//
#define MICS5524 35
#define POWER_PIN 16 
DFRobot_MICS_ADC mics(/*adcPin*/MICS5524,/*powerPin*/POWER_PIN);
float GasConcentration;
 /**!
    Type of detection gas
    MICS-4514 You can get all gas state
    MICS-5524 You can get the state of CO, CH4, C2H5OH, C3H8, C4H10, H2, H2S, NH3
    MICS-2714 You can get the state of NO2, H2 ,NO
      CO       = 0x01  (Carbon Monoxide)
      CH4      = 0x02  (Methane)
      C2H5OH   = 0x03  (Ethanol)
      C3H8     = 0x04  (Propane)
      C4H10    = 0x05  (Iso Butane)
      H2       = 0x06  (Hydrogen)
      H2S      = 0x07  (Hydrothion)
      NH3      = 0x08  (Ammonia)
      NO       = 0x09  (Nitric Oxide)
      NO2      = 0x0A  (Nitrogen Dioxide)
  */
   /**!
    Gas type for lectures in ppm:
    MICS-4514 You can get all gas concentration
    MICS-5524 You can get the concentration of CH4, C2H5OH, H2, NH3, CO
    MICS-2714 You can get the concentration of NO2
      Methane          (CH4)    (1000 - 25000)PPM
      Ethanol          (C2H5OH) (10   - 500)PPM
      Hydrogen         (H2)     (1    - 1000)PPM
      Ammonia          (NH3)    (1    - 500)PPM
      Carbon Monoxide  (CO)     (1    - 1000)PPM
      Nitrogen Dioxide (NO2)    (0.1  - 10)PPM
  */
void setup() 
{
  Serial.begin(115200);   // inicializa monitor serie a 115200 bps
  //Buzzer
  pinMode(tonePin, OUTPUT);  
  digitalWrite(tonePin, HIGH);
  //Nrf24
  nrf24.begin();    
  
  nrf24.openWritingPipe(direccion);
  Serial.println(nrf24.getPALevel());
  Serial.println(nrf24.getDataRate());
  Serial.println(nrf24.getCRCLength());
  //MPU6050
  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  //BMP280 +AHT20  
  while (myAHT20.begin() != true) {
    Serial.println(F("AHT20 not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    delay(5000);
  }
  //Serial.println(F("AHT20 OK"));
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  P0 = bmp.readPressure()/100; // Altitud = 0 && Read initial pressure in hPa 
  //PM2.5
  pinMode(ledPower,OUTPUT);
  //MICS5524
  while(!mics.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }
  /*while(!mics.warmUpTime(CALIBRATION_TIME)){
    //Serial.println("Please wait until the warm-up time is over!");
    delay(1000);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /* Operating Mode. */
  Adafruit_BMP280::SAMPLING_X2, /* Temp. oversampling */
  Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
  Adafruit_BMP280::FILTER_X16, /* Filtering. */
  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

 //Serial.println("Transmisor Cansat iniciado");
}

void loop() {
  //Json format
  //String JsonData;
  float datos[12]; //Data array
  if (nrf24.begin()){
    //Activate Buzzer
    if(available == 0){
      for (int i =0; i <= 2; i++) {
        digitalWrite(tonePin, HIGH);
        delay(100);
        digitalWrite(tonePin, LOW);
        delay(100);
        digitalWrite(tonePin, HIGH); //Off Buzzere
      } 
      available ++;
    }
    //MPU6050
     /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
     // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    accelerometer_x = a.acceleration.x; // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accelerometer_y = a.acceleration.y; // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accelerometer_z = a.acceleration.z; // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    //U : m/s2
    gyro_x = g.gyro.x; // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    gyro_y = g.gyro.y; // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    gyro_z = g.gyro.z; // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
    //U : rad/s
    //BMP280+AHT20
    temperature = myAHT20.readTemperature();
    humidity = myAHT20.readHumidity();
    pressure = bmp.readPressure()* 0.01;//Lecture in hPa Range: (0 - 1200 hPa) // for lectures in Pa delete : (* 0.01)
    //Alt to level floor: In m
    //altitude = bmp.readAltitude(P0);
    altitude = bmp.readAltitude(841.51511);//Read altitude respect level Sea //MEDELLÍN
    //PM2.5
    digitalWrite(ledPower,LOW); //Turn ON the led
    delayMicroseconds(samplingtime); // wait 0.28 ms = 280 us
    voMeasured = analogRead(measurePin); //measure the peak of the output pulse
    delayMicroseconds(deltatime); // wait 40 us
    digitalWrite(ledPower,HIGH); //turn OFF the led
    delayMicroseconds(sleeptime); //Delay sleep time
    calcVoltage = voMeasured * (5.0/ 4095.0); // 0 - 5V mapped to 0- 4095 integer values with ESP32
    dustDensity = 170* calcVoltage - 0.1; //in ug/m3
    //MICS5524
    /*
    float ch4 = mics.getGasData(CH4);
    float c2h5oh = mics.getGasData(C2H5OH);
    float h2 = mics.getGasData(H2);
    float nh3 = mics.getGasData(NH3);
    float co = mics.getGasData(CO);
    */
    float gasread = analogRead(35); // gas concentration
    GasConcentration = gasread/4098 * 100;
    //Download datos in array
    datos[0] = temperature;
    datos[1] = humidity;
    datos[2] = pressure;
    datos[3] = dustDensity;
    datos[4] = GasConcentration;
    datos[5] = altitude;
    datos[6] = accelerometer_x;
    datos[7] = accelerometer_y;
    datos[8] = accelerometer_z;
    datos[9] = gyro_x;
    datos[10] = gyro_y;
    datos[11] = gyro_z;

    //Send data to base
    bool ok = nrf24.write(datos, sizeof(datos));
    if(ok)
      {
        Serial.print("Datos enviados "); 
        Serial.print(datos[0]); 
        Serial.print(" , "); 
        Serial.print(datos[1]); 
        Serial.print(" , ");
        Serial.print(datos[2]); 
        Serial.print(" , "); 
        Serial.print(datos[3]); 
        Serial.print(" , ");
        Serial.print(datos[4]); 
        Serial.print(" , "); 
        Serial.print(datos[5]); 
        Serial.print(" , ");
        Serial.print(datos[6]); 
        Serial.print(" , "); 
        Serial.print(datos[7]); 
        Serial.print(" , ");
        Serial.print(datos[8]); 
        Serial.print(" , "); 
        Serial.print(datos[9]); 
        Serial.print(" , ");
        Serial.print(datos[10]); 
        Serial.print(" , "); 
        Serial.println(datos[11]); 
      }else
      {
       Serial.println("no se ha podido enviar");
      }
   }else {
    Serial.println("Disconected...");
    
    digitalWrite(tonePin, HIGH);
    delay(500);
    digitalWrite(tonePin, LOW);
   }
}
