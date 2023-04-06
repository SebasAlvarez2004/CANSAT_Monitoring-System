//CódeCansat
//Autor : Sebastian Alvarez Palacio
//UNIVERSIDAD EIA 2023

#include <Wire.h>  // Wire library - used for I2C communication
#include <SPI.h>  // incluye libreria SPI para comunicacion con el modulo
#include <RH_NRF24.h> // incluye la seccion NRF24 de la libreria RadioHead
#include <Adafruit_BMP280.h>
#include <AHT10.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
//Buzzer
const int tonePin = 2;
int available = 0;
//General
String str_datos; //Data_String_type
String strTemperature, strHumidity, strPressure, strDustDensity, straccelerometer_x, straccelerometer_y, straccelerometer_z, strgyro_x, strgyro_y, strgyro_z;
//AHT20 + BMP280
Adafruit_BMP280 bmp;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);
float temperature;
float humidity;
float pressure;
//MPU6050
Adafruit_MPU6050 mpu;
float accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
float gyro_x, gyro_y, gyro_z; // variables for gyro raw data
//NRF42L01
RH_NRF24 nrf24(4,5);  
//PM2.5
int measurePin = 34;  
int ledPower = 32;
int samplingtime = 280;
int deltatime = 40;
int sleeptime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity;

void setup() 
{
  Serial.begin(115200);   // inicializa monitor serie a 115200 bps
  //Buzzer
  pinMode(tonePin, OUTPUT);  
  //Nrf24
  if (!nrf24.init())    // si falla inicializacion de modulo muestra texto
    Serial.println("fallo de inicializacion");
  if (!nrf24.setChannel(1)) // si falla establecer canal muestra texto
    Serial.println("fallo en establecer canal");
  if (!nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm)) // si falla opciones 
    Serial.println("fallo en opciones RF");     // RF muestra texto
  //MPU6050
  //Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    //Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    //Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  //Serial.print("Filter bandwidth set to: ");
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
    //Serial.println("5 Hz");
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
  //PM2.5
 pinMode(ledPower,OUTPUT);
  
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
  String JsonData;
  if (nrf24.init() && nrf24.setChannel(2)&& nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm) ){
    //Activate Buzzer
    if(available == 0){
      for (int i =0; i <= 2; i++) {
        digitalWrite(tonePin, HIGH);
        delay(100);
        digitalWrite(tonePin, LOW);
        delay(100);
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
    pressure = bmp.readPressure();
    //PM2.5
    digitalWrite(ledPower,LOW);
    delayMicroseconds(samplingtime);
    voMeasured = analogRead(measurePin);
    delayMicroseconds(deltatime);
    digitalWrite(ledPower,HIGH);
    delayMicroseconds(sleeptime);
    calcVoltage = voMeasured * (5.0/ 1024.0); // 0 - 5V mapped to 0- 1023 integer values
    dustDensity = 170* calcVoltage - 0.1; 
    //Print data
    //Serial.println("Temperature: " + strTemperature + " Humidity: " + strHumidity + " Pressure: " + strPressure + " dustDensity: "+ strDustDensity  + " Ax: "+ straccelerometer_x + " Ay: " + straccelerometer_y + " Az: " + straccelerometer_z + " Gx: " + strgyro_x + " Gy: " + strgyro_y + " Gz: " + strgyro_z);
    //String transform data:
    strTemperature = String(temperature);
    strHumidity = String(humidity);
    strPressure = String(pressure);
    strDustDensity = String(dustDensity);
    straccelerometer_x = String(accelerometer_x);
    straccelerometer_y = String(accelerometer_y);
    straccelerometer_z = String(accelerometer_z);
    strgyro_x = String(gyro_x);
    strgyro_y = String(gyro_y);
    strgyro_z = String(gyro_z);
    //convert to Json and print___
    DynamicJsonDocument data(JSON_OBJECT_SIZE(11));
        data["Temperature"] = temperature;
        data["Humidity"] = humidity;
        data["Pressure"] = pressure;
        data["DustDensity"] = dustDensity;
    serializeJson(data, JsonData);
    Serial.println(JsonData);
    //Transmitter data NRF24___
    str_datos = strTemperature + "," + strHumidity+ "," + strPressure + "," + strDustDensity + "," + straccelerometer_x + "," + straccelerometer_y + "," + straccelerometer_z + "," + strgyro_x + "," + strgyro_y + "," + strgyro_z; // concatena valores separados mediante una coma
    const char *datos = str_datos.c_str(); //String to C
  
    nrf24.send((const uint8_t*)datos, strlen(datos));   //send text
    nrf24.waitPacketSent();   // wait send data
   }else {
    Serial.println("Desconectado...");
    digitalWrite(tonePin, HIGH);
    delay(500);
    digitalWrite(tonePin, LOW);
   }
}
