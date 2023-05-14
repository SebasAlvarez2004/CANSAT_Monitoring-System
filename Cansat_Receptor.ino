#include <SPI.h> // library SPI from comunication with module
#include <nRF24L01.h>
#include <RF24.h>
#include <ArduinoJson.h>

const int led = 5;
//Nrf24
#define CE_PIN 8
#define CSN_PIN 10
byte direccion[5] ={'c','a','n','a','l'}; //Channel to read
RF24 nrf24(CE_PIN, CSN_PIN);  // bus SPI
//Data Variables
float datos[12];

void setup() 
{
  Serial.begin(115200); // inicializa monitor serie a 9600 bps
  //Led
  pinMode(led, OUTPUT); //Led
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
  //NRF24L01
  nrf24.begin();
  if(!nrf24.begin()){
    Serial.print("No initialized");    
  }
  nrf24.openReadingPipe(1, direccion); //Open to channel lecture
  //Start listening cansat
  nrf24.startListening();
  //Serial.println("Base iniciada");  // texto para no comenzar con ventana vacia
  Serial.println(nrf24.getPALevel());
  nrf24.setDataRate(0); // 2Mbs
  Serial.println(nrf24.getDataRate());
}

void loop()
{  
  uint8_t numero_canal;
  String JsonData;
  if (nrf24.available()) // if exist information available
  { 
    digitalWrite(led, 1);
    //Read data and save in data[] array 
    nrf24.read(datos,sizeof(datos));
    //convert to Json and print___
    DynamicJsonDocument data(JSON_OBJECT_SIZE(12));
        data["Temperature"] = datos[0];
        data["Humidity"] = datos[1];
        data["Pressure"] = datos[2];
        data["DustDensity"] = datos[3];
        data["GasConcentration"] = datos[4];
        data["altitude"] = datos[5];
        data["Accelerometer_x"] = datos[6];
        data["Accelerometer_y"] = datos[7];
        data["Accelerometer_z"] = datos[8];
        data["Gyro_x"] = datos[9];
        data["Gyro_y"] = datos[10];
        data["Gyro_z"] = datos[11];
        /*
        data["CH4"] = ch4;
        data["C2H5OH"] = c2h5oh;
        data["H2"] = h2;
        data["NH3"] = nh3;
        data["CO"] = co; 
        */    
    serializeJson(data, JsonData);
    Serial.println(JsonData);
    
  }
  else{
    //Serial.println("None information");
    digitalWrite(led, 0);
    delay(50);
  }
}
