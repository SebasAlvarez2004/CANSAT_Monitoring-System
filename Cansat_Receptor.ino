#include <SPI.h>  // incluye libreria SPI para comunicacion con el modulo
#include <RH_NRF24.h> // incluye la seccion NRF24 de la libreria RadioHead
#include <ArduinoJson.h>

const int led = 5;
String str_datos; //Data_String_type
String strTemperature, strHumidity, strPressure, strDustDensity;
String strAX, strAY, strAZ, strGX, strGY, strGZ;

RH_NRF24 nrf24;   // crea objeto con valores por defecto para bus SPI
      // y pin digital numero 8 para CE
void setup() 
{
  Serial.begin(115200); // inicializa monitor serie a 9600 bps
  //Led
  pinMode(led, OUTPUT); //Led
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
  if (!nrf24.init()){    // si falla inicializacion de modulo muestra texto
    Serial.println("fallo de inicializacion");
  } 
  if (!nrf24.setChannel(1)){ // si falla establecer canal muestra texto
    Serial.println("fallo en establecer canal");
  }
  if (!nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm)){ // si falla opciones 
    Serial.println("fallo en opciones RF");             // RF muestra texto
  }
  //Serial.println("Base iniciada");  // texto para no comenzar con ventana vacia
}

void loop()
{
  String JsonData;
  if (nrf24.available())      // si hay informacion disponible
  {     
    digitalWrite(led, HIGH);  
    uint8_t buf[64];      // buffer
    uint8_t buflen = sizeof(buf); // obtiene longitud del buffer
    if (nrf24.recv(buf, &buflen))      // si hay informacion valida en el buffer
    {
      str_datos = String((char*)buf); // almacena en str_datos datos recibidos
      for (int i = 0; i < str_datos.length(); i++) {  // bucle recorre str_datos desde el inicio
        if (str_datos.substring(i, i+1) == ",") { // si en el indice hay una coma
          strTemperature = str_datos.substring(0, i);  // obtiene desde indice 0 hasta una posicion anterior
          strHumidity = str_datos.substring(i+1, i+6); // obtiene desde indice posterior a la coma
          strPressure = str_datos.substring(i+7, i+15);
          strDustDensity = str_datos.substring(i+16, i+23);
          strAX = str_datos.substring(i+24, i+28);
          strAY = str_datos.substring(i+29, i+34);
          strAZ = str_datos.substring(i+35, i+39);
          strGX = str_datos.substring(i+40, i+45);
          strGY = str_datos.substring(i+46, i+50);
          strGZ = str_datos.substring(i+51, i+55);
          break;// hasta el final del string y sale del bucle
          }
        }
      /*
      Serial.print("Temperature = " + strTemperature);
      Serial.print(" Humidity = " + strHumidity);
      Serial.print(" Pressure = " + strPressure);
      Serial.println(" DustDensity = " + strDustDensity);*/
      DynamicJsonDocument data(JSON_OBJECT_SIZE(11));
        data["Temperature"] = strTemperature;
        data["Humidity"] = strHumidity;
        data["Pressure"] = strPressure;
        data["DustDensity"] = strDustDensity;
        data["AcelerationX"] = strAX;
        data["AcelerationY"] = strAY;
        data["AcelerationZ"] = strAZ;
        data["GyroX"] = strGX;
        data["GyroY"] = strGY;
        data["GyroZ"] = strGZ;
      serializeJson(data, JsonData);
      Serial.print(JsonData);
    }
    else// si falla la recepcion
    {
      Serial.println("fallo en recepcion"); // muestra texto
    }
  }
  
}
