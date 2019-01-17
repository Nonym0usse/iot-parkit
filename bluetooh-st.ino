#include <Wire.h>
#include <SPI.h>
#include <WiFiST.h>       // WiFi ST microelectonics
#include <vl53l0x_class.h>
#include <PubSubClient.h> // PubSubClient MQTT

// DEBUG
// Configuration du mode DEBUG
#define DEBUG 1   
// Macros
#ifdef DEBUG
 #define DEBUG_PRINT(x)     Serial.print (x)
 #define DEBUG_PRINTDEC(x)  Serial.print (x, DEC)
 #define DEBUG_PRINTHEX(x)  Serial.print (x, HEX)
 #define DEBUG_PRINTLN(x)   Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTHEX(x)
 #define DEBUG_PRINTLN(x) 
#endif



// Create components.
TwoWire WIRE1(PB11, PB10);  //SDA=PB11 & SCL=PB10
VL53L0X sensor_vl53l0x(&WIRE1, PC6, PC7); //XSHUT=PC6 & INT=PC7

unsigned long time1;
unsigned long timestamp;
int passed_time;
/* --- Connexion au serveur Thingsboard --- */



// Wifi ST
SPIClass SPI_3(PC12, PC11, PC10);
WiFiClass WiFi(&SPI_3, PE0, PE1, PE8, PB13);
WiFiClient STClient;

// Données pour le point d'accès Wifi
char ssid[] = "Ame doux lila";  // Le SSID du réseau WiFi à utiliser
char pass[] = "azertyuiop";      // Le mot de passe à utiliser pour le SSID en question

int conn_status = WL_IDLE_STATUS;        // the Wifi radio's status

PubSubClient mqttClient(STClient);
char mqtt_server[] = "172.20.10.6"; // L'adresse IP ou l'URL d'accès au serveur MQTT
const uint16_t mqtt_port = 1883;                  // Le port à utiliser pour le serveur MQTT
#define TOKEN "Xm3r5DO1Ilmt0w4nBezc"              // Le Token d'identification de votre device

void initMqttClient(){
  DEBUG_PRINT("Serveur MQTT : ");
  DEBUG_PRINT(mqtt_server);
  DEBUG_PRINT(" / Port : ");
  DEBUG_PRINTLN(mqtt_port);
  mqttClient.setServer(mqtt_server, mqtt_port);    //Configuration de la connexion au serveur MQTT
}

void initWiFi(){
  // Initialisation du module WiFi
  if (WiFi.status() == WL_NO_SHIELD) {
    DEBUG_PRINTLN("Module WiFi non détécté");
    // don't continue:
    while (true);
  }

  // Vérification de la version du firmware Wifi
  String fv = WiFi.firmwareVersion();
  DEBUG_PRINT("Firmware version: ");
  DEBUG_PRINTLN(fv);
  if (fv != "C3.5.2.5.STM") {
    DEBUG_PRINTLN("Please upgrade the firmware");
  }

  // Tenative(s) de connexion au réseau WiFi
  while (conn_status != WL_CONNECTED) {
    DEBUG_PRINT("Tentative de connexion au réseau WPA/WPA2 network: ");
    DEBUG_PRINTLN(ssid);
    conn_status = WiFi.begin(ssid, pass);
    if (conn_status != WL_CONNECTED) {
      // Connexion au réseau WPA (TKIP) :
      conn_status = WiFi.begin(ssid, pass, ES_WIFI_SEC_WPA);
    }
    // Temps d'attente pour activation de la connexion
    delay(10000);
  }

  // Connexion réussie
  Serial.println("Connected.\nNetwork information:");
  #ifdef DEBUG
    printWiFiStatus();
  #endif
}

void printWiFiStatus() {
  // Affichage du SSID
  DEBUG_PRINT("SSID: ");
  DEBUG_PRINTLN(WiFi.SSID());

  // Affichage de l'adresse IP
  IPAddress ip = WiFi.localIP();
  DEBUG_PRINT("IP Address: ");
  DEBUG_PRINTLN(ip);
  
  // Affichage de l'adresse MAC du module WiFi
  byte mac[6];
  WiFi.macAddress(mac);
  DEBUG_PRINT("MAC address: ");
  for (uint8_t i = 0; i < 6; i++) {
    if (mac[i] < 0x10) {
      DEBUG_PRINT("0");
    }
    DEBUG_PRINTHEX(mac[i]);
    if (i != 5) {
      DEBUG_PRINT(":");
    } else {
      DEBUG_PRINTLN();
    }
  }

  // Affichage du niveau de puissance (RSSI)
  int32_t rssi = WiFi.RSSI();
  DEBUG_PRINT("Signal strength (RSSI): ");
  DEBUG_PRINT(rssi);
  DEBUG_PRINTLN(" dBm");
}



/* Setup ---------------------------------------------------------------------*/

void setup() {
  int status;
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

// Interruption pour déclencher la commande
//  attachInterrupt(digitalPinToInterrupt(userButton), led_blink, FALLING);

  // Initialize serial for output.
  Serial.begin(9600);

  // Initialize I2C bus
  WIRE1.begin();

  // Switch off VL53L0X component.
  sensor_vl53l0x.VL53L0X_Off();

  // Initialize VL53L0X top component.
  status = sensor_vl53l0x.InitSensor(0x10);
  if(status)
  {
    Serial.println("Init sensor_vl53l0x failed...");
  }

 
   // Initialisation de la communication WiFi
  initWiFi();

  // Initialisation de la connexion avec le serveur MQTT
  initMqttClient();
}


/* Loop ----------------------------------------------------------------------*/

void loop() {
  // Vérification de l'état de la connexion WiFi
  
  conn_status = WiFi.status();
  if ( conn_status != WL_CONNECTED) {
    while ( conn_status != WL_CONNECTED) {
      initWiFi();
    }
  }

  // Vérification de l'état de la connexion au serveur MQTT
  if ( !mqttClient.connected() ) {
    reconnect();
  }

  // Lecture et envoi des données
  verifDistance();

  // Boucle de lecture du client MQTT pour voir si des messages sont arrivés
  mqttClient.loop();
}

void verifDistance()
{

  uint32_t distance;
  int status = sensor_vl53l0x.GetDistance(&distance);

  if(status == VL53L0X_ERROR_NONE)
  {

    digitalWrite(LED_BUILTIN, HIGH);
    // Output data.
    char report[64];
    snprintf(report, sizeof(report), "| Distance [mm]: %ld |", distance);
    Serial.println(report);

    if(distance < 1000){
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Relevé en cours...");
       time1 = millis();
       timestamp = time1;
      while(true){
        time1 = millis();
        sensor_vl53l0x.GetDistance(&distance);
        if(distance >= 1000){break;}
      }
      Serial.println("Terminé!");
      int passed_time = time1 - timestamp;
      char result[64];
      snprintf(result, sizeof(result), "Temps passé : %ld ms", passed_time);
      Serial.println(result);
      
      String payload = "{";
      payload += "\"distance\":"; payload += distance; payload += ",";
      payload += "\"time\":"; payload += passed_time;
      payload += "}";

      char attributes[100];
      payload.toCharArray(attributes, 100 );
      mqttClient.publish( "v1/devices/me/telemetry", attributes );
    }
  } 
}

void reconnect() {
  while (!mqttClient.connected()) {
    DEBUG_PRINT("Connexion au serveur ThingsBoard ...");
    if ( mqttClient.connect("STM32", TOKEN, NULL) ) {
      DEBUG_PRINTLN( "[OK]" );
    } else {
      DEBUG_PRINT( "[ERREUR] [ rc = " );
      DEBUG_PRINT( mqttClient.state() );
      DEBUG_PRINTLN( " : Nouvelle tentative dans 5 sec]" );
      delay(5000);
    }
  }
}
