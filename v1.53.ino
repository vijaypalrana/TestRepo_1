/*! \mainpage ZeroVir Firmware Document v1.50

   \section intro_sec Introduction

   This document contain the list of all API's that are used in this project.
   1. API for Wifi Connectivity.
   2. API for Server Connectivity.
   3. API for Sending Data to Server.
   4. API for Receiving Data from Server.
   5. API for Storing Data in Memory.
   6. API for Device OTA firmware upgrade.
   7. Need to check Wifi reconnection logic
   8. Need to add crc check in receiving and transmitting data.

   Copyright (c) : Lithion Power Pvt. Ltd.

*/
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ESP8266WiFiMulti.h>
#include <FS.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Ticker.h>

#define Debug 0x01              /**< Debug logs */
#define SLAVE_ADDR 0xA0         /**< EEPROM Slave Address */
#define WIFI_CONNECT_RETRIES 1  /**< Maximum connection retry count for Wifi Connection*/
#define WIFI_RETRY_DELAY 1000   /**< Delay value for Wifi connectioin retry*/
#define SERVER_RETRY_DELAY 1000 /**< Delay value for server connection retry*/
#define WIFI_RETRIES 5          /**Maximum connection retry for Wifi connect retries */
#define TIME_CLIENT_RETRY 10
#define MAXIMUM_VALUE 1 /**< Maximum connection retries for server*/


#define Relay 4         /**< Relay toggle pin*/
#define PIR1 5          /**< PIR sensor 1 input*/
////////////////////////#define PIR2    14          /**< PIR sensor 2 input*/
#define PIR3 13 /**< PIR sensor 3 input*/
///////////////////////#define PIR4    12      /**< PIR sensor 4 input*/
#define PIR4_AS_PWR_CTR    12           // tseting purpose using PIR4 as PIR power control
#define LED                0         /**< Led toggle pin*/
#define LED2               15         /**< Led toggle pin*/


#define SCL 14 /**< I2C SCL pin 14 CLK*/
#define SDA 2  /**< I2C SDA pin 2 DATA*/

#define Buzzer 16         /**< Buzzer output*/
#define SSID1_ADDR 0x0000 /**< SSID 1 Storage address*/
#define PASS1_ADDR 0x001F /**< PASS 1 Storage address*/
#define DATA_ADDR 0x003F  /**< Data Storage Address*/
#define CRC_GLOBAL 0x004F /**< CRC address for global data*/
#define MAC_SIZE 12       /**< MAC address size in char*/
#define OTA_ADDR 0x0052   /**< OTA handler address*/
#define EVENT_START_ADDR 0x0060
#define EVENT_END_ADDR 0x0200
#define EVENT_SIZE sizeof(uint32_t) + sizeof(uint8_t)

#define NO_OF_PIR_SENSOR 3 /**< No of PIR sensors attached*/

#define PIR_Polling_Or_Interrupt 0x01 /**< Debug logs */

/**
   @brief  Device Modes
*/
typedef enum Modes
{
    FAULT_MODE = 4,
    SELF_DIA_MODE = 3,
    AUTO_MODE = 2,
    MANUAL_MODE = 1,
    SAFE_MODE = 0,
};

/** Time in seconds*/
#define SEC_MULTI 1000
#define MIN_MULTI 60000

struct events
{
    uint32_t event_time;
    uint8_t event_type;
};

struct events events_t;

/**
   @brief  Device States
*/
typedef enum Device_State
{
    DEVICE_INIT_START = 101,
    DEVICE_INIT_SUCCESS = 102,
    STATE_ONDEMAND = 103,
    STATE_AUTOMODE = 104,
    OBSERVING_ENV = 105,
    SANITIZATION_ON = 106,
    EXPOSER_OVEFLOW = 108,
    SLEEPING = 109,
    STATE_OBSER_FORCED_STOP = 107,
    DAILY_OP_LIMIT_EXCEED = 110,
    ON_DEMAND_START_CALLED_WHILE_OBSER = 111,
    ON_DEMAND_STOP = 112,
    SELF_DIA_STATE = 113,
    FAULT_STATE = 114,
};

/**
   @brief  Device Acknowledgements
*/
typedef enum Acknowledgements
{
    SUCCESS_SANITIZATION = 200,
    DEVICE_INIT_START_ACK = 201, // not in use
    ON_DEMAND_START = 202,
    ACK_ON_DEMAND_DEEP_CLEAN = 203,
    ACK_SANITIZATION_ATTEMPT = 204,
    RESETTING_START = 205,    // not in use
    RESETTING_COMPLETE = 206, // not in use
    OBSERVATION_FORCED_STOP = 207,
    STARTING_SANITIZATION = 208,
    NO_MOTION_TIMEOUT = 209,                      // not in use
    REAL_TIME_SYNC_START = 210,                   // not in use
    REAL_TIME_SYNC_SUCCESS = 211,                 // not in use
    ON_DEMAND_START_CALLED_WHILE_OBSER_ACK = 212, // not in use
    OBSERVATION_ABORTED = 213,
    SANITIZATION_ABORTED_ACK = 214,
    SANITIZATION_FORCED_STOP_ACK = 215,
    RESET_LAST_SANITIZATION_TIME_ACK = 216, // not in use
    PIR_OK_ACK = 217, // not in use
    PIR_ERROR_ACK = 218, // not in use
    LAMP_OK_ACK = 219, // not in use
    LAMP_ERROR_ACK = 220
};

/**
   @brief  Device Parameters
*/
struct GlobalData
{
    uint8_t BULB_ON_DURATION;            // in minutes
    uint8_t DEEP_CLEAN_BULB_ON_DURATION; // in minutes
    uint8_t AUTO_MODE_BULB_ON_DURATION;  // in minutes
    uint8_t OBSERVATION_TIME;            // in seconds
    uint8_t PRE_BUZZER_ON_TIMER;         // in seconds
    uint8_t SLEEP_TIMER;                 // in minutes
    uint8_t DAILY_SANITIZATION_LIMIT;    // in minutes
    uint16_t AUTO_OP_RESET;              // in minutes
    uint16_t AUTO_MODE_NO_MOTION;        // in minutes
    uint8_t dummy_var;
    uint16_t events_current_address;
};

/**
   @brief  Setting Parameters to default
*/

struct GlobalData globalData =
    {
        .BULB_ON_DURATION = 1,                    // in minutes   2
        .DEEP_CLEAN_BULB_ON_DURATION = 1,         // in minutes   2
        .AUTO_MODE_BULB_ON_DURATION = 1,          // in minutes   2
        .OBSERVATION_TIME = 30,                   // in seconds   60
        .PRE_BUZZER_ON_TIMER = 5,                // in seconds    5
        .SLEEP_TIMER = 1,                         // in minutes   1
        .DAILY_SANITIZATION_LIMIT = 60,           // in minutes   60
        .AUTO_OP_RESET = 1440,                    // in minutes   1440
        .AUTO_MODE_NO_MOTION = 1, // testing purpose              3
        .dummy_var = 90,
        .events_current_address = EVENT_START_ADDR,
};

#define AUTO_MODE_ACTIVE /** Device Default Mode*/

ESP8266WiFiMulti WiFiMulti; /** Multi Wifi Object creation*/
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

/**
   @brief  Device Packet Structure : 1st byte - Data Identifier | 2nd Byte - Data length | 3rd byte - Data bytes as per the length
*/

/**
   @brief  Device Data downlink Packet Identifier
*/
enum Downstream
{
    CONTROL = 1,
    STATUS_DEVICE,
    PARAMETERS,
    OTA,
    SSID_USER,
    EVENTS,
};

/**
   @brief  Device Parameter Set Parameter
*/
enum parameters
{
    BDUR = 1,
    DEEP_CLN_BDUR,
    AUTO_MD_BDUR,
    OB_TIME,
    PRE_BZZ_OTIME,
    SLEEP_TMR,
    DAILY_SN_LMT,
    AUTO_OP_RST,
    AUTO_MD_NMTN,
    SEND_PMTRS,
};

/**
   @brief  Device Data uplink packet idetifers
*/
enum Upstream
{
    Ping = 1,
    Status_upstream,
    Parameters_upstream,
    Acknowledgement,
    Events,
};

/**
    Internet Realted Error - 00
    Status realted - 01
    Acknowledgement type - 10
    RFU - 11
*/

/**
   @brief  API used to connect with Wifi network
   @retval None
*/
bool connect_wifi();

/**
   @brief  API used to check wifi connection status
   @retval bool : true/false
*/
bool check_wifi();

/**
   @brief  API used to connect with Server
   @retval bool : true/false
*/
bool connectToServer();

/**
   @brief  API used to send data data to MQTT server
   @param  *data    : uint8_t : pointer to the data
   @param  len      : uint8_t : length of data
   @retval uint8_t  : server response
*/
uint32_t send_data_to_server(const char *topic, uint8_t *data, uint32_t len);

/**
   @brief  API used to enable GPIO Peripherals
   @retval None
*/
void enableGpioPeripherals();

/**
   @brief  Callback routine for the downlink received from MQTT server
   @param  topic    : char *  : topic on which downlink received
   @param  payload  : byte *  : pointer to downlink data
   @param  len      : int     : length of data
   @retval None
*/
void callback(char *topic, byte *payload, unsigned int len);

/**
   @brief  API used to read ssid password stored in memory
   @retval None
*/
void read_ssid_password();

/**
   @brief  API used to store ssid and password data in memory
   @param  *data  : char    : pointer to ssid data
   @param  *data1 : char    : pointer to password data
   @param  len    : uint8_t : ssid data length
   @param  len1   : uint8_t : password data length
   @retval None
*/
void store_ssid_password(char *data, char *data1, uint8_t len, uint8_t len1);

/**
   @brief  API used for Device OTA
   @param  ver1 : uint8_t : MSB of firmware version
   @param  ver2 : uint8_t : LSB of firmware vresion
   @retval None
*/
bool enable_OTA(uint8_t ver1, uint8_t ver2);

/**
   @brief  API used to convert the ASCII data received in HEX format
   @param  *data        : uint8_t : pointer to the ASCII data
   @param  len          : uint8_t : ASCII data length
   @param  *decode_data : uint8_t : pointer to the HEX data
   @retval None
*/
void data_decode(uint8_t *data, uint8_t len, uint8_t *decode_data);

/**
   @brief  API used the parse the SSID data come from server
   @param  *data  : uint8_t : SSID and Password data received from server
   @retval None
*/
void set_custom_ssid(uint8_t *data);

/**
   @brief  API used to get the ASCII data for given HEXADECIMAL
   @param  c  : uint8_t : HEXADECIMAL Data
   @retval uint8_t : ASCII Char
*/

uint8_t hexToChar(uint8_t c);
/**
   @brief  API used to get the HEX data for given ASCII Char
   @param  c  : char : ASCII Char
   @retval uint8_t : HEX data
*/
uint8_t charToHex(char c);

/**
   @brief  API used to send ping to the server
   @retval None
*/
void send_ping();

/**
   @brief  API used to send the device status to server
   @retval None
*/
void send_device_status();

/**
   @brief  API used to send device parameters to the server
   @retval None
*/
void send_parameters_server();

/**
   @brief  API used to send acknowledgement to the server
   @param  *data  : uint8_t : acknowledgement data
   @param  len    : uint8_t : length of acknowledgement data
   @retval None
*/
void send_acknowledgement(uint8_t *data, uint8_t len);

/**
   @brief  API used to save device parameters in memory.
   @retval None
*/
void saveCurrentConfig();

/**
   @brief  API used to generate CRC data
   @param  *data  : uint8_t : pointer to the data
   @param  len    : size_t  : data length
   @retval uint8_t: crc8 value
*/
uint8_t gencrc(uint8_t *data, size_t len);

/**
   @brief  API used to save CRC8 data in memory
   @retval None
*/
void StoreCRCInMemory();

/**
   @brief  API used to read CRC8 in memory
   @retval uint8_t : CRC8
 * */
uint8_t readCRCFromMemory();

/**
   @brief  API used to Init the Device configuration from memory
   @retval None
*/
void deviceConfigInit();

/**
   @brief  API used to check if device need OTA update or not
   @retval None
*/
void OTA_check();

/**
   @brief  API used to save OTA configuration in memory
   @param  *data    : uint8_t : pointer to the OTA data
   @param  save_data: bool    : save device config in memory
   @retval None
*/
void OTA_save(uint8_t *data, bool save_data);

/**
   @brief  API def for unit programing
   @retval None
*/
void UNIT_PROGRAM();
/**
   @brief  API used to toggle the buzzer state
   @param  state  : bool : enable/disable
   @retval None
*/
void buzzer_toggle(bool state);

/**
   @brief  API used to toggle UV bulb state
   @param  state  : bool : enable/disable
   @retval None
*/
void light_toggle(bool state);
void light_state(bool state);
void create_event(uint8_t event_type);
uint8_t pir_sensor_output();
void send_events_data();
void read_data_from_memory(uint8_t *data, uint32_t len);
void stopAfterPIR_interrupt();
void clearCounter();

uint8_t Is_uv_lamps_ok();
void toggle_pir_power();
void buzzerTonediagnoseTime();

uint8_t fm_version[2] = {0x01, 0x32};                                         /** Device Firmware version*/
const char *ssid = "VPSR-wifi";                                                 /** Device Default SSID*/
const char *pass = "test12345";                                             /** Device Default Password*/
const char *AWS_endpoint = "a2d3l80c08c1jp-ats.iot.ap-south-1.amazonaws.com"; /** Default MQTT server*/
uint16_t mqtt_port = 8883;                                                    /** Default MQTT port*/
const char *topic_ack = "zerovir/inward/ack";                                 /** Default MQTT upstream topic*/
const char *topic_param = "zerovir/inward/param";                             /** Default MQTT upstream topic*/
const char *topic_ping = "zerovir/inward/ping";                               /** Default MQTT upstream topic*/
const char *topic_status = "zerovir/inward/status";
uint8_t WiFiMac[6];                           /** Store Wifi MAC address*/
bool send_status = false;                     /** Flag for sending device status*/
bool send_parameters = false;                 /** Flag for sending device parameters*/
volatile bool pir_interrupt_received = false; /** Flag for logging interrupt status*/
volatile uint8_t ack_stop_interrupt = 0;
volatile uint8_t ack_stop_interrupt_data[5] = {0};
uint8_t ack_pir_status = 0;


WiFiClientSecure espClient;
PubSubClient client(AWS_endpoint, 8883, callback, espClient); //set MQTT port number to 8883 as per //standard

/**
   varibales for unit program
*/
uint8_t isDeviceOperating = 0;
uint8_t gDeviceState = DEVICE_INIT_START;
// DeviceMode 1 is manual mode,DeviceMode 2 is Auto mode,DeviceMode 0 is reseting mode
uint8_t gDeviceMode = AUTO_MODE;
uint8_t gPIRSensingFlag = 0;
// starting with 9 as if device starts in auto mode it will go directly into on demand flag
uint8_t gOnDemandDeepCleanFlag = 9;
uint8_t gPreBuzzerFlag = 0;
uint8_t gStartSanitizationFlag = 0;
uint32_t gOBSERVATION_TIME;
uint32_t gBULB_ON_DURATION;
uint8_t gCurrentlyObservingFlag = 0;
uint32_t nBulbOnTimeCounter = 0;
uint32_t nSleepTimeCounter = 0;
uint32_t nOnCommandStartCounter = 0;
uint8_t printFlag = 0;
volatile uint8_t pir1_interruptCounter = 0;
volatile uint8_t pir2_interruptCounter = 0;
volatile uint8_t pir3_interruptCounter = 0;
volatile uint8_t pir4_interruptCounter = 0;

volatile uint8_t pir_interrupt_digital_filter_count = 1;

volatile uint8_t dia_pir_int_count_1 = 0;
volatile uint8_t dia_pir_int_count_2 = 0;
volatile uint8_t dia_pir_int_count_3 = 0;
volatile uint8_t dia_pir_int_count_4 = 0;
volatile uint8_t pir_check = 0;
uint8_t pirCheckRetry = 2;
unsigned long prevTime = 0;
unsigned long currTime = 0;
unsigned long interval = 10000;
volatile uint8_t self_diagnose_completed = 0;
volatile uint8_t self_diagnose_time_count = 0;
volatile uint8_t self_diagnose_inprogress = 0;
//uint8_t self_diagnose_prebuzzer_check = 0;
volatile uint8_t wifi_connected_or_not = 0;
volatile uint8_t dont_check_internet_if_no_internet = 0; //dont_check_internet_if_no_internet




Ticker timerPIR;
Ticker timerBuzzer;
Ticker timerled2;
Ticker timerDiagnose;

/**
   @brief  API initially called for setting device settings
   @retval None
*/
void setup()
{
    WiFi.mode(WIFI_STA);
    espClient.setBufferSizes(512, 512); //ok
    enableGpioPeripherals();            //ok
    enablePIR_Interrupts();
    EEPROM.begin(512);                  //ok
    Serial.begin(115200);               //ok
    WiFiMulti.addAP(ssid, pass);        //ok
    deviceConfigInit();
    read_ssid_password();
    loadCertificates();
    WiFi.macAddress(WiFiMac);
    WiFi.setOutputPower(20);
    Serial.println("\r\nDevice Active");
    if (!client.connected())
    {
        
        Serial.println("First time connection for OTA");
        connectToServer();
    }

    OTA_check();
    send_ping();
    send_parameters_server();
}

/**
   @brief  API that run continuously and handle all device performance
   @retval None  
*/
void loop()
{

    if (!client.connected() && self_diagnose_inprogress == 0 && dont_check_internet_if_no_internet == 0 && gDeviceState != OBSERVING_ENV)
    {
        Serial.println("Loop Entry,if client not connected");
        connectToServer();
    }
    
    if(wifi_connected_or_not == 1 && self_diagnose_inprogress == 0 && dont_check_internet_if_no_internet == 0 && gDeviceState != OBSERVING_ENV)
    {
      client.loop();
    }

//    if (gStartSanitizationFlag == 1 && self_diagnose_completed == 1 && ack_pir_status == PIR_OK_ACK)
//    {
//      digitalWrite(LED2, LOW);
//      delay(250);
//      digitalWrite(LED2, HIGH);
//      delay(250);
//    }
    
    UNIT_PROGRAM();

    if (send_status /*&& client.connected()*/)
    {
        send_status = false;
        send_device_status();
    }

    if (send_parameters /*&& client.connected()*/)
    {
        send_parameters = false;
        send_parameters_server();
    }

     currTime = millis();
      if(currTime - prevTime > interval)
      {
        if(LAMP_OK_ACK == recheck_uv_lamps_ok() && gDeviceState != SANITIZATION_ON)
        {
         
            Serial.println("This is Relay issue (Hardware Issue :D)...");
            light_toggle(0);
            
         }
         prevTime = currTime;
      }

    
}

void loadCertificates()
{
    SPIFFS.begin();

    File cert = SPIFFS.open("/cert.der", "r"); //replace cert.crt with your uploaded file name
    if (!cert)
        Serial.println("Failed to open cert file");
    delay(1000);
    if (!espClient.loadCertificate(cert)) // add the thing certificate to the client
        Serial.println("cert not loaded");

    // Load private key file
    File private_key = SPIFFS.open("/private.der", "r"); //replace private with your uploaded file name
    if (!private_key)
        Serial.println("Failed to open private cert file");
    delay(1000);
    if (!espClient.loadPrivateKey(private_key)) // add the private key to the client
        Serial.println("private key not loaded");

    // Load CA file
    File ca = SPIFFS.open("/ca.der", "r"); //replace ca with your uploaded file name
    if (!ca)
        Serial.println("Failed to open ca ");
    delay(1000);
    if (!espClient.loadCACert(ca)) // add the AWS root certificate to the client
        Serial.println("ca not loaded");
#if Debug == 1
    Serial.print("Heap: ");
    Serial.println(ESP.getFreeHeap());
#endif
}

void deviceConfigInit()
{
    Serial.println("Config Init");
    uint8_t crc_check = 0, crc_current = 0;

    crc_current = gencrc((uint8_t *)&globalData, sizeof(globalData));
    crc_check = readCRCFromMemory();
    crc_current = gencrc((uint8_t *)&globalData, sizeof(globalData));
    if (crc_current != crc_check)
    {
        Serial.printf("crc_calculated = %d crc_found = %d , mismatch in globalData Structure, Saving configuration to default", crc_current, crc_check);
    }
    else
    {
        Serial.printf("crc_calculated = %d crc_found = %d , match in globalData Structure, configuration load from memory", crc_current, crc_check);
        read_data_from_memory((uint8_t *)&globalData, sizeof(globalData));
    }
}

/**
   @brief  ISR for external interrupts received
   @retval None
*/

void ICACHE_RAM_ATTR pir1_handleInterrupt()
{
    pir1_interruptCounter++;
    dia_pir_int_count_1++;
    if(self_diagnose_inprogress == 0)
    {
      dia_pir_int_count_1 = 0;
    }
    Serial.printf("PIR 1 interrupt received!  %d", pir1_interruptCounter);
    Serial.println("");
    if (pir1_interruptCounter >= pir_interrupt_digital_filter_count && (gDeviceState == OBSERVING_ENV || gDeviceState == SANITIZATION_ON) && pir_check == 0)
    {
        Serial.print("Device State : ");
        Serial.println(gDeviceState);
        pir_interrupt_received = true;
        ack_stop_interrupt = 1;
        stopAfterPIR_interrupt();
        if(self_diagnose_inprogress == 1)
       {
        timerBuzzer.detach();
        timerDiagnose.detach();
       }
        
        light_toggle(0);
        buzzer_toggle(0);
        timerled2.detach();
        digitalWrite(LED2, HIGH);
        
        
        self_diagnose_completed = 0;
        pirCheckRetry = 2;

        Serial.println("PIR 1 interrupt accepted!");
        pir1_interruptCounter = 0;
    }
    else
    {
        pir_interrupt_received = false;
        ack_stop_interrupt = 0;
    }
}



void ICACHE_RAM_ATTR pir2_handleInterrupt()
{
  pir2_interruptCounter++;
  dia_pir_int_count_2++;
  if(self_diagnose_inprogress == 0)
  {
    dia_pir_int_count_2 = 0;
  }
  Serial.printf("PIR 2 interrupt received!  %d", pir2_interruptCounter);
  Serial.println("");
  if (pir2_interruptCounter >= pir_interrupt_digital_filter_count && (gDeviceState == OBSERVING_ENV || gDeviceState == SANITIZATION_ON) && pir_check == 0)
  {
        pir_interrupt_received = true;
        ack_stop_interrupt = 1;
        stopAfterPIR_interrupt();
        if(self_diagnose_inprogress == 1)
       {
        timerBuzzer.detach();
        timerDiagnose.detach();
       }
        light_toggle(0);
        buzzer_toggle(0);
        timerled2.detach();
        digitalWrite(LED2, HIGH);
        
        self_diagnose_completed = 0;
        pirCheckRetry = 2;
        Serial.println("PIR 2 interrupt accepted!");
        pir2_interruptCounter = 0;
  }
  else
  {
    pir_interrupt_received = false;
    ack_stop_interrupt = 0;
  }
}



void ICACHE_RAM_ATTR pir3_handleInterrupt()
{
  pir3_interruptCounter++;
  dia_pir_int_count_3++;
  if(self_diagnose_inprogress == 0)
  {
    dia_pir_int_count_3 = 0;
  }
  Serial.printf("PIR 3 interrupt received!   %d", pir3_interruptCounter);
  Serial.println("");
  if (pir3_interruptCounter >= pir_interrupt_digital_filter_count && (gDeviceState == OBSERVING_ENV || gDeviceState == SANITIZATION_ON) && pir_check == 0)
  {
        pir_interrupt_received = true;
        ack_stop_interrupt = 1;
        stopAfterPIR_interrupt();
        if(self_diagnose_inprogress == 1)
       {
        timerBuzzer.detach();
        timerDiagnose.detach();
       }
        light_toggle(0);
        buzzer_toggle(0);
        timerled2.detach();
        digitalWrite(LED2, HIGH);

        self_diagnose_completed = 0;
        pirCheckRetry = 2;
        Serial.println("PIR 3 interrupt accepted!");
        pir3_interruptCounter = 0;
  }
  else
  {
    pir_interrupt_received = false;
    ack_stop_interrupt = 0;
  }
}




void ICACHE_RAM_ATTR pir4_handleInterrupt()
{
  pir4_interruptCounter++;
  dia_pir_int_count_4++;
  if(self_diagnose_inprogress == 0)
  {
    dia_pir_int_count_4 = 0;
  }
  Serial.printf("PIR 4 interrupt received!   %d", pir4_interruptCounter);
  Serial.println("");
  if (pir4_interruptCounter >= pir_interrupt_digital_filter_count && (gDeviceState == OBSERVING_ENV || gDeviceState == SANITIZATION_ON) && pir_check == 0)
  {
        pir_interrupt_received = true;
        ack_stop_interrupt = 1;
        stopAfterPIR_interrupt();
        if(self_diagnose_inprogress == 1)
       {
        timerBuzzer.detach();
        timerDiagnose.detach();
       }
        light_toggle(0);
        buzzer_toggle(0);
        timerled2.detach();
        digitalWrite(LED2, HIGH);

        self_diagnose_completed = 0;
        pirCheckRetry = 2;
        Serial.println("PIR 4 interrupt accepted!");
        pir4_interruptCounter = 0;
  }
  else
  {
    pir_interrupt_received = false;
    ack_stop_interrupt = 0;
  }
}


void enableGpioPeripherals()
{
    uint8_t pir_sensor_count = 0;
    pir_sensor_count = NO_OF_PIR_SENSOR;

    pinMode(Relay, OUTPUT);
    pinMode(Buzzer, OUTPUT);
    pinMode(PIR1, INPUT);
    //pinMode(PIR2, INPUT);
    pinMode(PIR3, INPUT);
    
    pinMode(PIR4_AS_PWR_CTR, OUTPUT);
    digitalWrite(PIR4_AS_PWR_CTR, HIGH);
    
    pinMode(SCL, INPUT); //PIR2
    //pinMode(LED, OUTPUT); //LED
    //digitalWrite(LED, HIGH);
    pinMode(LED2, OUTPUT); //LED
    digitalWrite(LED2, HIGH);
    


#if Debug == 1
    Serial.println(F("PIR1 and PIR3 enabled"));
#endif

}

void enablePIR_Interrupts()
{
  #if PIR_Polling_Or_Interrupt == 1

  #if NO_OF_PIR_SENSOR == 1
      attachInterrupt(digitalPinToInterrupt(PIR1), pir1_handleInterrupt, RISING);
      timerPIR.attach(2, clearCounter);
  #endif
  #if NO_OF_PIR_SENSOR == 2
      attachInterrupt(digitalPinToInterrupt(PIR1), pir1_handleInterrupt, RISING);
      //attachInterrupt(digitalPinToInterrupt(PIR2), pir2_handleInterrupt, RISING);
      timerPIR.attach(2, clearCounter);
  #endif
  #if NO_OF_PIR_SENSOR == 3
      attachInterrupt(digitalPinToInterrupt(PIR1), pir1_handleInterrupt, RISING);
      //attachInterrupt(digitalPinToInterrupt(PIR2), pir1_handleInterrupt, RISING);
      attachInterrupt(digitalPinToInterrupt(PIR3), pir3_handleInterrupt, RISING);
      timerPIR.attach(2, clearCounter);
  #endif
  #if NO_OF_PIR_SENSOR == 4
      attachInterrupt(digitalPinToInterrupt(PIR1), pir1_handleInterrupt, RISING);
      //attachInterrupt(digitalPinToInterrupt(PIR2), pir2_handleInterrupt, RISING);
      attachInterrupt(digitalPinToInterrupt(PIR3), pir3_handleInterrupt, RISING);
//      attachInterrupt(digitalPinToInterrupt(PIR4), pir4_handleInterrupt, RISING);
      timerPIR.attach(2, clearCounter);
  #endif
  
#endif
}


void disablePIR_Interrupts()
{
  #if PIR_Polling_Or_Interrupt == 1

  #if NO_OF_PIR_SENSOR == 1
      detachInterrupt(digitalPinToInterrupt(PIR1));
      timerPIR.detach();
  #endif
  #if NO_OF_PIR_SENSOR == 2
      detachInterrupt(digitalPinToInterrupt(PIR1));
//      detachInterrupt(digitalPinToInterrupt(PIR2));
      timerPIR.detach();
  #endif
  #if NO_OF_PIR_SENSOR == 3
      detachInterrupt(digitalPinToInterrupt(PIR1));
//      detachInterrupt(digitalPinToInterrupt(PIR2));
      detachInterrupt(digitalPinToInterrupt(PIR3));
      timerPIR.detach();
  #endif
  #if NO_OF_PIR_SENSOR == 4
      detachInterrupt(digitalPinToInterrupt(PIR1));
//      detachInterrupt(digitalPinToInterrupt(PIR2));
      detachInterrupt(digitalPinToInterrupt(PIR3));
//      detachInterrupt(digitalPinToInterrupt(PIR4));
      timerPIR.detach();
  #endif
  
#endif
}

bool connect_wifi()
{
    for (uint8_t retry_counter = 0; retry_counter < WIFI_CONNECT_RETRIES; retry_counter++)
    {   
      disablePIR_Interrupts();
        if (WiFiMulti.run() == WL_CONNECTED)
        {
#if Debug == 1
            Serial.print("\r\n Wifi Connected \r\n");
#endif
            WiFi.macAddress(WiFiMac);
            enablePIR_Interrupts();
            return true;
        }
        delay(WIFI_RETRY_DELAY);
    }
#if Debug == 1
    Serial.print("\r\n WIFI returned false \r\n");
#endif
    return false;
}

bool check_wifi(void)
{
    // read_ssid_password();
    
     bool result = WiFiMulti.run() == WL_CONNECTED || connect_wifi();
    enablePIR_Interrupts();
    return result;
}

bool connectToServer()
{
    uint8_t maxRetries = MAXIMUM_VALUE;
    uint8_t retry_counter = 0;
    bool canContinue = false;

Retry:
    if (!check_wifi())
    {
        retry_counter++;
        if (retry_counter > WIFI_RETRIES)
        {
#if Debug == 1
            Serial.print("\r\n Wifi Error can't connect to server \r\n");
#endif

            retry_counter = 0;
            return false;
        }
        goto Retry;
    }

    timeClient.begin();
    for (uint8_t attempt = 0; attempt < (TIME_CLIENT_RETRY || !timeClient.update()); attempt++)
    {
        timeClient.forceUpdate();
    }
#if Debug == 1
    Serial.println("Not stuck in timeClient");
#endif
    espClient.setX509Time(timeClient.getEpochTime());

    if (client.connected())
    {
        return true;
    }

    for (uint8_t attempt = 0; attempt < MAXIMUM_VALUE; attempt++)
    {
#if Debug == 1
        Serial.print("Attempting MQTT connection...\r\n");
#endif
        if (client.connect((char *)WiFiMac))
        {

#if Debug == 1
            Serial.println("connected");
#endif

            uint8_t BleMacServer[MAC_SIZE];
            for (int i = 0, j = 0; i < sizeof(WiFiMac); i++, j += 2) // logic for converting byte to bytes
            {
                BleMacServer[j] = (byte)hexToChar(WiFiMac[i] >> 4);
                BleMacServer[j + 1] = (byte)hexToChar(((byte)(WiFiMac[i] << 4)) >> 4);
            }

            uint8_t downStreamTopic[40] = "zerovir/outward/control/";
            memcpy(downStreamTopic + 24, BleMacServer, MAC_SIZE);
            Serial.printf("%s\r\n", downStreamTopic);
            // ... and resubscribe
            client.subscribe((char *)downStreamTopic);

            Serial.println("subscribe done");
            canContinue = true;
            break;
        }
        delay(SERVER_RETRY_DELAY);
    }
    if (!canContinue)
    {
#if Debug == 1
        Serial.println("Server connection error");
#endif
        return false;
    }
    return true;
}

void callback(char *topic, byte *payload, unsigned int len)
{
    if (len > 200)
    {
        return;
    }
#ifdef Debug == 1
    Serial.printf("length Received = %d\r\n", len);
#endif
    bool save_settings = false;
    uint8_t data_received[200] = {0};
    data_decode(payload, len, data_received);
    switch (data_received[0])
    {
    case CONTROL:
#if Debug == 1
        Serial.printf("Relay turned %02x\r\n", data_received[2]);
#endif
        
        if (data_received[2] == 0x00)
        {
            //        gDeviceState = ON_DEMAND_STOP;
            gDeviceMode = MANUAL_MODE;
            gOnDemandDeepCleanFlag = 4;

#ifdef Debug == 1
    Serial.printf("data_received[2] == 0x00\r\n");
#endif
        }
        if (data_received[2] == 0x01)
        {
            if (gCurrentlyObservingFlag == 0 && self_diagnose_inprogress == 0)
            {
                gDeviceState = STATE_ONDEMAND;
                gDeviceMode = MANUAL_MODE;
                gOnDemandDeepCleanFlag = 0;
                gCurrentlyObservingFlag = 1;
                pirCheckRetry = 0;
                ack_pir_status = PIR_ERROR_ACK;
                
                dia_pir_int_count_1 = 0;
                dia_pir_int_count_2 = 0;
                dia_pir_int_count_3 = 0;
                dia_pir_int_count_4 = 0;
                self_diagnose_time_count = 0;
                //nOnCommandStartCounter = 0;  
                light_toggle(0);
                buzzer_toggle(0);
#ifdef Debug == 1
    Serial.printf("data_received[2] == 0x01\r\n");
#endif
            }

                        
        }
        else if (data_received[2] == 0x02 && self_diagnose_inprogress == 0)
        {
            if (gCurrentlyObservingFlag == 0)
            {

                gDeviceState = STATE_ONDEMAND;
                gDeviceMode = MANUAL_MODE;
                gOnDemandDeepCleanFlag = 1;
                gCurrentlyObservingFlag = 1;
                pirCheckRetry = 0;
                ack_pir_status = PIR_ERROR_ACK;
                dia_pir_int_count_1 = 0;
                dia_pir_int_count_2 = 0;
                dia_pir_int_count_3 = 0;
                dia_pir_int_count_4 = 0;
                self_diagnose_time_count = 0;
               
            }
        }
        break;

    case STATUS_DEVICE:
        send_status = true;
        break;

    case PARAMETERS:
        switch (data_received[2])
        {
        case BDUR:
            globalData.BULB_ON_DURATION = data_received[3];
            save_settings = true;
            break;

        case DEEP_CLN_BDUR:
            globalData.DEEP_CLEAN_BULB_ON_DURATION = data_received[3];
            save_settings = true;
            break;

        case AUTO_MD_BDUR:
            globalData.AUTO_MODE_BULB_ON_DURATION = data_received[3];
            save_settings = true;
            break;

        case OB_TIME:
            globalData.OBSERVATION_TIME = data_received[3];
            save_settings = true;
            break;

        case PRE_BZZ_OTIME:
            globalData.PRE_BUZZER_ON_TIMER = data_received[3];
            save_settings = true;
            break;

        case SLEEP_TMR:
            globalData.SLEEP_TIMER = data_received[3];
            save_settings = true;
            break;

        case DAILY_SN_LMT:
            globalData.DAILY_SANITIZATION_LIMIT = data_received[3];
            save_settings = true;
            break;

        case AUTO_OP_RST:
            globalData.AUTO_OP_RESET = ((data_received[3] << 8) | data_received[4]);
            save_settings = true;
            break;

        case AUTO_MD_NMTN:
            globalData.AUTO_MODE_NO_MOTION = ((data_received[3] << 8) | data_received[4]);
            save_settings = true;
            break;

        case SEND_PMTRS:
            send_parameters = true;
            break;
        }
        break;

    case OTA:
#if Debug == 1
        Serial.printf("Firmware version %02x %02x\r\n", data_received[2], data_received[3]);
#endif
        //enable_OTA(data_received[2], data_received[3]);
        uint8_t OTA_Data[3];
        OTA_Data[0] = 0x01;
        OTA_Data[1] = data_received[2];
        OTA_Data[2] = data_received[3];
        OTA_save(OTA_Data, 1);
        ESP.restart();
        break;

    case SSID_USER:
        set_custom_ssid(data_received);
        break;

    case EVENTS:
        send_events_data();
        break;
    }
    if (save_settings == true)
    {
        save_settings = false;
        saveCurrentConfig();
        StoreCRCInMemory();
    }
}

void data_decode(uint8_t *data, uint8_t len, uint8_t *decode_data)
{
    int i = 0, j = 0, len_integer;
    len_integer = len;
    while (len_integer > 0)
    {
        decode_data[i] = ((charTohex(data[j++])) << 4 | (charTohex(data[j++])));
#if Debug == 1
        Serial.printf("decode data = %02x i = %d, j = %d len = %d\r\n", decode_data[i], i, j, len_integer);
#endif

        i++;
        len_integer -= 2;
    }
}

void set_custom_ssid(uint8_t *data)
{
    uint8_t ssid_length = 0;
    uint8_t password_length = 0;
    ssid_length = data[2];
    password_length = data[ssid_length + 3];
    char ssid_1[ssid_length];
    char pass_1[password_length];
    for (uint8_t i = 0; i < ssid_length; i++)
    {
        ssid_1[i] = (char)data[i + 3];
    }
    ssid_1[ssid_length] = '\0';
    for (uint8_t i = 0; i < password_length; i++)
    {
        pass_1[i] = (char)data[i + ssid_length + 4];
    }
    pass_1[password_length] = '\0';
#if Debug == 1
    Serial.printf("%s %s\r\n", ssid_1, pass_1);
#endif
    store_ssid_password(ssid_1, pass_1, strlen(ssid_1), strlen(pass_1));
}

uint8_t charTohex(char c)
{
    if (c >= '0' && c <= '9')
    {
        return (c - 0x30);
    }
    switch (c)
    {
    case 'A':
    case 'a':
        return 10;
    case 'B':
    case 'b':
        return 11;
    case 'C':
    case 'c':
        return 12;
    case 'D':
    case 'd':
        return 13;
    case 'E':
    case 'e':
        return 14;
    case 'F':
    case 'f':
        return 15;
    }
}

uint8_t hexToChar(uint8_t c)
{
    if (c >= 0 && c <= 9)
    {
        return (c + 0x30);
    }
    switch (c)
    {
    case 0x0A:
        return 'A';
    case 0x0B:
        return 'B';
    case 0x0C:
        return 'C';
    case 0x0D:
        return 'D';
    case 0x0E:
        return 'E';
    case 0x0F:
        return 'F';
    }
}

uint32_t send_data_to_server(const char *topic, uint8_t *data, uint32_t len)
{
    uint8_t len_received = len;
    uint8_t *data_send = (uint8_t *)malloc(300 * sizeof(uint8_t));
    uint8_t BleMacServer[MAC_SIZE];
    for (int i = 0, j = 0; i < sizeof(WiFiMac); i++, j += 2) // logic for converting byte to bytes
    {
        BleMacServer[j] = (byte)hexToChar(WiFiMac[i] >> 4);
        BleMacServer[j + 1] = (byte)hexToChar(((byte)(WiFiMac[i] << 4)) >> 4);
    }
    memcpy(&data_send[0], &BleMacServer[0], MAC_SIZE); // copying device mac array to final array

    int i = MAC_SIZE, j = 0;
    while (len > 0)
    {
        data_send[i++] = hexToChar((data[j] >> 4 & 0x0F));
        data_send[i++] = hexToChar((data[j] & 0x0F));
        j++;
        len -= 1;
    }
    uint16_t data_length = 0;
    data_length = ((len_received * 2) + MAC_SIZE);
    for (uint8_t i = 0; i < data_length; i++)
    {
        Serial.printf("%c", data_send[i]);
    }
    data_send[data_length] = '\0';

    char *data_to_send = (char *)malloc(350 * sizeof(char));
    sprintf(data_to_send, "{\n  \"hexData\":\"%s\"\n}", (char *)data_send);
    Serial.printf("\r\n%s\r\n", data_to_send);
    free(data_send);

    if (!connectToServer())
    {
       wifi_connected_or_not = 0;
#ifdef Debug == 1
        Serial.println(".........NOT connecting to server");
#endif
        free(data_to_send);
        return 0;
    }
    else
    {
      wifi_connected_or_not = 1;
    }
    Serial.println("............After trying connections to server OR client");
    client.publish(topic, data_to_send);
    free(data_to_send);
    Serial.println("...........After Publish");
    return 1;
}

void send_ping()
{
    uint8_t ping_data[3] = {Ping, fm_version[0], fm_version[1]};
    send_data_to_server(topic_ping, ping_data, sizeof(ping_data));
}

/* Do ESP OTA */
bool enable_OTA(uint8_t ver1, uint8_t ver2)
{
    if (!check_wifi())
    {
        return false;
    }
    char *fingerprint = "92 8a 0b f0 b4 d6 ff d8 54 38 13 7a 65 b5 f9 c6 c2 91 00 50";
    Serial.println("OTA started");
    BearSSL::WiFiClientSecure UpdateClient;
    UpdateClient.setInsecure();
    char Firmware_Path[] = "";
    sprintf(Firmware_Path, "/ZeroVir_%d.%d.bin", ver1, ver2);
    t_httpUpdate_return res = ESPhttpUpdate.update(UpdateClient, "lithionhex.s3.ap-south-1.amazonaws.com", 443, Firmware_Path, fingerprint);
    switch (res)
    {
    case HTTP_UPDATE_FAILED:
        delay(2000);
        Serial.println("OTA failed");
        WiFi.forceSleepBegin();
        wdt_reset();
        ESP.restart();
        while (1)
        {
            wdt_reset();
        }
        break;
    case HTTP_UPDATE_NO_UPDATES:
        break;
    case HTTP_UPDATE_OK:
        Serial.println("OTA success");
        WiFi.forceSleepBegin();
        wdt_reset();
        ESP.restart();
        while (1)
        {
            wdt_reset();
        }
        break;
    }
}

void store_ssid_password(char *data, char *data1, uint8_t len, uint8_t len1)
{
    int addr = 0;
    addr = SSID1_ADDR;
    for (uint8_t i = 0; i < len; i++)
    {
        EEPROM.write(addr + i, data[i]); //Write one by one with starting address of 0x0F
    }
    addr = PASS1_ADDR;
    for (uint8_t i = 0; i < len1; i++)
    {
        EEPROM.write(addr + i, data1[i]); //Write one by one with starting address of 0x0F
    }
    EEPROM.commit(); //Store data to EEPROM
}

void read_ssid_password()
{
    char ssid1[30], pass1[30];
    int i = 0;
    bool exit1 = false;
    while (exit1 == false)
    {
        ssid1[i] = char(EEPROM.read(SSID1_ADDR + i)); //Read one by one with starting address of 0x0F
        if (ssid1[i] == 0XFF)
        {
            ssid1[i] = '\0';
            exit1 = true;
        }
        i++;
    }

    exit1 = false;
    i = 0;
    while (exit1 == false)
    {
        pass1[i] = char(EEPROM.read(PASS1_ADDR + i)); //Read one by one with starting address of 0x0F
        if (pass1[i] == 0XFF)
        {
            pass1[i] = '\0';
            exit1 = true;
        }
        i++;
    }
    WiFiMulti.addAP(ssid1, pass1);
}

void UNIT_PROGRAM()
{

    //if (gPIRSensingFlag == 1 && (gDeviceState == OBSERVING_ENV || gDeviceState == SANITIZATION_ON))
    //{   //if pir_sense_flag is 1 start reading inputs from PIR sensors
    //uint8_t lPIRStatus = pir_interrupt_received;//pirSensor moved to function
    //Serial.printf("PIR STATUS %d\r\n",lPIRStatus);
    //pir_interrupt_received = false;
    //
    //#if PIR_Polling_Or_Interrupt == 0
    //        uint8_t lPIRStatus = pir_sensor_output();
    //        //Serial.printf("PIR STATUS %d\r\n",lPIRStatus);
    //#endif

#if PIR_Polling_Or_Interrupt == 1
    uint8_t lPIRStatus = pir_interrupt_received;
    //Serial.printf("PIR STATUS %d\r\n",lPIRStatus);
    pir_interrupt_received = false;

#endif
    if (ack_stop_interrupt == 1)
    {
        //ack_stop_interrupt_data
        Serial.println("...............Sending acknowledgement");
        send_acknowledgement((uint8_t *)ack_stop_interrupt_data, sizeof(ack_stop_interrupt_data));
        ack_stop_interrupt = 0;
    }
    // }

    if (gDeviceMode == AUTO_MODE)
    {
        if (nSleepTimeCounter == 0)
        {
            if (gDeviceState != OBSERVING_ENV && gDeviceState != SANITIZATION_ON)
            {
                gDeviceState = SLEEPING;
                nSleepTimeCounter = millis();
#if Debug == 1
                Serial.printf("auto loop device state is : %d\r\n", gDeviceState);
                //Serial.printf("gObservation time : %d\r\n", gOBSERVATION_TIME);
#endif        
                //wifi_connected_or_not = 0;
            }
        }

        if ((millis() - nSleepTimeCounter > (globalData.SLEEP_TIMER * MIN_MULTI)) && gDeviceState != OBSERVING_ENV && gDeviceState != SANITIZATION_ON)
        {
            gDeviceState = STATE_AUTOMODE;
            nSleepTimeCounter = 0;
            gCurrentlyObservingFlag = 1;
            gOnDemandDeepCleanFlag = 9;  // this will put the correct ack for device
            gDeviceMode == AUTO_MODE;    // safety purpose not required
            nOnCommandStartCounter == 0; // so it can change gDeviceState
        }
    }

//    if(gDeviceMode == FAULT_MODE)                                                ///################################################################################## SELF DIAGNOSE MODE CHECK
//    {
//      gOnDemandDeepCleanFlag = 9; // setting flag for safety
//      gStartSanitizationFlag = 0; // reseting the start sanitization flag
//      gCurrentlyObservingFlag = 0;
//      gDeviceState = FAULT_STATE;
//      //self_diagnose_time_count = 0;
//
//      
////      Serial.printf("Device state when in self mode  %d\r\n",gDeviceState);
//    }

    if (gDeviceState == STATE_AUTOMODE || gDeviceState == STATE_ONDEMAND)
    {
        //gPIRSensingFlag = 1;
        uint8_t ack_device = 0;
        if (nOnCommandStartCounter == 0)
        {
            nOnCommandStartCounter = millis();
            gDeviceState = OBSERVING_ENV;
            // adding aditional mode check
            if (gOnDemandDeepCleanFlag == 0 && gDeviceMode == MANUAL_MODE)
            {
                ack_device = ON_DEMAND_START;
                pirCheckRetry = 0;
#ifdef Debug == 1
    Serial.printf("gOnDemandDeepCleanFlag == 0 && gDeviceMode == MANUAL_MODE\r\n");
#endif
            }
            else if (gOnDemandDeepCleanFlag == 1 && gDeviceMode == MANUAL_MODE)
            {
                ack_device = ACK_ON_DEMAND_DEEP_CLEAN;
                pirCheckRetry = 0;
#ifdef Debug == 1
    Serial.printf("gOnDemandDeepCleanFlag == 1 && gDeviceMode == MANUAL_MODE\r\n");
#endif
            }
            else
            {
                ack_device = ACK_SANITIZATION_ATTEMPT;
                pirCheckRetry = 0;
                
#ifdef Debug == 1
    Serial.printf("ack_device = ACK_SANITIZATION_ATTEMPT;\r\n");
#endif
            }
            if (gDeviceMode == MANUAL_MODE)
            {
                gOBSERVATION_TIME = globalData.OBSERVATION_TIME * SEC_MULTI;
                pirCheckRetry = 0;
#ifdef Debug == 1
    Serial.printf("gDeviceMode == MANUAL_MODE     >>> %ul \r\n", gOBSERVATION_TIME);
#endif
            }
            else
            {
                gOBSERVATION_TIME = globalData.AUTO_MODE_NO_MOTION * MIN_MULTI;
                pirCheckRetry = 0;
#ifdef Debug == 1
    Serial.printf("gOBSERVATION_TIME = globalData.AUTO_MODE_NO_MOTION * MIN_MULTI     >>> %ul \r\n", gOBSERVATION_TIME);
#endif
            }
#if Debug == 1
            Serial.printf("device state is : %d\r\n", gDeviceState);
            Serial.printf("gObservation time : %d\r\n", gOBSERVATION_TIME);
#endif
            gPreBuzzerFlag = 1;
            send_acknowledgement(&ack_device, 1);
        }
        
          
          
        
    }


    if(pirCheckRetry == 0 && gDeviceState == OBSERVING_ENV)                                         ///##################################################################################PIR CHECK
    {
      self_diagnose_inprogress = 1;
      dont_check_internet_if_no_internet = 1;
      
      dia_pir_int_count_1 = 0;
      dia_pir_int_count_2 = 0;
      dia_pir_int_count_3 = 0;
      dia_pir_int_count_4 = 0;
      pir_check = 1;              // disable main pir interrupt
      toggle_pir_power();
      pirCheckRetry = 1;         
      timerBuzzer.attach(1, buzzerTonediagnoseTime);
      timerDiagnose.attach(1, start_diagnose_period);
      
#if Debug == 1
            Serial.printf("\r\n Diagnose in progress.......\r\n");
#endif
    }

    if(pirCheckRetry == 1 && gDeviceState == OBSERVING_ENV)
    {
 
      if(self_diagnose_time_count >= 29)
      {
         /* #if NO_OF_PIR_SENSOR == 1
          if(dia_pir_int_count_1 > 1)
          #elif NO_OF_PIR_SENSOR == 2
          if(dia_pir_int_count_1 > 1 && dia_pir_int_count_2 > 1)
          #elif NO_OF_PIR_SENSOR == 3
          if(dia_pir_int_count_1 > 1 && dia_pir_int_count_3 > 1)  //ADD PIR 2 also...removed for testing
          #elif NO_OF_PIR_SENSOR == 4
          if(dia_pir_int_count_1 > 1 && dia_pir_int_count_2 > 1 && dia_pir_int_count_3 > 1 && dia_pir_int_count_4 > 1)
          #endif*/
          if((dia_pir_int_count_1 > 1) && (dia_pir_int_count_3 > 1))
          {
            ack_pir_status = PIR_OK_ACK;
            //send_acknowledgement(&ack_pir_status, 1);
            self_diagnose_completed = 1;
            pirCheckRetry = 2;
            
            #if Debug == 1
              Serial.printf("\r\n PIR OK! \r\n");
            #endif
            nOnCommandStartCounter = millis();
            
          }
          else
          {
            gDeviceMode = FAULT_MODE;    // set device state to self diagnose mode;
            gDeviceState = FAULT_STATE;
            self_diagnose_completed = 1;  
            gStartSanitizationFlag = 0;
            
            timerled2.detach();
            digitalWrite(LED2, HIGH);
            
            ack_pir_status = PIR_ERROR_ACK;
            pirCheckRetry = 2;
            dia_pir_int_count_1 = 0;
            dia_pir_int_count_2 = 0;
            dia_pir_int_count_3 = 0;
            dia_pir_int_count_4 = 0;

            gOnDemandDeepCleanFlag = 9; // setting flag for safety
            gStartSanitizationFlag = 0; // reseting the start sanitization flag
            gCurrentlyObservingFlag = 0;
            
            //send_acknowledgement(&ack_pir_status, 1);
            
            #if Debug == 1
              Serial.printf("\r\n PIR Fault detected!! \r\n");
            #endif
          }

          timerBuzzer.detach();
          timerDiagnose.detach();
          buzzer_toggle(0);               //off buzzer
          light_toggle(0);                //off UV Lamp
//          buzzer_toggle(1);
          pir_check = 0; // enable main interrupt
          send_acknowledgement(&ack_pir_status, 1);
          self_diagnose_inprogress = 0;
          dont_check_internet_if_no_internet = 0;
          self_diagnose_time_count = 0;

          
//        #if Debug == 1
//          Serial.printf("\r\n Self diagnostic time completed!\r\n");
//          Serial.printf("\r\n Pre buzzer time >>>>    nOnCommandStartCounter =  %ul  gOBSERVATION_TIME =  %ul  ,  (millis() - nOnCommandStartCounter) = %ul  ,   ((gOBSERVATION_TIME - (globalData.PRE_BUZZER_ON_TIMER * SEC_MULTI))) = %ul\r\n    %ul", 
//              nOnCommandStartCounter, gOBSERVATION_TIME,((millis() - nOnCommandStartCounter)),((gOBSERVATION_TIME - (globalData.PRE_BUZZER_ON_TIMER * SEC_MULTI))), globalData.PRE_BUZZER_ON_TIMER * SEC_MULTI);
//        #endif
      }      
      
   }


    
    // Starting pre-buzzer can be move into function
    if (gDeviceState == OBSERVING_ENV && self_diagnose_completed == 1 && ack_pir_status == PIR_OK_ACK)
    {
        
        if (gPreBuzzerFlag == 1 && ((millis() - nOnCommandStartCounter) > (gOBSERVATION_TIME - (globalData.PRE_BUZZER_ON_TIMER * SEC_MULTI))) )
        {
//            timerBuzzer.detach();
            //timerDiagnose.detach();
            if(pir_interrupt_received == false) 
            {
                buzzer_toggle(1);
                #if Debug == 1
                  Serial.printf("\r\n Pre buzzer on\r\n");
                #endif
            }
            gPreBuzzerFlag = 0;
            pirCheckRetry = 2;
            
#if Debug == 1
            Serial.printf("\r\n Pre buzzer on\r\n");
#endif
        }
    }

    
    
    if(ack_pir_status == PIR_OK_ACK && self_diagnose_completed == 1)
    {
      if (millis() - nOnCommandStartCounter > gOBSERVATION_TIME )     
      {

          if (gDeviceState == OBSERVING_ENV)
          {
              gStartSanitizationFlag = 1; // main flag for starting sanitization
              nBulbOnTimeCounter = 0;
              
          }
          nOnCommandStartCounter = 0; // timeCounter reset
          
       }
  
    }
    //TODO change gDeviceState commad
    // got off command
    if(gOnDemandDeepCleanFlag == 4 && gDeviceState != SLEEPING)
    {

        light_toggle(0);
        buzzer_toggle(0);
        
        uint8_t lAckForMobile;
        uint32_t localBulbOnDuration = 0;
        // if off command recv during observation period ack returned is OBSERVATION_FORCED_STOP
        if (gDeviceState == OBSERVING_ENV)
        {
            lAckForMobile = OBSERVATION_FORCED_STOP;
        }
        // if off command recv detected while sanitizing return SANITIZATION_FORCED_STOP_ACK
        else if (gDeviceState == SANITIZATION_ON)
        {

            localBulbOnDuration = round((millis() - nBulbOnTimeCounter) / SEC_MULTI);
            lAckForMobile = SANITIZATION_FORCED_STOP_ACK;
#if Debug == 1
            Serial.printf("ACK ON %d %ul\r\n", lAckForMobile, localBulbOnDuration);
#endif
        }
        uint8_t data[5] = {0};
        data[0] = lAckForMobile;
        data[1] = localBulbOnDuration >> 24;
        data[2] = localBulbOnDuration >> 16;
        data[3] = localBulbOnDuration >> 8;
        data[4] = localBulbOnDuration;
        
        send_acknowledgement(data, sizeof(data));
        
        gDeviceState = SLEEPING;
#if Debug == 1
        Serial.printf("UV LAMP OFF BEC RECIEVD OFF CMD\r\n");
#endif
        timerled2.detach();
        digitalWrite(LED2, HIGH);
        
#ifdef AUTO_MODE_ACTIVE
        nSleepTimeCounter = 0;
        gDeviceMode = AUTO_MODE;
#else
        gDeviceMode = SAFE_MODE;
#endif
        gStartSanitizationFlag = 0;
        nOnCommandStartCounter = 0;
        nBulbOnTimeCounter = 0;
        gPIRSensingFlag = 0;
        gOnDemandDeepCleanFlag = 0; // changing from zero to 9;
        gCurrentlyObservingFlag = 0;

        ack_pir_status = PIR_ERROR_ACK;
        pirCheckRetry = 2;
        
        if(self_diagnose_inprogress == 1)
       {
        timerBuzzer.detach();
        timerDiagnose.detach();
       }
        digitalWrite(LED2, HIGH);
        //dont_check_internet_if_no_internet = 0;
        
    }

    if (gStartSanitizationFlag == 1 && self_diagnose_completed == 1 && ack_pir_status == PIR_OK_ACK)
    {   
         //timerled2.attach(2, blinkLed);

        //timerDiagnose.detach();
        if (nBulbOnTimeCounter == 0)
        {
            gDeviceState = SANITIZATION_ON;
            nBulbOnTimeCounter = millis();
            uint8_t start_sani = STARTING_SANITIZATION;
#if Debug == 1
            Serial.printf("Sending Acks before dealy\r\n");
//            Serial.printf("%l", gBULB_ON_DURATION);
#endif
            send_acknowledgement(&start_sani, 1);
            delay(20);

#if Debug == 1
            Serial.printf("Sending Acks after dealy\r\n");
//            Serial.printf("%l", gBULB_ON_DURATION);
#endif

            // bulb on duration can be setup here
            if (gOnDemandDeepCleanFlag == 1)
            {
                gBULB_ON_DURATION = globalData.DEEP_CLEAN_BULB_ON_DURATION * MIN_MULTI;
            }
            else if (gDeviceMode == AUTO_MODE)
            {
                gBULB_ON_DURATION = globalData.AUTO_MODE_BULB_ON_DURATION * MIN_MULTI;
            }
            else
            {
                gBULB_ON_DURATION = globalData.BULB_ON_DURATION * MIN_MULTI;
            }

#if Debug == 1
            Serial.printf("UV LAMP ON\r\n");
//            Serial.printf("%l", gBULB_ON_DURATION);
#endif
            //      actual bulb on and buzzer on command
            light_toggle(1);
            buzzer_toggle(1);
#if Debug == 1
            Serial.printf("Tripy relay\r\n");
#endif      
            timerled2.attach(1, blinkLed);

        }
        if (millis() - nBulbOnTimeCounter > gBULB_ON_DURATION)
        {
            
            
            uint8_t lamp_check_data = 0;                                  ///################################################################################## LAMP CHECK DATA      
            lamp_check_data = Is_uv_lamps_ok();
            delay(1000);
            timerled2.detach();
            digitalWrite(LED2, HIGH);
            light_toggle(0);
            buzzer_toggle(0);
            
            
            uint32_t localBulbOnDuration = round((millis() - nBulbOnTimeCounter) / SEC_MULTI);

            uint8_t data[5];
            data[0] = SUCCESS_SANITIZATION;
            data[1] = localBulbOnDuration >> 24;
            data[2] = localBulbOnDuration >> 16;
            data[3] = localBulbOnDuration >> 8;
            data[4] = localBulbOnDuration;
#if Debug == 1
            Serial.printf("Sending Ack for Success sanitization\r\n");
#endif

            send_acknowledgement(data, sizeof(data));
            send_acknowledgement(&lamp_check_data, 1);
            gDeviceState = SLEEPING;
#if Debug == 1
            Serial.printf("Sanitization Complete\r\n");
#endif
            
            nBulbOnTimeCounter = 0;
            gOnDemandDeepCleanFlag = 9; // setting flag for safety
            gStartSanitizationFlag = 0; // reseting the start sanitization flag
            gPIRSensingFlag = 0;
            gCurrentlyObservingFlag = 0;
#ifdef AUTO_MODE_ACTIVE
            nSleepTimeCounter = 0;
            gDeviceMode = AUTO_MODE;
#else
            gDeviceMode = SAFE_MODE;
#endif
            ack_pir_status = PIR_ERROR_ACK;
            pirCheckRetry = 2;
//            timerBuzzer.detach();
//            timerled2.detach();

            //timerDiagnose.detach();
//            digitalWrite(LED2, HIGH);
            //dont_check_internet_if_no_internet = 0;
        }
    }
}

void save_data_in_memory(uint8_t *data, uint32_t len)
{
    int addr = 0;
    addr = DATA_ADDR;
    for (uint8_t i = 0; i < len; i++)
    {
        EEPROM.write(addr + i, data[i]); //Write one by one with starting address of 0x0F
    }
    EEPROM.commit();
}

void read_data_from_memory(uint8_t *data, uint32_t len)
{
    int addr = 0;
    addr = DATA_ADDR;
    for (uint8_t i = 0; i < len; i++)
    {
        data[i] = EEPROM.read(addr + i);
    }
}

void send_device_status()
{
    uint8_t data[4]; //sizeof(gDeviceState)+ sizeof(gDeviceMode)+ sizeof(gOnDemandDeepCleanFlag)];
    data[0] = Status_upstream;
    data[1] = gDeviceState;
    data[2] = gDeviceMode;
    data[3] = gOnDemandDeepCleanFlag;
    send_data_to_server(topic_status, data, sizeof(data));
}

void send_parameters_server()
{
    uint8_t data[sizeof(globalData) - 3 + 1];
    data[0] = Parameters_upstream;
    memcpy(data + 1, &globalData, sizeof(globalData) - 3);
    send_data_to_server(topic_param, data, sizeof(globalData) - 2);
}

void send_acknowledgement(uint8_t *data, uint8_t len)
{
    uint8_t data_server[50] = {0};
    data_server[0] = Acknowledgement;
    memcpy(data_server + 1, data, len);
    send_data_to_server(topic_ack, data_server, len + 1);
}

void saveCurrentConfig()
{
    save_data_in_memory((uint8_t *)&globalData, sizeof(globalData));
}

void buzzer_toggle(bool state)
{
    digitalWrite(Buzzer, state);
#if Debug == 1
    Serial.printf("\r\n");
    Serial.printf("Buzzer toggled to new state = %d", state);
    Serial.printf("\r\n");
#endif 
}

void light_toggle(bool state)
{
    digitalWrite(Relay, state);
#if Debug == 1
    Serial.printf("\r\n");
    Serial.printf("Relay state toggled new state =  %d", state);
    Serial.printf("\r\n");
#endif 
}

uint8_t gencrc(uint8_t *data, size_t len)
{
    uint8_t crc = 0xff;
    size_t i, j;
    for (i = 0; i < len; i++)
    {
        crc ^= sizeof(data[i]);
        for (j = 0; j < 8; j++)
        {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}

void StoreCRCInMemory()
{
    uint8_t crc = 0;
    Serial.println("Saving CRC in memory");
    crc = gencrc((uint8_t *)&globalData, sizeof(globalData));
    Serial.println(crc);
    EEPROM.write(CRC_GLOBAL, crc);
    EEPROM.commit();
}

uint8_t readCRCFromMemory()
{
    uint8_t crc = 0;
    crc = (uint8_t)EEPROM.read(CRC_GLOBAL);
    return crc;
}

void OTA_save(uint8_t *data, bool save_data)
{
    if (save_data)
    {
        saveCurrentConfig();
        StoreCRCInMemory();
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        EEPROM.write(OTA_ADDR + i, data[i]);
    }

    EEPROM.commit();
}

void OTA_check()
{
    uint8_t ver1, ver2;
    if ((uint8_t)EEPROM.read(OTA_ADDR) == 0x01)
    {
        ver1 = (uint8_t)EEPROM.read(OTA_ADDR + 1);
        ver2 = (uint8_t)EEPROM.read(OTA_ADDR + 2);
        uint8_t data[3] = {0x00, 0x00, 0x00};
        data[0] = 0xFF;
        OTA_save(data, 0);
        enable_OTA(ver1, ver2);
    }
}

uint8_t pir_sensor_output()
{
    uint8_t result_output = 0;
    switch (NO_OF_PIR_SENSOR)
    {
    case 1:
        if (digitalRead(PIR1) == 1)
        {
            result_output = 1;
        }
        break;
    case 2:
        if ((digitalRead(PIR1) == 1) || (digitalRead(PIR3) == 1))
        {
            Serial.printf("Sensor 1 2 output, %d %d\r\n", digitalRead(PIR1), digitalRead(PIR3));
            result_output = 1;
            Serial.printf("THIS CLI \r\n");
        }
        break;
    default:
        //    if invalid no of sensor value entered it will always say high so device cannot operate
        result_output = 1;
        break;
    }

    return result_output;
}

void clearCounter()
{
    pir1_interruptCounter = 0;
    pir2_interruptCounter = 0;
    pir3_interruptCounter = 0;
    pir4_interruptCounter = 0;
    //pir_interrupt_received = false;
}

void create_event(uint8_t event_type)
{
    events_t.event_time = timeClient.getEpochTime();
    events_t.event_type = event_type;
    uint8_t event_buff[EVENT_SIZE];
    memcpy(event_buff, (uint8_t *)&events_t, EVENT_SIZE);
    for (uint8_t i = 0; i < EVENT_SIZE; i++)
    {
        EEPROM.write(globalData.events_current_address + i, event_buff[i]);
    }
    EEPROM.commit();
    globalData.events_current_address += EVENT_SIZE;
    if (globalData.events_current_address == EVENT_END_ADDR)
    {
        globalData.events_current_address = EVENT_START_ADDR;
    }
}

void send_events_data()
{
    uint8_t event_buff[(EVENT_START_ADDR - globalData.events_current_address) + 1];
    event_buff[0] = Events;
    for (uint8_t i = 0; i < EVENT_START_ADDR - globalData.events_current_address; i++)
    {
        event_buff[i + 1] = EEPROM.read(EVENT_START_ADDR + i);
    }
    send_data_to_server(topic_ping, event_buff, EVENT_START_ADDR - globalData.events_current_address);
}

void stopAfterPIR_interrupt()
{
    //if (pir_interrupt_received == true)
    //{ //if motion detected by PIR sensor(s)

        light_toggle(0);
        buzzer_toggle(0);
        uint8_t lAckForTopic;
        uint32_t localBulbOnDuration;
        if (gDeviceState == OBSERVING_ENV)
        {
            lAckForTopic = OBSERVATION_ABORTED;
        }
        else if (gDeviceState == SANITIZATION_ON)
        {
            localBulbOnDuration = round((millis() - nBulbOnTimeCounter) / SEC_MULTI);
            lAckForTopic = SANITIZATION_ABORTED_ACK;
        }

        uint8_t data[5] = {0};
        data[0] = lAckForTopic;
        data[1] = localBulbOnDuration >> 24;
        data[2] = localBulbOnDuration >> 16;
        data[3] = localBulbOnDuration >> 8;
        data[4] = localBulbOnDuration;

        ack_stop_interrupt_data[0] = data[0];
        ack_stop_interrupt_data[1] = data[1];
        ack_stop_interrupt_data[2] = data[2];
        ack_stop_interrupt_data[3] = data[3];
        ack_stop_interrupt_data[4] = data[4];

        //send_acknowledgement(data, sizeof(data));

#ifdef AUTO_MODE_ACTIVE
        nSleepTimeCounter = 0;
        gDeviceMode = AUTO_MODE;
#else
        gDeviceMode = SAFE_MODE;
#endif

        gDeviceState = SLEEPING;

        gStartSanitizationFlag = 0;
        nOnCommandStartCounter = 0;
        nBulbOnTimeCounter = 0;
        gPIRSensingFlag = 0;
        gOnDemandDeepCleanFlag = 9; // changing flag;
        gCurrentlyObservingFlag = 0;
        dont_check_internet_if_no_internet = 0;
        pirCheckRetry = 2;
        ack_pir_status = PIR_ERROR_ACK;
   // }
}


uint8_t recheck_uv_lamps_ok()
{
    if (digitalRead(SCL) == LOW)
     {  
        return LAMP_OK_ACK;
     }
     else
     {        
        return LAMP_ERROR_ACK;   
     }
     
}



uint8_t Is_uv_lamps_ok()
{
    if (digitalRead(SCL) == LOW)
     {  
#if Debug == 1
              Serial.print("/r/n..................LAMP OK.....................! /r/n");
#endif   
        return LAMP_OK_ACK;
     }
  //if (digitalRead(SCL) == LOW)
  else
     {
#if Debug == 1
            Serial.print("/r/n....................LAMP FAILED.................! /r/n");
#endif        
        return LAMP_ERROR_ACK;   
     }
     
}


void toggle_pir_power()
{
    
    digitalWrite(PIR4_AS_PWR_CTR, LOW);
    //digitalWrite(LED, LOW);
    delay(500);
    //digitalWrite(LED, HIGH);
    digitalWrite(PIR4_AS_PWR_CTR, HIGH);
#if Debug == 1
            Serial.println("PIR POWER TOGGLED.....!");
#endif
      
}

void buzzerTonediagnoseTime()
{
    digitalWrite(Buzzer, !(digitalRead(Buzzer)));
}

void blinkLed()
{
    digitalWrite(LED2, !(digitalRead(LED2)));
}

void start_diagnose_period()
{
  //self_diagnose_time_count = !self_diagnose_time_count;
#if Debug == 1
            Serial.printf("self_diagnose_time_count  =   %d!\r\n", self_diagnose_time_count);
#endif
  self_diagnose_time_count++;
  
  if(self_diagnose_time_count >= 30 || pir_interrupt_received == true)
  {
    self_diagnose_time_count = 0;
    timerDiagnose.detach();
  }

}
