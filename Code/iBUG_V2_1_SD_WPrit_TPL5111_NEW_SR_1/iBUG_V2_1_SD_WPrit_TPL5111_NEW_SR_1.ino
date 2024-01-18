#if !defined(ARDUINO_ARCH_RP2040)
  #error For RP2040 only
#endif

#if defined(ARDUINO_ARCH_MBED)
  
  #define PIN_SD_MOSI       PIN_SPI_MOSI
  #define PIN_SD_MISO       PIN_SPI_MISO
  #define PIN_SD_SCK        PIN_SPI_SCK
  #define PIN_SD_SS         PIN_SPI_SS

#else

  #define PIN_SD_MOSI       PIN_SPI0_MOSI
  #define PIN_SD_MISO       PIN_SPI0_MISO
  #define PIN_SD_SCK        PIN_SPI0_SCK
  #define PIN_SD_SS         PIN_SPI0_SS
  
#endif


#include <Arduino.h>
#include "LoRaWan-Arduino.h" //http://librarymanager/All#SX126x
#include <SPI.h>

#include <stdio.h>
#include "Adafruit_BME680.h"
#include <Adafruit_Sensor.h>
#include <RP2040_SD.h>
#include <SparkFun_RV8803.h>

#include "mbed.h"
#include "rtos.h"
#include <avr/dtostrf.h>

using namespace std::chrono_literals;
using namespace std::chrono;

Adafruit_BME680 bme; // I2C
RV8803 rtc; // I2C

#define LEDPIN   24
#define DONEPIN  8

///distance measurement defines
#define TRIGGER_PIN 6
#define ECHO_PIN 9

#define USONIC_DIV 58.0
#define MEASURE_SAMPLE_DELAY 5
#define MEASURE_SAMPLES 20
#define MEASURE_DELAY 25
//end //


File root;
bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_0									  /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5							/*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3										  /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;					/* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_US915;    /* Region:EU868*/
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;				  /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;							        /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);
void lorawan_unconf_finished(void);
void lorawan_conf_finished(bool result);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                          lorawan_rx_handler, lorawan_has_joined_handler,
                                          lorawan_confirm_class_handler, lorawan_join_failed_handler,
                                          lorawan_unconf_finished, lorawan_conf_finished
                                         };
//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x42, 0x96};
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE7};
uint8_t nodeAppKey[16] = {0xAC, 0x60, 0x1E, 0xAB, 0x3B, 0x1B, 0xDB, 0x01, 0xF4, 0xF8, 0xF6, 0xA2, 0x1D, 0xCB, 0x25, 0x54};

// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 500                        /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.
#define SEALEVELPRESSURE_HPA (1013.25)
char dataStr[100] = "";
char buffer[7];
char buff[] = "20180810T143000Z";

mbed::Ticker appTimer;
void tx_lora_periodic_handler(void);

static uint32_t count = 0;
static uint32_t count_fail = 0;
String data = "";
bool send_now = false;
bool send_complete = false;
float pressure;   
float temperature;  
float altimeter; 
float humidity;
float gasResistance;


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(TRIGGER_PIN, OUTPUT); // Initializing Trigger Output and Echo Input
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIGGER_PIN, LOW); // Reset the trigger pin and wait a half a second

  Wire.begin();
  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }
  Serial.print("Initializing SD card with SS = ");  Serial.println(PIN_SD_SS);
  Serial.print("SCK = ");   Serial.println(PIN_SD_SCK);
  Serial.print("MOSI = ");  Serial.println(PIN_SD_MOSI);
  Serial.print("MISO = ");  Serial.println(PIN_SD_MISO);

  if (!SD.begin(PIN_SD_SS)) 
  {
    Serial.println("Initialization failed in SD Card!");
    return;
  }
   
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  if (rtc.begin() == false)
  {
    Serial.println("RTC RB8803 Device not found. Please check wiring. Freezing.");
    while(1);
  }
  Serial.println("RTC online!");



  // Initialize LoRa chip.
  lora_rak11300_init();

  Serial.println("=====================================");
  Serial.println("Welcome to RAK11300 LoRaWan!!!");
  if (doOTAA)
  {
    Serial.println("Type: OTAA");
  }
  else
  {
    Serial.println("Type: ABP");
  }

  switch (g_CurrentRegion)
  {
    case LORAMAC_REGION_AS923:
      Serial.println("Region: AS923");
      break;
    case LORAMAC_REGION_AU915:
      Serial.println("Region: AU915");
      break;
    case LORAMAC_REGION_CN470:
      Serial.println("Region: CN470");
      break;
    case LORAMAC_REGION_CN779:
      Serial.println("Region: CN779");
      break;
    case LORAMAC_REGION_EU433:
      Serial.println("Region: EU433");
      break;
    case LORAMAC_REGION_IN865:
      Serial.println("Region: IN865");
      break;
    case LORAMAC_REGION_EU868:
      Serial.println("Region: EU868");
      break;
    case LORAMAC_REGION_KR920:
      Serial.println("Region: KR920");
      break;
    case LORAMAC_REGION_US915:
      Serial.println("Region: US915");
      break;
    case LORAMAC_REGION_RU864:
      Serial.println("Region: RU864");
      break;
    case LORAMAC_REGION_AS923_2:
      Serial.println("Region: AS923-2");
      break;
    case LORAMAC_REGION_AS923_3:
      Serial.println("Region: AS923-3");
      break;
    case LORAMAC_REGION_AS923_4:
      Serial.println("Region: AS923-4");
      break;
  }
  Serial.println("=====================================");

  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }
  else
  {
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);
  }

  // Initialize LoRaWan
  uint32_t err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  lmh_join();
  Serial.println("Initialization done.");
}

void loop()
{
  // Every LORAWAN_APP_INTERVAL milliseconds send_now will be set
  digitalWrite(LEDPIN, HIGH);
  String dataString = "";
  dataStr[0] = 0;
  // true by the application timer and collects and sends the data
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading of BME68X in Loop :(");
    return;
  }
 long distance = measure();
 pressure = bme.pressure/100.0;
 temperature = bme.temperature;
 humidity = bme.humidity;
 gasResistance= bme.gas_resistance / 1000.0;
 altimeter = bme.readAltitude(SEALEVELPRESSURE_HPA); 
 data = "Temp = " + String(temperature) + " " + " humidity = " + String(humidity) + " " + " Pr = " + String(pressure) + " "+ " gasResistance = " + String(gasResistance) + " " + " Alt = " + String(altimeter) + " " + "Distance = " + String(distance);
//Serial.println(data);

if (rtc.updateTime() == true) //Updates the time variables from RTC
  {
    sprintf(buff, "%02d/%02d/%02d %02d:%02d:%02d", rtc.getYear(),rtc.getMonth(), rtc.getWeekday(), rtc.getHours(), rtc.getMinutes(),rtc.getSeconds());
  }
  else
  {
    Serial.print("RTC read failed");
  }



delay(1000);

#define fileName  "DATALOG.txt"
File dataFile = SD.open("DATALOG.txt", FILE_WRITE);

  // if the file is available, write to it:
if (dataFile) 
  {
    Serial.print("Inside SD ");
    dataFile.print(buff);
    dataFile.print(" : ");
    dataFile.println(data);
    dataFile.close();
    // print to the serial port too:
    Serial.print(buff);
    Serial.print(" : ");
    Serial.println(data);
    //Serial.println(dataStr);

    delay(1000);
   
  }
  // if the file isn't open, pop up an error:
else 
  {
    Serial.print("Error opening "); Serial.println(fileName);
  }
  

delay(1000);

if (send_now)
  {
    Serial.println("Sending frame now...");
    send_now = false;
    send_lora_frame();
       
  }
else {


}
delay(3000);

if(send_complete){
  Serial.println("Send Complete!");
  send_complete =false; 
  digitalWrite(LEDPIN, LOW);
  digitalWrite(DONEPIN, HIGH);
  }

else {
  Serial.println("Error: Unable to send!");
  digitalWrite(LEDPIN, LOW);
  digitalWrite(DONEPIN, HIGH);

}  
  
}

/**@brief LoRa function for handling HasJoined event.
*/
void lorawan_has_joined_handler(void)
{
  if (doOTAA == true)
  {
    Serial.println("OTAA Mode, Network Joined!");
  }
  else
  {
    Serial.println("ABP Mode");
  }

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    // Start the application timer. Time has to be in microseconds
    appTimer.attach(tx_lora_periodic_handler, (std::chrono::microseconds)(LORAWAN_APP_INTERVAL * 1000));
  }
}
/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}
/**@brief Function for handling LoRaWan received data from Gateway

   @param[in] app_data  Pointer to rx data
*/
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
                app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void lorawan_unconf_finished(void)
{
  Serial.println("TX finished");
}

void lorawan_conf_finished(bool result)
{
  Serial.printf("Confirmed TX %s\n", result ? "success" : "failed");
}

void send_lora_frame(void)
{
  uint16_t Tm = 0;
  uint16_t Pr = 0;
  uint16_t Alt = 0;
  uint16_t Hm = 0;
  uint16_t gR = 0;
  long Distance = 0; 
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading of BME 680x in LoRa Send :(");
    return;
  }
 Pr = bme.pressure/100.0;
 Tm = bme.temperature;
 Hm = bme.humidity;
 gR= bme.gas_resistance / 1000.0;
 Alt = bme.readAltitude(SEALEVELPRESSURE_HPA); 
 Distance = measure();

  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }

  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = 0x02;
  m_lora_app_data.buffer[i++] = ',';
  m_lora_app_data.buffer[i++] = (uint8_t)((Tm& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(Tm& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((Hm& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(Hm& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((Pr & 0xFF00) >> 8);   
  m_lora_app_data.buffer[i++] = (uint8_t)(Pr & 0x00FF);  
  m_lora_app_data.buffer[i++] = (uint8_t)((Distance & 0xFF00) >> 8);   
  m_lora_app_data.buffer[i++] = (uint8_t)(Distance & 0x00FF);  
  m_lora_app_data.buffsize = i;
  Serial.printf("Data %d\n", m_lora_app_data);
  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
  send_complete =true;
}

/**@brief Function for handling user timerout event.
*/
void tx_lora_periodic_handler(void)
{
  appTimer.attach(tx_lora_periodic_handler, (std::chrono::microseconds)(LORAWAN_APP_INTERVAL * 1000));
  // This is a timer interrupt, do not do lengthy things here. Signal the loop() instead
  send_now = true;
}




// distnace measure function 

long measure()
{
long measureSum = 0;
  for (int i = 0; i < MEASURE_SAMPLES; i++)
    {
      delay(MEASURE_SAMPLE_DELAY);   //avg measurement
      measureSum += singleMeasurement();
    }
  return measureSum / MEASURE_SAMPLES;
}

long singleMeasurement()
{
  long duration = 0; // Measure: Put up Trigger...
  digitalWrite(TRIGGER_PIN, HIGH); // Wait for 11 µs ...
  delayMicroseconds(11); // Put the trigger down ...
  digitalWrite(TRIGGER_PIN, LOW); // Wait for the echo ...
  duration = pulseIn(ECHO_PIN, HIGH);
  return (long) (((float) duration / USONIC_DIV) * 10.0);
}

