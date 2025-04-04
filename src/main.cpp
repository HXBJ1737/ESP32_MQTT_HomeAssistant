#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

#include "BH1750FVI.h"
#include "DHT.h"
#include <time.h>
#include "main.h"

#include <WiFi.h>
#include <PubSubClient.h>
/*******************************************
 MAX30102(心率_温度) IIC_SDA 40 IIC_SCL 39 led 38 37
 GY30(光照度)        IIC_SDA 36, IIC_SCL 35
 CO2                 ADC  6
 温湿度(DHT11)       Onewire 47
 TFT(ST7789)         SCL 1  SDA 2  DC 42  CS 41
 MQ Sensor ADC 7
*/
/*****************************************/
unsigned long loopCount;
unsigned long startTime;
/***************Timer****************** */
hw_timer_t *tim1 = NULL;
volatile unsigned int tim1_IRQ_count = 1;

hw_timer_t *tim2 = NULL;
volatile unsigned int tim2_IRQ_count = 0;

/**************TFT******************** */
TFT_eSPI tft = TFT_eSPI();
/************MAX30102***************** */

MAX30105 particleSensor;
const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
int spo2_1_time = 0;
float beatsPerMinute;
int beatAvg;
int spo2_1;
#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100];  // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data
int32_t bufferLength;    // data length
int32_t spo2;            // SPO2 value
int8_t validSPO2;        // indicator to show if the SPO2 calculation is valid
int32_t heartRate;       // heart rate value
int8_t validHeartRate;   // indicator to show if the heart rate calculation is valid
byte pulseLED;           // Must be on PWM pin   38
byte readLED;            // Blinks with each data read   37

/**************GY30****************** */
#define GY30_IIC_SDA 36
#define GY30_IIC_SCL 35
BH1750FVI myLux(0x23);

float lux_val;
/**************DHT11***************** */
#define DHTPIN 47
#define DHTTYPE DHT11 // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE);
float h;
float t;
/**************KEY****************** */
#define KEY1 4
#define KEY2 5
#define KEY3 6
#define LED0 38
#define LED1 37

byte mode = 0;
/**************MQ Sensor************ */
#define MQ_ADC 7
uint16_t mq_adc_val;
/***************JW01**************** */
#define JW01_A 17 // TX
#define JW01_B 18 // RX
int co2_val;
/**************MQTT***************** */
#define MQTT_LED 19
const char *ssid = "iQOO_Neo5";
const char *password = "death001";
const char *mqtt_server = "121.37.24.129";
const char *client_user = "admin";
const char *client_password = "death001";
const uint16_t mqtt_client_buff_size = 4096; // 客户端缓存大小
WiFiClient espClient;
PubSubClient client;
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
/*-------------------------------------------------------------------*/
const char *sensor1_config_topic = "homeassistant/sensor/HA/HA-ESP32-02-01/config";
const char *sensor2_config_topic = "homeassistant/sensor/HA/HA-ESP32-02-02/config";
const char *sensor3_config_topic = "homeassistant/sensor/HA/HA-ESP32-02-03/config";
const char *sensor4_config_topic = "homeassistant/sensor/HA/HA-ESP32-02-04/config";
const char *sensor5_config_topic = "homeassistant/sensor/HA/HA-ESP32-02-05/config";
const char *sensor6_config_topic = "homeassistant/sensor/HA/HA-ESP32-02-06/config";
const char *sensor7_config_topic = "homeassistant/sensor/HA/HA-ESP32-02-07/config";
const char *sensor8_config_topic = "homeassistant/sensor/HA/HA-ESP32-02-08/config";
const char *sensor1_state_topic = "HA-ESP32-02/01/state";
const char *sensor2_state_topic = "HA-ESP32-02/02/state";
const char *sensor3_state_topic = "HA-ESP32-02/03/state";
const char *sensor4_state_topic = "HA-ESP32-02/04/state";
const char *sensor5_state_topic = "HA-ESP32-02/05/state";
const char *sensor6_state_topic = "HA-ESP32-02/06/state";
const char *sensor7_state_topic = "HA-ESP32-02/07/state";
const char *sensor8_state_topic = "HA-ESP32-02/08/state";
/*
{
    "unique_id": "HA-ESP32-01-01",
    "name": "key1",
    "icon": "mdi:lightbulb",
    "state_topic": "HA-ESP32-01/01/state",
    "json_attributes_topic": "HA-ESP32-01/01/attributes",
    "unit_of_measurement": " ",
    "command_topic":"HA-ESP32-01/01/set",
    "device": {
        "identifiers": "ESP32-01",
        "manufacturer": "恒星不见",
        "model": "HA",
        "name": "ESP32-01",
        "sw_version": "1.0"
    }
}
*/
/*-------------------------------------------------------------------*/
const char *key1_config_topic = "homeassistant/switch/HA/HA-ESP32-01-01/config";
const char *key2_config_topic = "homeassistant/switch/HA/HA-ESP32-01-02/config";
const char *key3_config_topic = "homeassistant/switch/HA/HA-ESP32-01-03/config";
const char *key4_config_topic = "homeassistant/switch/HA/HA-ESP32-01-04/config";
const char *key5_config_topic = "homeassistant/switch/HA/HA-ESP32-01-05/config";
const char *key6_config_topic = "homeassistant/switch/HA/HA-ESP32-01-06/config";
const char *key7_config_topic = "homeassistant/switch/HA/HA-ESP32-01-07/config";
const char *key8_config_topic = "homeassistant/switch/HA/HA-ESP32-01-08/config";

const char *key1_control_topic = "HA-ESP32-01/01/set";
const char *key2_control_topic = "HA-ESP32-01/02/set";
const char *key3_control_topic = "HA-ESP32-01/03/set";
const char *key4_control_topic = "HA-ESP32-01/04/set";
const char *key5_control_topic = "HA-ESP32-01/05/set";
const char *key6_control_topic = "HA-ESP32-01/06/set";
const char *key7_control_topic = "HA-ESP32-01/07/set";
const char *key8_control_topic = "HA-ESP32-01/08/set";

const char *key1_state_topic = "HA-ESP32-01/01/state";
const char *key2_state_topic = "HA-ESP32-01/02/state";
const char *key3_state_topic = "HA-ESP32-01/03/state";
const char *key4_state_topic = "HA-ESP32-01/04/state";
const char *key5_state_topic = "HA-ESP32-01/05/state";
const char *key6_state_topic = "HA-ESP32-01/06/state";
const char *key7_state_topic = "HA-ESP32-01/07/state";
const char *key8_state_topic = "HA-ESP32-01/08/state";

const char *DEVICE_ON = "ON";
const char *DEVICE_OFF = "OFF";

const PROGMEM uint8_t key1 = 14;
const PROGMEM uint8_t key2 = 13;
const PROGMEM uint8_t key3 = 12;
const PROGMEM uint8_t key4 = 11;
const PROGMEM uint8_t key5 = 10;
const PROGMEM uint8_t key6 = 9;
const PROGMEM uint8_t key7 = 46;
const PROGMEM uint8_t key8 = 3;

bool key1_state = false;
bool key2_state = false;
bool key3_state = false;
bool key4_state = false;
bool key5_state = false;
bool key6_state = false;
bool key7_state = false;
bool key8_state = false;
String strRecv = "";
long now = 0;
long lastRecv = 0;
bool newDataComing = false;
/*********************************** */
#define FAN_PIN 15
#define ENHUM_PIN 16
#define BLED_PIN 8
int en_bled = 30;
int en_enhum = 60;
int en_fan = 2100;
int de_fan = 2000;
/*********************************** */

void setup()
{
    Serial.begin(9600);
    loopCount = 0;
    startTime = millis();
    srand((unsigned)time(NULL));
    pinMode(KEY1, INPUT_PULLUP);
    pinMode(KEY2, INPUT_PULLUP);
    pinMode(KEY3, INPUT_PULLUP);
    pinMode(LED1, OUTPUT);
    pinMode(LED0, OUTPUT);

    pinMode(FAN_PIN, OUTPUT);
    pinMode(ENHUM_PIN, OUTPUT);
    pinMode(BLED_PIN, OUTPUT);

    pinMode(MQ_ADC, INPUT);
    /********************************* */
    tft.init();
    tft.fillScreen(TFT_BLACK);
    tft.setTextWrap(false);
    /********************************* */
    pinMode(MQTT_LED, OUTPUT);
    pinMode(key1, OUTPUT);
    pinMode(key2, OUTPUT);
    pinMode(key3, OUTPUT);
    pinMode(key4, OUTPUT);
    pinMode(key5, OUTPUT);
    pinMode(key6, OUTPUT);
    pinMode(key7, OUTPUT);
    pinMode(key8, OUTPUT);
    setkey1_State();
    setkey2_State();
    setkey3_State();
    setkey4_State();
    setkey5_State();
    setkey6_State();
    setkey7_State();
    setkey8_State();
    // setup_wifi();
    client.setClient(espClient);
    client.setServer(mqtt_server, 1883);
    client.setBufferSize(mqtt_client_buff_size);
    client.setCallback(callback);
    /********************************* */
    Serial1.begin(9600, SERIAL_8N1, JW01_B, JW01_A);
    /********************************* */
    if (particleSensor.begin(Wire, I2C_SPEED_FAST) == false) // Use default I2C port, 400kHz speed
    {
        Serial.println("MAX30105 was not found. Please check wiring/power. ");
        while (1)
            ;
    }

    /********************************* */
    Wire1.begin(GY30_IIC_SDA, GY30_IIC_SCL);
    myLux.powerOn();
    myLux.setContHighRes();
    myLux.setAngle(0);
    /************DHT11**************** */
    dht.begin();
    /*************Timer*************** */
    tim1 = timerBegin(0, 80, true);
    timerAttachInterrupt(tim1, Tim1Interrupt, true);
    timerAlarmWrite(tim1, 100000ul, true); // 100ms进一次中断
    timerAlarmEnable(tim1);

    tim2 = timerBegin(1, 80, true);
    timerAttachInterrupt(tim2, Tim2Interrupt, true);
    timerAlarmWrite(tim2, 100000ul, true); // 100ms进一次中断
    timerAlarmEnable(tim2);
}

void loop()
{
    /******************** */
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();
    /******************* */
    byte key_v = key_get();
    if (key_v == 1)
    {
        mode = 1 - mode;
    }
    if (mode == 0)
    {
        digitalWrite(LED1, LOW);
        show_init0();
        while (1)
        {
            if (!client.connected())
            {
                reconnect();
            }
            client.loop();
            /***************** */
            loopCount++;
            if ((millis() - startTime) > 5000)
            {
                Serial.print("Average loops per second = ");
                Serial.println(loopCount / 5);
                startTime = millis();
                loopCount = 0;
            }
            /**************** */
            byte key_v = key_get();
            if (key_v == 1)
            {
                mode = 1 - mode;
                break;
            }

            if (tim1_IRQ_count == 1)
            {
                tft.setTextSize(2);
                tft.setTextColor(TFT_BLACK);
                tft.setCursor(60, 30);
                tft.print(lux_val, 0);
                tft.setCursor(60, 60);
                tft.print(t, 1);
                tft.setCursor(60, 90);
                tft.print(h, 1);
                tft.setCursor(60, 120);
                tft.print(co2_val);
                tft.setCursor(60, 150);
                tft.print(mq_adc_val);
                //
                h = dht.readHumidity();
                t = dht.readTemperature();
                lux_val = myLux.getLux();
                mq_adc_val = analogRead(MQ_ADC);
                const int packetSize = 6;
                byte packet[packetSize];
                if (Serial1.available() >= packetSize)
                {
                    for (int i = 0; i < packetSize; i++)
                    {
                        packet[i] = Serial1.read();
                    }
                    // 直接计算CO2浓度，不进行校验和检验
                    co2_val = (packet[1] * 256) + packet[2];
                }
                /*---------------------*/
                if (lux_val < en_bled)
                    digitalWrite(BLED_PIN, HIGH);
                else
                    digitalWrite(BLED_PIN, LOW);
                if (mq_adc_val > en_fan)
                    digitalWrite(FAN_PIN, HIGH);
                else if (mq_adc_val < de_fan)
                    digitalWrite(FAN_PIN, LOW);

                if (h < en_enhum)
                    digitalWrite(ENHUM_PIN, HIGH);
                else
                    digitalWrite(ENHUM_PIN, LOW);
                /*---------------------*/
                //
                tft.setCursor(0, 30);
                tft.setTextColor(TFT_GREEN);
                tft.setCursor(60, 30);
                tft.print(lux_val, 0);
                tft.setCursor(60, 60);
                tft.print(t, 1);
                tft.setCursor(60, 90);
                tft.print(h, 1);
                tft.setCursor(60, 120);
                tft.print(co2_val);
                tft.setCursor(60, 150);
                tft.print(mq_adc_val);

                tim1_IRQ_count = 0;
            }
            if (tim2_IRQ_count == 9)
            {
                sprintf(msg, "%.0f", lux_val);
                client.publish(sensor1_state_topic, msg);
                sprintf(msg, "%.0f", t);
                client.publish(sensor2_state_topic, msg);
                sprintf(msg, "%.0f", h);
                client.publish(sensor3_state_topic, msg);
                sprintf(msg, "%d", co2_val);
                client.publish(sensor4_state_topic, msg);
                sprintf(msg, "%d", mq_adc_val);
                client.publish(sensor5_state_topic, msg);

                tim2_IRQ_count = 0;
            }
        }
    }
    else if (mode == 1)
    {
        digitalWrite(LED1, HIGH);
        show_init1();
        max30102_heartrate_init();

        while (1)
        {
            if (!client.connected())
            {
                reconnect();
            }
            client.loop();
            /***************** */
            loopCount++;
            if ((millis() - startTime) > 5000)
            {
                Serial.print("Average loops per second = ");
                Serial.println(loopCount / 5);
                startTime = millis();
                loopCount = 0;
            }
            /**************** */
            byte key_v = key_get();
            if (key_v == 1)
            {
                mode = 1 - mode;
                max30102_heartrate_deinit();
                break;
            }

            max30102_BPM_get();

            // Serial.println(beatAvg);
            if (tim1_IRQ_count == 1)
            {
                spo2_1_time += 1;
                if (spo2_1_time > 10000)
                    spo2_1_time = 0;
                spo2_1_set(spo2_1_time);
                tft.fillRect(60, 30, 100, 90, TFT_BLACK);
                tft.setTextColor(TFT_GREEN);
                tft.setCursor(60, 30);
                tft.print(beatAvg);
                tft.setCursor(60, 60);
                tft.print(beatsPerMinute);
                tft.setCursor(60, 90);
                tft.print(spo2_1);
                tim1_IRQ_count = 0;
            }
            if (tim2_IRQ_count == 9)
            {
                sprintf(msg, "%d", beatAvg);
                client.publish(sensor6_state_topic, msg);
                sprintf(msg, "%.0f", beatsPerMinute);
                client.publish(sensor7_state_topic, msg);
                sprintf(msg, "%d", spo2_1);
                client.publish(sensor8_state_topic, msg);

                tim2_IRQ_count = 0;
            }
        }
    }
}
void Tim1Interrupt()
{ // 中断服务函数
    tim1_IRQ_count++;
    if (tim1_IRQ_count > 10)
        tim1_IRQ_count = 0;
}
void Tim2Interrupt()
{ // 中断服务函数
    tim2_IRQ_count++;
    if (tim2_IRQ_count > 10)
        tim2_IRQ_count = 0;
}
void show_init0()
{
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED);
    tft.setCursor(0, 30);
    tft.print("LUX:");
    tft.setCursor(0, 60);
    tft.print("TEM:");
    tft.setCursor(0, 90);
    tft.print("HUM:");
    tft.setCursor(0, 120);
    tft.print("CO2:");
    tft.setCursor(0, 150);
    tft.print("SMK:");
}
void show_init1()
{
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED);
    tft.setCursor(0, 30);
    tft.print("HR:  ");
    tft.setCursor(0, 60);
    tft.print("HR/m:");
    tft.setCursor(0, 90);
    tft.print("SPO2:");
}
byte key_get()
{
    byte keyval = 0;
    if (digitalRead(KEY1) == LOW)
    {
        delay(10);
        while (digitalRead(KEY1) == LOW)
            ;
        keyval = 1;
    }
    else if (digitalRead(KEY2) == LOW)
    {
        delay(10);
        while (digitalRead(KEY2) == LOW)
            ;
        keyval = 2;
    }
    else if (digitalRead(KEY3) == LOW)
    {
        delay(10);
        while (digitalRead(KEY3) == LOW)
            ;
        keyval = 3;
    }
    return keyval;
}
void max30102_heartrate_init()
{
    particleSensor.setup();                    // Configure sensor. Turn off LEDs
    particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED
                                               // particleSensor.enableDIETEMPRDY(); // Enable the temp ready interrupt. This is required.
}
void max30102_heartrate_deinit()
{
    particleSensor.setup(0); // Configure sensor. Turn off LEDs
}
void max30102_spo2_init()
{
    byte pulseLED = 38; // Must be on PWM pin   38
    byte readLED = 37;  // Blinks with each data read   37
    pinMode(pulseLED, OUTPUT);
    pinMode(readLED, OUTPUT);
    byte ledBrightness = 60; // Options: 0=Off to 255=50mA
    byte sampleAverage = 4;  // Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2;        // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 100;   // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411;    // Options: 69, 118, 215, 411
    int adcRange = 4096;     // Options: 2048, 4096, 8192, 16384

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with these settings
}
void max30102_spo2_get()
{
    bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps

    // read the first 100 samples, and determine the signal range
    for (byte i = 0; i < bufferLength; i++)
    {
        while (particleSensor.available() == false) // do we have new data?
            particleSensor.check();                 // Check the sensor for new data

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); // We're finished with this sample so move to next sample

        Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.println(irBuffer[i], DEC);
    }

    // calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while (1)
    {
        // dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
        for (byte i = 25; i < 100; i++)
        {
            redBuffer[i - 25] = redBuffer[i];
            irBuffer[i - 25] = irBuffer[i];
        }

        // take 25 sets of samples before calculating the heart rate.
        for (byte i = 75; i < 100; i++)
        {
            while (particleSensor.available() == false) // do we have new data?
                particleSensor.check();                 // Check the sensor for new data

            digitalWrite(readLED, !digitalRead(readLED)); // Blink onboard LED with every data read

            redBuffer[i] = particleSensor.getRed();
            irBuffer[i] = particleSensor.getIR();
            particleSensor.nextSample(); // We're finished with this sample so move to next sample

            // send samples and calculation result to terminal program through UART
            Serial.print(F("red="));
            Serial.print(redBuffer[i], DEC);
            Serial.print(F(", ir="));
            Serial.print(irBuffer[i], DEC);

            Serial.print(F(", HR="));
            Serial.print(heartRate, DEC);

            Serial.print(F(", HRvalid="));
            Serial.print(validHeartRate, DEC);

            Serial.print(F(", SPO2="));
            Serial.print(spo2, DEC);

            Serial.print(F(", SPO2Valid="));
            Serial.println(validSPO2, DEC);
        }

        // After gathering 25 new samples recalculate HR and SP02
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    }
}
float max30102_temp_get()
{
    float temperature = particleSensor.readTemperature();
    return temperature;
}
void max30102_BPM_get()
{
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true)
    {
        // We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
            rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
            rateSpot %= RATE_SIZE;                    // Wrap variable

            // Take average of readings
            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++)
                beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
        }
    }
    if (irValue < 50000)
    {
        // Serial.print(" No finger?");
        spo2_1_time = 0;
        digitalWrite(LED0, HIGH);
    }
    else
    {
        digitalWrite(LED0, LOW);
    }
}
void spo2_1_set(int time)
{
	/******************/
    spo2_1 = 100;
}

void setup_wifi()
{
    tft.fillScreen(TFT_BLACK);
    tft.setRotation(1);
    tft.setTextColor(TFT_WHITE);
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("WIFI connecting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        tft.print(".");
    }

    randomSeed(micros());

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    tft.fillScreen(TFT_BLACK);
    
    tft.setCursor(0, 40);
    tft.println("WIFI connected");
    tft.println(WiFi.localIP());
    delay(2000);
    tft.setRotation(0);
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        setup_wifi();
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "ESP8266Client-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str(), client_user, client_password))
        {
            Serial.println("connected");

            publishkey1_State();
            publishkey2_State();
            publishkey3_State();
            publishkey4_State();
            publishkey5_State();
            publishkey6_State();
            publishkey7_State();
            publishkey8_State();
            client.subscribe(key1_control_topic);
            client.subscribe(key2_control_topic);
            client.subscribe(key3_control_topic);
            client.subscribe(key4_control_topic);
            client.subscribe(key5_control_topic);
            client.subscribe(key6_control_topic);
            client.subscribe(key7_control_topic);
            client.subscribe(key8_control_topic);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}
void callback(char *p_topic, byte *p_payload, unsigned int p_length)
{
    digitalWrite(MQTT_LED, HIGH);
    String payload;
    Serial.println("INFO:callback...");
    for (uint8_t i = 0; i < p_length; i++)
    {
        payload.concat((char)p_payload[i]);
    }
    if (String(key1_control_topic).equals(p_topic))
    {
        if (payload.equals(String(DEVICE_ON)))
        {
            key1_state = true;
        }
        else if (payload.equals(String(DEVICE_OFF)))
        {
            key1_state = false;
        }
        setkey1_State();
        publishkey1_State();
    }
    else if (String(key2_control_topic).equals(p_topic))
    {
        if (payload.equals(String(DEVICE_ON)))
        {
            key2_state = true;
        }
        else if (payload.equals(String(DEVICE_OFF)))
        {
            key2_state = false;
        }
        setkey2_State();
        publishkey2_State();
    }
    else if (String(key3_control_topic).equals(p_topic))
    {
        if (payload.equals(String(DEVICE_ON)))
        {
            key3_state = true;
        }
        else if (payload.equals(String(DEVICE_OFF)))
        {
            key3_state = false;
        }
        setkey3_State();
        publishkey3_State();
    }
    else if (String(key4_control_topic).equals(p_topic))
    {
        if (payload.equals(String(DEVICE_ON)))
        {
            key4_state = true;
        }
        else if (payload.equals(String(DEVICE_OFF)))
        {
            key4_state = false;
        }
        setkey4_State();
        publishkey4_State();
    }
    else if (String(key5_control_topic).equals(p_topic))
    {
        if (payload.equals(String(DEVICE_ON)))
        {
            key5_state = true;
        }
        else if (payload.equals(String(DEVICE_OFF)))
        {
            key5_state = false;
        }
        setkey5_State();
        publishkey5_State();
    }
    else if (String(key6_control_topic).equals(p_topic))
    {
        if (payload.equals(String(DEVICE_ON)))
        {
            key6_state = true;
        }
        else if (payload.equals(String(DEVICE_OFF)))
        {
            key6_state = false;
        }
        setkey6_State();
        publishkey6_State();
    }
    else if (String(key7_control_topic).equals(p_topic))
    {
        if (payload.equals(String(DEVICE_ON)))
        {
            key7_state = true;
        }
        else if (payload.equals(String(DEVICE_OFF)))
        {
            key7_state = false;
        }
        setkey7_State();
        publishkey7_State();
    }
    else if (String(key8_control_topic).equals(p_topic))
    {
        if (payload.equals(String(DEVICE_ON)))
        {
            key8_state = true;
        }
        else if (payload.equals(String(DEVICE_OFF)))
        {
            key8_state = false;
        }
        setkey8_State();
        publishkey8_State();
    }

    digitalWrite(MQTT_LED, LOW);
}
void publishkey1_State()
{
    if (key1_state)
    {
        client.publish(key1_state_topic, DEVICE_ON, true);
    }
    else
    {
        client.publish(key1_state_topic, DEVICE_OFF, true);
    }
}
void publishkey2_State()
{
    if (key2_state)
    {
        client.publish(key2_state_topic, DEVICE_ON, true);
    }
    else
    {
        client.publish(key2_state_topic, DEVICE_OFF, true);
    }
}
void publishkey3_State()
{
    if (key3_state)
    {
        client.publish(key3_state_topic, DEVICE_ON, true);
    }
    else
    {
        client.publish(key3_state_topic, DEVICE_OFF, true);
    }
}

void publishkey4_State()
{
    if (key4_state)
    {
        client.publish(key4_state_topic, DEVICE_ON, true);
    }
    else
    {
        client.publish(key4_state_topic, DEVICE_OFF, true);
    }
}
void publishkey5_State()
{
    if (key5_state)
    {
        client.publish(key5_state_topic, DEVICE_ON, true);
    }
    else
    {
        client.publish(key5_state_topic, DEVICE_OFF, true);
    }
}
void publishkey6_State()
{
    if (key6_state)
    {
        client.publish(key6_state_topic, DEVICE_ON, true);
    }
    else
    {
        client.publish(key6_state_topic, DEVICE_OFF, true);
    }
}
void publishkey7_State()
{
    if (key7_state)
    {
        client.publish(key7_state_topic, DEVICE_ON, true);
    }
    else
    {
        client.publish(key7_state_topic, DEVICE_OFF, true);
    }
}
void publishkey8_State()
{
    if (key8_state)
    {
        client.publish(key8_state_topic, DEVICE_ON, true);
    }
    else
    {
        client.publish(key8_state_topic, DEVICE_OFF, true);
    }
}
void setkey1_State()
{
    if (key1_state)
    {
        digitalWrite(key1, HIGH);
        Serial.println("INFO: Turn key1 on...");
    }
    else
    {
        digitalWrite(key1, LOW);
        Serial.println("INFO: Turn key1 off...");
    }
}
void setkey2_State()
{
    if (key2_state)
    {
        digitalWrite(key2, HIGH);
        Serial.println("INFO: Turn key2 on...");
    }
    else
    {
        digitalWrite(key2, LOW);
        Serial.println("INFO: Turn key2 off...");
    }
}
void setkey3_State()
{
    if (key3_state)
    {
        digitalWrite(key3, HIGH);
        Serial.println("INFO: Turn key3 on...");
    }
    else
    {
        digitalWrite(key3, LOW);
        Serial.println("INFO: Turn key3 off...");
    }
}
void setkey4_State()
{
    if (key4_state)
    {
        digitalWrite(key4, HIGH);
        Serial.println("INFO: Turn key4 on...");
    }
    else
    {
        digitalWrite(key4, LOW);
        Serial.println("INFO: Turn key4 off...");
    }
}
void setkey5_State()
{
    if (key5_state)
    {
        digitalWrite(key5, HIGH);
        Serial.println("INFO: Turn key5 on...");
    }
    else
    {
        digitalWrite(key5, LOW);
        Serial.println("INFO: Turn key5 off...");
    }
}

void setkey6_State()
{
    if (key6_state)
    {
        digitalWrite(key6, HIGH);
        Serial.println("INFO: Turn key6 on...");
    }
    else
    {
        digitalWrite(key6, LOW);
        Serial.println("INFO: Turn key6 off...");
    }
}

void setkey7_State()
{
    if (key7_state)
    {
        digitalWrite(key7, HIGH);
        Serial.println("INFO: Turn key7 on...");
    }
    else
    {
        digitalWrite(key7, LOW);
        Serial.println("INFO: Turn key7 off...");
    }
}

void setkey8_State()
{
    if (key8_state)
    {
        digitalWrite(key8, HIGH);
        Serial.println("INFO: Turn key8 on...");
    }
    else
    {
        digitalWrite(key8, LOW);
        Serial.println("INFO: Turn key8 off...");
    }
}
