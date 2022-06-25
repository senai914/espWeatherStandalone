/**
 ******************************************************************************
 * @file    main.cpp
 * @author  Professor Kleber Lima da Silva <kleber.lima@sp.senai.br>
 * @version V0.1.0
 * @date    26-Jun-2022
 * @brief   Code for Weather Station (Standalone Mode) - SENAI 9.14.
 ******************************************************************************
 */

/* Includes -----------------------------------------------------------------*/
#include <esp_task_wdt.h>
#include <SPI.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <LoRa.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <ThingsBoard.h>
#include <NTPClient.h>


/* Defines ------------------------------------------------------------------*/
#define ENABLE_WIFI     1   /* 1 to enable WIFI connection */

#define WIFI_AP_NAME    "KleberW6"
#define WIFI_PASSWORD   "wifiKleber6"

#define TOKEN           "nKIiGpevYLNYWgxTbtYf"
#define TB_SERVER       "demo.thingsboard.io"
#define INTERVAL_LOOP   1000
#define INTERVAL_SEND   60000

#define DHTTYPE DHT22
#define CTE_CAL_ANEMOMETRO  1.224f  /* 1 hz = 1.224 km/h */
#define CTE_CAL_PLUVIOMETRO 0.33f   /* 1 tick = 0.33 mm */
#define INTERVAL_SENSORS    INTERVAL_SEND


/* Macros -------------------------------------------------------------------*/

/* Pin numbers --------------------------------------------------------------*/
const int WIND_DIRECTION = 36;  /* Wind Direction Sensor Pin */
const int WIND_SPEED = 18;      /* Wind Speed Sensor Pin */
const int RAIN_GAUGE = 4;       /* Rain Guage Sensor Pin */
const int DHT_DATA = 19;        /* DHT22 DATA Pin */
const int BMP_SCL = 22;         /* BMP180 SCL Pin */
const int BMP_SDA = 21;         /* BMP180 SDA Pin */
const int LORA_SCK = 32;        /* LORA Module SCK Pin */
const int LORA_MISO = 33;       /* LORA Module MISO Pin */
const int LORA_MOSI = 25;       /* LORA Module MOSI Pin */
const int LORA_NSS = 26;        /* LORA Module NSS Pin */
const int LORA_DIO0 = 26;       /* LORA Module DIO0 Pin */
const int LORA_RESET = 14;      /* LORA Module RESET Pin */


/* Private variables --------------------------------------------------------*/
WiFiClient espClient;
ThingsBoard tb(espClient);
uint32_t interval = 0;
uint8_t status = WL_IDLE_STATUS;

WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP);
int day = 0;

DHT dht(DHT_DATA, DHTTYPE);
Adafruit_BMP085 bmp;

volatile uint32_t countWindSpeed = 0;
volatile uint32_t countRainGauge = 0;

float tempValue = 0, humidityValue = 0;
float rainValue = 0, wSpeedValue = 0;
uint8_t wDirectionValue = 0;
int32_t pressureValue = 0;
int8_t rssiValue = 0;

int adc[8] = {3840, 3460, 2960, 2330, 1670, 970, 560, 200};
String strDirection[8] = {"W", "NW", "N", "SW", "NE", "S", "SE", "E"};
uint32_t angleDirection[8] = {270, 315, 0, 225, 45, 180, 135, 90};


/* Private Functions Prototypes ---------------------------------------------*/
void connectWifi();
float getRain();
float getWindSpeed();
uint8_t getWindDirection();


/* Callbacks Functions ------------------------------------------------------*/
void cbWindSpeed()
{
    countWindSpeed++;
}

void cbRainGauge()
{
    countRainGauge++;
}


/* Setup --------------------------------------------------------------------*/
void setup()
{
    /* Pins initializations */
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(WIND_SPEED, INPUT_PULLUP);
    pinMode(RAIN_GAUGE, INPUT_PULLUP);
    digitalWrite(LED_BUILTIN, HIGH);

    /* Serial debug initialization */
    Serial.begin(9600);
    while (!Serial);
    Serial.println("Weather Station| SENAI 9.14");

    /* Wi-Fi initialization */
    Serial.println("Connecting to AP ...");
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    connectWifi();

    /* NTP configure: -3 = -10800 (BRASIL) */
    ntp.begin();
    ntp.setTimeOffset(-10800);

    /* Init interrupts */
    attachInterrupt(WIND_SPEED, cbWindSpeed, FALLING);
    attachInterrupt(RAIN_GAUGE, cbRainGauge, FALLING);

    dht.begin();
    
    if (!bmp.begin())
    {
	    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    }

    /* Enable Watchdog timeout 4s */
    esp_task_wdt_init(4, true);
    esp_task_wdt_add(NULL);

    /* End of initializations */
    digitalWrite(LED_BUILTIN, LOW);

    /*while (1) // wind direction calibration
    {
        Serial.println(analogRead(WIND_DIRECTION));
        Serial.println(direcoes[getWindDirection()]);
        delay(1000);
    }*/
}

/* Main Loop ----------------------------------------------------------------*/
void loop()
{
    /* Main loop interval */
    delay(INTERVAL_LOOP);
    interval += INTERVAL_LOOP;

    /* Reconnect to WiFi, if needed */
    if (WiFi.status() != WL_CONNECTED)
    {
        connectWifi();
        return;
    }

    /* Reconnect to ThingsBoard, if needed */
    if (tb.connected() == false)
    {
        /* Connect to the ThingsBoard */
        Serial.print("Connecting to: ");
        Serial.print(TB_SERVER);
        Serial.print(" with token ");
        Serial.println(TOKEN);
        if (tb.connect(TB_SERVER, TOKEN) == false)
        {
            Serial.println("Failed to connect to Thingsboard");
            return;
        }
    }

    /* Check if it is a time to send telemetry data */
    if (interval > INTERVAL_SEND)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Reading sensors...");

        /* Read all sensors */
        float aux;
        aux = dht.readTemperature();
        if (aux != NAN) tempValue = aux;
        aux = dht.readHumidity();
        if (aux != NAN) humidityValue = aux;
        pressureValue = bmp.readPressure();

        rainValue = getRain();
        wSpeedValue = getWindSpeed();
        wDirectionValue = getWindDirection();

        rssiValue = WiFi.RSSI();

        /* NTP time */
        if (ntp.update())
        {
            Serial.print("Relogio: ");
            Serial.println(ntp.getFormattedTime());

            /* Reset Rain Gauge (new day) */
            if (day != ntp.getDay())
            {
                Serial.println("New day");
                
                countRainGauge = 0;
                day = ntp.getDay();
            }
        }

        /* Serial Debug */
        Serial.print("Temperatura (Celsius): ");
        Serial.println(tempValue, 2);
        
        Serial.print("Umidade do Ar (%): ");
        Serial.println(humidityValue, 2);

        Serial.print("Pressão Atmosférica (Pa): ");
        Serial.println(pressureValue);

        Serial.print("Chuva (mm): ");
        Serial.println(rainValue, 3);

        Serial.print("Vento (km/h): ");
        Serial.println(wSpeedValue, 3);
        
        Serial.print("Direcao: ");
        Serial.println(strDirection[wDirectionValue]);

        Serial.print("RSSI: ");
        Serial.println(rssiValue);

        /* Sending Data */
        Serial.print("Sending data...");
#if TEST_DASHBOARD == 1     /* Random test data */
        tb.sendTelemetryFloat("temperature", random(0, 500) / 10.0f);
        tb.sendTelemetryFloat("humidity", random(0, 1000) / 10.0f);
        tb.sendTelemetryFloat("rain", random(0, 500) / 10.0f);
        tb.sendTelemetryFloat("pressure", random(8000, 12000) / 10.0f);
        tb.sendTelemetryFloat("winddirection", random(0, 3600) / 10.0f);
        tb.sendTelemetryFloat("windspeed", random(0, 500) / 10.0f);
#else   /* Real sensors data */
        tb.sendTelemetryFloat("temperature", tempValue);
        tb.sendTelemetryFloat("humidity", humidityValue);
        tb.sendTelemetryFloat("rain", rainValue);
        tb.sendTelemetryFloat("pressure", pressureValue);
        tb.sendTelemetryFloat("winddirection", angleDirection[wDirectionValue]);
        tb.sendTelemetryFloat("windspeed", wSpeedValue);
        tb.sendTelemetryFloat("rssi", rssiValue);
#endif
        interval = 0;
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("ok");
    }

    /* Process messages */
    //tb.loop();

    /* STATUS LED blink */
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);

    /* Watchdog reset */
    esp_task_wdt_reset();
}


/* Accumulated Rain ---------------------------------------------------------*/
float getRain()
{
    float rain;

    rain = countRainGauge * CTE_CAL_PLUVIOMETRO;

    return rain;
}


/* Wind Speed ---------------------------------------------------------------*/
float getWindSpeed()
{
    float avgSpeed;

    avgSpeed = countWindSpeed;
    avgSpeed *= 1000.0 * CTE_CAL_ANEMOMETRO;
    avgSpeed /= INTERVAL_SENSORS;

    countWindSpeed = 0;
    return avgSpeed;
}


/* Wind Direction -----------------------------------------------------------*/
uint8_t getWindDirection()
{
    uint8_t direction = 0;
    uint16_t value = 0;

    for (int i = 0 ; i < 8 ; i++) value += analogRead(WIND_DIRECTION);
	value /= 8;

    for (direction = 0; direction < 8; direction++)
    {
        if (value >= adc[direction])
            break;
    }

    return direction;
}


/* Wi-Fi connection ---------------------------------------------------------*/
void connectWifi()
{
    status = WiFi.status();

#if ENABLE_WIFI == 1
    /* Loop until we're reconnected */
    if (status != WL_CONNECTED)
    {
        WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(1000);
            Serial.print(".");

            WiFi.disconnect();
            WiFi.reconnect();

            esp_task_wdt_reset();
        }
        Serial.println("Connected to AP");
    }
#endif
}
