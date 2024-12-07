#include <ModbusMaster.h>
#include <SD.h>
#include <RTClib.h>
#include <SPI.h>

// Pin Definitions - Using Serial2 (Pins 16/17)
#define RS485_RX        16    // Hardware Serial2 RX
#define RS485_TX        17    // Hardware Serial2 TX
#define SD_CS_PIN       53    // CS pin for SD card on Mega 2560
#define LED_PIN         13    // Built-in LED on Mega 2560

// LED Timing (milliseconds)
#define FLASH_DURATION  50    // Duration of each LED flash
#define FLASH_GAP       150   // Gap between flashes

// PZEM-016 Configuration
#define PZEM_ADDR       0x01  // Default Modbus address
#define SERIAL_BAUD     9600  // Baud rate for both Serial interfaces

// Modbus Register Addresses
#define REG_VOLTAGE     0x0000
#define REG_TOTAL_REGS  10    // Number of registers to read

// Register Positions (offset from first register)
#define POS_VOLTAGE      0x00
#define POS_CURRENT_LOW  0x01
#define POS_CURRENT_HIGH 0x02
#define POS_POWER_LOW    0x03
#define POS_POWER_HIGH   0x04
#define POS_ENERGY_LOW   0x05
#define POS_ENERGY_HIGH  0x06
#define POS_FREQUENCY    0x07
#define POS_POWER_FACTOR 0x08
#define POS_ALARMS       0x09

// Conversion Factors
#define VOLTAGE_FACTOR    10.0   // Divide by 10 to get voltage in V
#define CURRENT_FACTOR    1000.0 // Divide by 1000 to get current in A
#define POWER_FACTOR      10.0   // Divide by 10 to get power in W
#define FREQ_FACTOR       10.0   // Divide by 10 to get frequency in Hz
#define PF_FACTOR         100.0  // Divide by 100 to get power factor
#define READING_INTERVAL  2000   // Delay between readings in ms

// File Configuration
#define LOG_FILENAME    "POWER.CSV"
#define WRITE_BUFFER_SIZE 128

// Global Objects
ModbusMaster node;
RTC_DS1307 rtc;
File logFile;

// Function Prototypes
void setupRS485(void);
void setupModbus(void);
void setupRTC(void);
void setupSD(void);
void readAndLogValues(void);
void writeHeaderToSD(void);

float calculateCurrent(uint16_t lowWord, uint16_t highWord);
float calculatePower(uint16_t lowWord, uint16_t highWord);
uint32_t calculateEnergy(uint16_t lowWord, uint16_t highWord);
void logReadingToSD(DateTime timestamp, float voltage, float current,
                    float power, uint32_t energy, float frequency,
                    float powerFactor, uint16_t alarms);

void preTransmission(void) {
    //  No DE pin handling needed with MAX13487
}

void postTransmission(void) {
    //  No DE pin handling needed with MAX13487
}

void setup() {
    Serial.begin(SERIAL_BAUD);  //  Using hardware Serial for debugging
    setupRS485();
    setupModbus();
    setupRTC();
    setupSD();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // The F() macro stores strings in flash memory to save RAM
    Serial.println(F("PZEM-016 Reader with Data Logging Started"));
}

void setupRS485(void) {
    Serial2.begin(SERIAL_BAUD);
}

void setupModbus(void) {
    node.begin(PZEM_ADDR, Serial2);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
}

void setupRTC(void) {
    if (!rtc.begin()) {
        Serial.println(F("Could not find RTC"));
        while (1);
    }

    if (!rtc.isrunning()) {
        Serial.println(F("RTC is NOT running, setting to compile time"));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
}

void setupSD(void) {
    pinMode(SD_CS_PIN, OUTPUT);

    if (!SD.begin(SD_CS_PIN)) {
        Serial.println(F("SD card initialization failed!"));
        while (1);
    }
    Serial.println(F("SD card initialized."));

    if (!SD.exists(LOG_FILENAME)) {
        writeHeaderToSD();
    }
}

void writeHeaderToSD(void) {
    logFile = SD.open(LOG_FILENAME, FILE_WRITE);
    if (logFile) {
        logFile.println(F("Timestamp,Voltage(V),Current(A),Power(W),"
                       "Energy(Wh),Frequency(Hz),PowerFactor,Alarms"));
        logFile.close();
    } else {
        Serial.println(F("Error opening log file for header!"));
    }
}

void heartbeatLED(void) {
    digitalWrite(LED_PIN, HIGH);
    delay(FLASH_DURATION);
    digitalWrite(LED_PIN, LOW);
    delay(FLASH_GAP);

    digitalWrite(LED_PIN, HIGH);
    delay(FLASH_DURATION);
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    readAndLogValues();
    heartbeatLED();
    delay(READING_INTERVAL - (2 * FLASH_DURATION + FLASH_GAP));
}

float calculateCurrent(uint16_t lowWord, uint16_t highWord) {
    uint32_t temp = lowWord + ((uint32_t)highWord << 16);
    return temp / CURRENT_FACTOR;
}

float calculatePower(uint16_t lowWord, uint16_t highWord) {
    uint32_t temp = lowWord + ((uint32_t)highWord << 16);
    return temp / POWER_FACTOR;
}

uint32_t calculateEnergy(uint16_t lowWord, uint16_t highWord) {
    return lowWord + ((uint32_t)highWord << 16);
}

void logReadingToSD(DateTime timestamp, float voltage, float current,
                    float power, uint32_t energy, float frequency,
                    float powerFactor, uint16_t alarms) {
    char buf[32];  // Increased buffer size for Mega

    logFile = SD.open(LOG_FILENAME, FILE_WRITE);

    if (logFile) {
        // Write timestamp directly
        logFile.print(timestamp.year());
        logFile.print('-');
        if (timestamp.month() < 10) logFile.print('0');
        logFile.print(timestamp.month());
        logFile.print('-');
        if (timestamp.day() < 10) logFile.print('0');
        logFile.print(timestamp.day());
        logFile.print(' ');
        if (timestamp.hour() < 10) logFile.print('0');
        logFile.print(timestamp.hour());
        logFile.print(':');
        if (timestamp.minute() < 10) logFile.print('0');
        logFile.print(timestamp.minute());
        logFile.print(':');
        if (timestamp.second() < 10) logFile.print('0');
        logFile.print(timestamp.second());
        logFile.print(',');

        // Write measurements
        dtostrf(voltage, 6, 2, buf);
        logFile.print(buf); logFile.print(',');

        dtostrf(current, 6, 3, buf);
        logFile.print(buf); logFile.print(',');

        dtostrf(power, 6, 2, buf);
        logFile.print(buf); logFile.print(',');

        logFile.print(energy); logFile.print(',');

        dtostrf(frequency, 5, 2, buf);
        logFile.print(buf); logFile.print(',');

        dtostrf(powerFactor, 5, 3, buf);
        logFile.print(buf); logFile.print(',');

        logFile.println(alarms, HEX);

        logFile.close();
    } else {
        Serial.println(F("Error opening log file for data!"));
    }
}

void readAndLogValues(void) {
    uint8_t result;
    DateTime now;

    result = node.readInputRegisters(REG_VOLTAGE, REG_TOTAL_REGS);

    if (result == node.ku8MBSuccess) {
        float voltage, current, power, frequency, powerFactor;
        uint32_t energy;
        uint16_t alarms;

        now = rtc.now();

        voltage = node.getResponseBuffer(POS_VOLTAGE) / VOLTAGE_FACTOR;
        current = calculateCurrent(
            node.getResponseBuffer(POS_CURRENT_LOW),
            node.getResponseBuffer(POS_CURRENT_HIGH)
        );
        power = calculatePower(
            node.getResponseBuffer(POS_POWER_LOW),
            node.getResponseBuffer(POS_POWER_HIGH)
        );
        energy = calculateEnergy(
            node.getResponseBuffer(POS_ENERGY_LOW),
            node.getResponseBuffer(POS_ENERGY_HIGH)
        );
        frequency = node.getResponseBuffer(POS_FREQUENCY) / FREQ_FACTOR;
        powerFactor = node.getResponseBuffer(POS_POWER_FACTOR) / PF_FACTOR;
        alarms = node.getResponseBuffer(POS_ALARMS);

        logReadingToSD(now, voltage, current, power, energy,
                      frequency, powerFactor, alarms);

        Serial.print(now.timestamp());
        Serial.print(F(" - V:"));
        Serial.print(voltage);
        Serial.print(F("V, I:"));
        Serial.print(current);
        Serial.print(F("A, P:"));
        Serial.print(power);
        Serial.println(F("W"));
    } else {
        Serial.println(F("Failed to read from PZEM"));
    }
}
