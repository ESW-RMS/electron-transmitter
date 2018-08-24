/*

  Stanford Engineers for a Sustainable World
  Remote Monitoring System | August 2018
  Michael Lin | mlin4@stanford.edu

  File: version_3.cpp
  --------------------------
  Main electron code

*/

#include "cellular_hal.h"
#include <math.h>
#include "application.h"
#include "sensors.h"

//for version 3, define status_change and measure
//for version 2, define measure
//for failsafe, define status_change

#define STATUS_CHANGE
#define MEASURE

//set system mode
SYSTEM_MODE(SEMI_AUTOMATIC);

//set cellular APN
//STARTUP(cellular_credentials_set("internet", "wap", "wap123", NULL)); 

Sensors Sensorboard;

unsigned long status_frequency = 5*60*1000; //milliseconds
unsigned long measurement_frequency = 60*60*1000; //change to 5*60*1000 for testing
unsigned long publish_frequency = 4*60*60*1000; //change to 5*60*1000 for testing
String data = "";
unsigned char publishedAll;
unsigned char offline;

long lastPublished;
long lastMeasured;
long lastStatus;

void resetElectron() {
    System.reset();
}

Timer connectTimer(3*60*1000, resetElectron);

void syncTime();
void putInEEPROM(String message, int address);
void publish(bool regular);
bool publishToCloud();
void storeMeasurements();
void publishStatus();

void setup() {
    Serial.begin(9600); // for testing
    
    Sensorboard.init();

    lastPublished = 0;
    lastMeasured = 0;
    lastStatus = 0;
    EEPROM.get(2000, publishedAll);
    EEPROM.get(2030, offline);

    //turns off cellular module
    Serial.println("turning off cellular");
    Cellular.off();
    Serial.println("delaying 5 sec");
    delay(5000);
    
}

void loop(){
    Sensorboard.fieldTest();
    if (Time.format("%y").toInt() < 18 || Time.format("%y").toInt() > 70) {
        syncTime();
    } 
    #ifdef STATUS_CHANGE
    if (millis()-lastStatus > status_frequency) {
        Serial.println("checking status");
        Sensorboard.refreshStatus();
        lastStatus = millis();
        if (offline == 'y') {
            if (Sensorboard.generatorIsOn()) {
                putInEEPROM("on,", 1900);
                offline = 'n';
                EEPROM.put(2030, offline);
                publish(false);
                EEPROM.put(1900, 0xFF);
            }
        } else {
            if (!Sensorboard.generatorIsOn()) {
                putInEEPROM("off,", 1800);
                offline = 'y';
                EEPROM.put(2030, offline);
                publish(false);
                EEPROM.put(1800, 0xFF);
            }
        }
    }
    #endif
    #ifdef MEASURE
    if (millis() - lastMeasured > measurement_frequency) {
        Serial.println("measuring\n\n\n");
        Sensorboard.refreshAll();
        storeMeasurements();
    }
    #endif
    if ((millis()-lastPublished > publish_frequency) || publishedAll == 'n') {
        Serial.println("publishing\n\n\n");
        publish(true);
    }
}

void syncTime() {
    cellular_credentials_set("internet", "wap", "wap123", NULL);
    Cellular.on();
    connectTimer.start();
    Cellular.connect();
    connectTimer.reset();
    connectTimer.stop();
    delay(30*1000);
    if (Cellular.ready()){
        Particle.connect();
        delay(1*60*1000);
        if (Particle.connected()){
            Particle.syncTime();
            delay(10*1000);
        }
        Particle.disconnect();
    } else {
        System.reset();
    }
    Cellular.disconnect();
    Cellular.off();
}

void putInEEPROM(String message, int address) { //this is to put the time off/on into the eeprom
    const int STRING_BUF_SIZE = 14;
    char stringBuf[STRING_BUF_SIZE];
    stringBuf[sizeof(stringBuf) - 1] = 0; // make sure it's null terminated
    String str = Time.format("%d%m%y%H%M");
    message += str;
    message.toCharArray(stringBuf, sizeof(stringBuf));
    EEPROM.put(address, stringBuf);
}

void storeMeasurements() { //EEPROM
    uint8_t value;
    EEPROM.get(0, value);
    uint8_t num;
    EEPROM.get(2010, num);
    if (value == 0xFF || value == 140) {
        // EEPROM was empty -> initialize value
        if (value != 140) num = 0;
        value = 0;
    }
    int address = value * 12 + 1;
    EEPROM.put(address,Sensorboard.getCurrent_1());
    EEPROM.put(address+2,Sensorboard.getCurrent_2());
    EEPROM.put(address+4,Sensorboard.getCurrent_3());
    EEPROM.put(address+6,Sensorboard.getFrequency());
    EEPROM.put(address+8,Sensorboard.getVoltage());
    EEPROM.put(address+10, Sensorboard.getPower());
    value++;
    EEPROM.put(0, value);
    if (num < 140) {
        num++;
        EEPROM.put(2010, num);
        EEPROM.put(2020, 0xFF);
    } else {
        EEPROM.put(2020, value);
    }
    Serial.println("finished storing measurements\n\n\n");
}

//turns cellular on, attempts to connect to cellular
//if connected, makes measurement and attempts to publish to cloud
//if publishes, sets lastPublished to current time
//if does not connect to cellular, resets system
void publish(bool regular) { //if regular, measure publish, if !regular, status publish
    cellular_credentials_set("internet", "wap", "wap123", NULL);
    Serial.println("calling cellular.on");
    Cellular.on();
    
    Serial.println("starting timer");
    connectTimer.start();
    
    Serial.println("calling cellular.connect");
    Cellular.connect();
    
    Serial.println("resetting timer");
    connectTimer.reset();
    Serial.println("stopping timer");
    connectTimer.stop();
    delay(1*30*1000);
    if (Cellular.ready()){
        Serial.println("connecting to electron");
        Particle.connect();
        delay(1*60*1000);
        if (Particle.connected()){
            Serial.println("particle connected, trying to publish to cloud");
            if (regular == false) {
                publishStatus();
                #ifdef STATUS_CHANGE
                #ifndef MEASURE
                lastPublished = millis();
                #endif
                #endif
            } else if (publishToCloud()){
                Serial.println("publish worked");
                lastPublished = lastMeasured;
            }
            delay(5*60*1000);
        }
        Serial.println("disconnecting from cloud");
        Particle.disconnect();
    } else {
        Serial.println("system is being reset");
        System.reset();
    }
    Serial.println("disconnecting from cellular");
    Cellular.disconnect();
    Serial.println("turning cellular off");
    Cellular.off();
}

void publishStatus(){
    bool sent;

    unsigned char stopped;
    EEPROM.get(1800, stopped);
    unsigned char started;
    EEPROM.get(1900, started);
    String total = "";
    if (stopped != 0xFF) {
        char stringBuf[14];
        EEPROM.get(1800, stringBuf);
        String off(stringBuf);
        total += off;
    }
    if (started != 0xFF) {
        char stringBuf[14];
        EEPROM.get(1900, stringBuf);
        String on(stringBuf);
        total += on;
    }
    if (total != "") {
        sent = Particle.publish("DATA",total, 60);
        if (sent) {
            EEPROM.put(1800, 0xFF);
            EEPROM.put(1900, 0xFF);
        }
    }
}

bool publishToCloud(){
    bool sent;
    bool include_time = true;
    int counter = 0;
    
    #ifdef STATUS_CHANGE
    publishStatus();
    #endif

    uint8_t value;
    EEPROM.get(0, value);
    uint8_t num;
    EEPROM.get(2010, num);
    data = String::format("%u", num);
    uint8_t temp;
    EEPROM.get(2020, temp);
    
    while (value > 0) {
        int address = value * 12 - 1; 
        unsigned short power;
        unsigned short v;
        unsigned short freq;
        unsigned short i1;
        unsigned short i2;
        unsigned short i3;
        EEPROM.get(address, power);
        EEPROM.get(address - 2, v);
        EEPROM.get(address - 4, freq);
        EEPROM.get(address - 6, i3);
        EEPROM.get(address - 8, i2);
        EEPROM.get(address - 10, i1);
        data += String::format(",%u,%u,%u,%u,%u,%u", i1, i2, i3, freq, v, power);
        Serial.println(data);
        value--;
        num--;
        if (value == 0) {
            uint16_t info;
            EEPROM.get(1679, info);
            if (info != 0xFFFF) value = 140;
        }
        counter++;
        #ifndef TEST
        if (counter == 4) {
            Serial.println("this is what im publishing: " + data);
            if (publishedAll != 'n' && include_time) {
                String t = Time.format(",%d%m%y%H%M");
                data += t;
            } else {
                data += ",0";
            }
            sent = Particle.publish("DATA",data, 60);
            delay(1000);
            counter = 0;
            data = String::format("%u", num);
            if (sent) {
                EEPROM.put(0, value);
                EEPROM.put(2010, num);
                include_time = false;
                if (num != 0) EEPROM.put(2000, 'n');
                else EEPROM.put(2000, 'y');
            } else {
                return false;
            }
        }
        #endif
        #ifdef TEST
        sent = Particle.publish("DATA", data, 60);
        if (!sent) return false;
        #endif
        if (value == temp) break;
    }
    Serial.println("eeprom is being cleared");
    value = 0;
    EEPROM.clear();
    publishedAll = 'y';
    return true;
}