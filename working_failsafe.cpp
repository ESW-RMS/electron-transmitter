// Site-specific code for Cinta Mekar

#include "cellular_hal.h"
#include <math.h>

//set system mode
SYSTEM_MODE(SEMI_AUTOMATIC);

//set cellular APN
//STARTUP(cellular_credentials_set("internet", "wap", "wap123", NULL)); 

//set pins
int voltage = A0;
int cur1 = A1;
int cur2 = A2;
int cur3 = A3;

//create variables to store measurements
unsigned short current_1;
unsigned short current_2;
unsigned short current_3;
unsigned short voltage;
unsigned short freq;

double conversion_with_battery = 0.0008203445; // <-- old offset 0.00385561915
double conversion_without_battery = 0.0008025682181; //0.003772070625 <-- old offset
double i1_offset = 2044.568;
double i2_offset = 2047.938;
double i3_offset = 2046.561;
double i1o_no_battery = 1977.921;
double i2o_no_battery = 1980.809;
double i3o_no_battery = 1979.482;

double off_threshold = 20.0;

//set site data
String locationCode = "SUM";
double csdf = 4000; //current sensor downgrading factor 
unsigned long status_frequency = 5*60*1000; //milliseconds
unsigned long publish_frequency = 4*60*60*1000;
String data = "";
unsigned char offline;

long lastPublished;
long lastStatus;
bool inputActive;

Timer connectTimer(3*60*1000, resetElectron);

void resetElectron() {
    System.reset();
}

void setup() {
    Serial.begin(9600); // for testing
    
    //set pin modes
    pinMode(voltage, INPUT);
    pinMode(cur1, INPUT);
    pinMode(cur2, INPUT);
    pinMode(cur3, INPUT);
    lastPublished = 0;
    lastStatus = 0;
    EEPROM.get(2030, offline);

    //turns off cellular module
    Serial.println("turning off cellular");
    Cellular.off();
    Serial.println("delaying 5 sec");
    delay(5000);
    
}

void loop(){
    if (Time.format("%y").toInt() < 18 || Time.format("%y").toInt() > 70) {
        syncTime();
    } 
    if (millis()-lastStatus > status_frequency) {
        Serial.println("checking status");
        checkStatus();
        lastStatus = millis();
        if (offline == 'y') {
            if (inputActive) {
                putInEEPROM("on,", 1900);
                offline = 'n';
                EEPROM.put(2030, offline);
                publish();
                EEPROM.put(1900, 0xFF);
            }
        } else {
            if (!inputActive) {
                putInEEPROM("off,", 1800);
                offline = 'y';
                EEPROM.put(2030, offline);
                publish();
                EEPROM.put(1800, 0xFF);
            }
        }
        
    }
    if (millis()-lastPublished > publish_frequency) {
        Serial.println("publishing\n\n\n");
        publish();
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

//turns cellular on, attempts to connect to cellular
//if connected, makes measurement and attempts to publish to cloud
//if publishes, sets lastPublished to current time
//if does not connect to cellular, resets system
void publish() {
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
            if (publishToCloud()){
                Serial.println("publish worked");
                lastPublished = millis();
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

bool publishToCloud(){
    bool sent;
    bool include_time = true;
    int counter = 0;
    
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
    sent = Particle.publish("DATA",total, 60);
    if (sent) {
        EEPROM.put(1800, 0xFF);
        EEPROM.put(1900, 0xFF);
    }
    return true;
}