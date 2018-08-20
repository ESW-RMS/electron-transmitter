// Site-specific code for Cinta Mekar

#include "cellular_hal.h"
#include <math.h>
#include "application.h"

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
unsigned short i1;
unsigned short i2;
unsigned short i3;
unsigned short v;
unsigned short freq;

double conversion_with_battery = 0.0008203445; // <-- old offset 0.00385561915
double conversion_without_battery = 0.0008025682181; //0.003772070625 <-- old offset
double i1_offset = 2044.568;
double i2_offset = 2047.938;
double i3_offset = 2046.561;
double i1o_no_battery = 1977.921;
double i2o_no_battery = 1980.809;
double i3o_no_battery = 1979.482;

//set site data
String locationCode = "SUM";
double csdf = 4000; //current sensor downgrading factor 
unsigned long status_frequency = 5*60*1000; //milliseconds
unsigned long measurement_frequency = 60*60*1000;
unsigned long publish_frequency = 4*60*60*1000;
String data = "";
unsigned char publishedAll;
unsigned char offline;

long lastPublished;
long lastMeasured;
long lastStatus;
bool inputActive;

void resetElectron() {
    System.reset();
}

Timer connectTimer(3*60*1000, resetElectron);

void syncTime();
void putInEEPROM(String message, int address);
void publish(bool regular);
bool publishToCloud();
void checkStatus();
void storeMeasurements();
void measure();
void publishStatus();
double test_voltage_rms(int pin);
double voltage_on_current(int cur);
double current_RMS(int cur);
double frequency(int vpin);

void setup() {
    Serial.begin(9600); // for testing
    
    //set pin modes
    pinMode(voltage, INPUT);
    pinMode(cur1, INPUT);
    pinMode(cur2, INPUT);
    pinMode(cur3, INPUT);
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
                publish(false);
                EEPROM.put(1900, 0xFF);
            }
        } else {
            if (!inputActive) {
                putInEEPROM("off,", 1800);
                offline = 'y';
                EEPROM.put(2030, offline);
                publish(false);
                EEPROM.put(1800, 0xFF);
            }
        }
    }
    if (millis() - lastMeasured > measurement_frequency) {
        Serial.println("measuring\n\n\n");
        measure();
        storeMeasurements();
    }
    if ((millis()-lastPublished > publish_frequency) || publishedAll == 'n') {
        Serial.println("publishing\n\n\n");
        publish(true);
    }
}

void checkStatus(){}

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
    if (value == 0xFF || value == 168) {
        // EEPROM was empty -> initialize value
        if (value != 168) num = 0;
        value = 0;
    }
    int address = value * 10 + 1;
    EEPROM.put(address,i1);
    EEPROM.put(address+2,i2);
    EEPROM.put(address+4,i3);
    EEPROM.put(address+6,freq);
    EEPROM.put(address+8,v);
    value++;
    EEPROM.put(0, value);
    if (num < 168) {
        num++;
        EEPROM.put(2010, num);
        EEPROM.put(2020, 0xFF);
    } else {
        EEPROM.put(2020, value);
    }
    Serial.println("finished storing measurements\n\n\n");
}

// makes measurements and stores in i1, i2, i3, and v
void measure() {
    v = (unsigned short) (test_voltage_rms(voltage)*100);
    i1 = (unsigned short) (current_RMS(cur1)*100);
    i2  = (unsigned short) (current_RMS(cur2)*100);
    i3 = (unsigned short) (current_RMS(cur3)*100);
    freq = (unsigned short) (frequency(voltage)*100);
    lastMeasured = millis();
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
            if (!regular) {
                publishStatus();
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
    } else {
        sent = Particle.publish("DATA", "connected", 60);
    }
}

bool publishToCloud(){
    bool sent;
    bool include_time = true;
    int counter = 0;
    
    publishStatus();

    uint8_t value;
    EEPROM.get(0, value);
    uint8_t num;
    EEPROM.get(2010, num);
    data = String::format("%u", num);
    uint8_t temp;
    EEPROM.get(2020, temp);
    
    while (value > 0) {
        int address = value * 10 - 1; 
        EEPROM.get(address, v);
        EEPROM.get(address - 2, freq);
        EEPROM.get(address - 4, i3);
        EEPROM.get(address - 6, i2);
        EEPROM.get(address - 8, i1);
        data += String::format(",%u,%u,%u,%u,%u", i1, i2, i3, freq, v);
        Serial.println(data);
        value--;
        num--;
        if (value == 0) {
            uint16_t info;
            EEPROM.get(1679, info);
            if (info != 0xFFFF) value = 168;
        }
        counter++;
        if (counter == 8) {
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
        if (value == temp) break;
    }
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
        if (sent) {
            EEPROM.put(2000, 'y');
        } else {
            return false;
        }
    }
    Serial.println("eeprom is being cleared");
    value = 0;
    EEPROM.clear();
    publishedAll = 'y';
    return true;
}

//measures RMS of voltage pin by taking 1000 samples and computing discrete time RMS of the data
double test_voltage_rms(int pin){
    double sum = 0.0;
    for (int i=0; i<1000; i++){
        double vtemp = analogRead(pin);
        sum+=(vtemp*vtemp);
        delay(1);
    }
    //found sum of squares of 1000 measurements, 50 periods
    return sqrt(2.0*sum/1000.0)*(conversion_with_battery)*(576.0/4.7);
    //calculated voltage by accounting for voltage divider, Electron measuremnt protocol and taking the rms via sqrting the sum
}

double voltage_on_current(int cur){
    double offset = 0.0;
    if (cur == cur1) offset = i1o_no_battery;
    if (cur == cur2) offset = i2o_no_battery;
    if (cur == cur3) offset = i3o_no_battery;
    double I_bin = analogRead(cur);
    //Serial.print(I_bin);
    //Serial.print("     ");
    double v_measured = (I_bin-offset)*conversion_without_battery;
    double i = (v_measured)*85.11; //4000.0/47.0;
    //Serial.println(i);
    return i;
}

//computes the RMS of the current waveform by sampling 1000 times and taking discrete time RMS of data
double current_RMS(int cur){
    double i_sum = 0.0;
    double current_cur = 0.0;
    for(int i = 0; i < 1000; i++){
        current_cur = voltage_on_current(cur);
        i_sum = i_sum + current_cur*current_cur;
    }
    double i_rms = sqrt(i_sum/1000.0);
    //Serial.println(i_rms);
    delay(1000);
    return i_rms;
}

double frequency(int vpin){
    //Serial.println("in frequency");
    int count = 0;
    bool check = false;
    bool ran = false;
    double threshold = 20; // NEED TO DETERMINE A THRESHOLD
    double v1 = 0;
    int t1 = 0;
    int t2 = 0;
    
    while(t1 < 5000){   //stay in loop for 5 seconds, check voltage and see if below threshold
        //Serial.print("in 1st loop: ");
        //Serial.println(t1);
        v1 = analogRead(vpin);
        //Serial.println(v1);
        if (v1 > threshold) check = true;
        while (check == true && v1 < threshold && t2 < 5000) {  //once crossed threshold, initiate counter. stay for a max of 5 seconds
            ran = true;
            count++;
            t2++;
            v1 = analogRead(vpin);
            delayMicroseconds(1);
            //Serial.print("in 2nd loop: ");
            //Serial.print("Tlow = ");
            //Serial.print(t2);
            //Serial.print("  Voltage = ");
            //Serial.println(v1);
        }
        if (ran) break;
        delayMicroseconds(1);
        t1++;
    }
    if(t1 > 4999) return 0; //if an invalid measurement return 0
    double f = 1.0/count/2.0/0.00009208103; //This is the equation for frequency... 1/F where count is the number of samples 
                                        //found in the down half wave and the .000555 constant was found experimentally. 
                                        //Kinda janky but due to delays of functionality it won't delay for an exact enough time
    //Serial.print(count);
    //Serial.print("  ");
    //Serial.println(f);
    return f;
}
