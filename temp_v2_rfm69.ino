/*
 * 
 * Mysensors Temperature Sensor with battery monitoring
 * 
 */

// Enable debug prints to serial monitor
//#define MY_DEBUG 
#define MY_SPLASH_SCREEN_DISABLED

// Enable and select radio type attached
//#define MY_RADIO_NRF24
#define MY_RADIO_RFM69
#define MY_IS_RFM69HW
#define MY_REPEATER_FEATURE
#define MY_NODE_ID 2

#include <SPI.h>
#include <MySensors.h> 
#include <DallasTemperature.h> 
#include <OneWire.h>
#include <avr/wdt.h>
#include <TimeLib.h> 
#include <Wire.h>
#include <Adafruit_INA219.h>

//extern RFM69 _radio;

#define BVOLT_ID 0
#define SVOLT_ID 1
#define TEMP_ID 2
#define RADIOTEMP_ID 3
#define RSSI_ID 4
#define CURR_ID 5

#define LED_POWERUP_COUNT 6
#define LED_DELAY 200

//Sketch information
#define SKETCH_INFO       "Solar and Temperature Sensor"
#define SKETCH_VERSION    "3.0"
#define TEMP_ID_INFO      "Sensor Temperature"
#define TEMP2_ID_INFO     "Sensor2 Temperature"
#define BVOLT_ID_INFO     "Battery Voltage"
#define SVOLT_ID_INFO     "Solar Voltage"
#define RADIOTEMP_ID_INFO "Radio Temperature"
#define RSSI_ID_INFO      "Radio RSSI"
#define CURR_ID_INFO      "Power Supply Current"

//DS18B20 configuration
#define ONE_WIRE_BUS 4 // Pin where dallas sensor is connected 
#define ONE_WIRE_VCC 5 // Pin where the one wire power is controlled
#define DELTA_TEMP 0 // define temperature delta to report to controller (a value of 0 sends everytime)
#define TEMP_RESOLUTION 11 //Temperature resolution


//for 1xli-ion
#define VMIN 3.5
#define VMAX 4.1  
#define VDELTA 0.6
#define BATT_CALC 0.0061070412
#define SOLAR_CALC 0.0117397661
#define CURR_CALC 17.419763513513513513513513513514
#define CURR_ZERO 506

/*
//for 2xnimh
#define VMIN 2.0
#define VMAX 2.8  
#define VDELTA 0.8
#define BATT_CALC 0.003037109
*/
#define BATTERY_READS 10
#define DELTA_VBATT 0
#define BATTERY_SENSE A3 
#define SOLAR_SENSE A0
#define PULSEPIN A2

//Auto-reset
#define MESSAGES_FAILED_REBOOT 20
#define CICLES_REBOOT 2880

//RSSI calculations
#define RSSI_READS 3

//Configurable ACK Timeout
#define ACK_TIMEOUT 4000

//BAUD RATE
#define BAUD_RATE 115200

//Cycles in between updates
#define SECONDS_PER_CICLE 150
#define CICLES_PER_PRESENT 2880
#define CICLES_PER_UPDATE 1
#define CICLES_PER_TIMEUPDATE 240

//Transmit Retries
#define HIGH_PRIORITY_RETRIES 10
#define LOW_PRIORITY_RETRIES 2


float lastTemperature=-127,temperature=-127,deltatemp,radioTemperature;
float Sbatt, Vbatt, deltavbatt, current;
unsigned int BattValue, Batt, Battarray[BATTERY_READS], Battindex = 0, BattarrayTotal = 0;
unsigned int Currarray[BATTERY_READS], Currindex = 0, CurrarrayTotal = 0;
int radioRSSIarray[RSSI_READS], radioRSSIindex=0,radioRSSIarrayTotal=0,messagesFailed=0;
volatile int radioRSSI, isACKed = false;
unsigned int nosend = CICLES_PER_UPDATE, i;
unsigned int topresent = CICLES_PER_PRESENT;
boolean ack = true;
boolean metric, temperatureError = false, timeReceived = false; 

unsigned long tempwakeupTime = 1000;
unsigned long sensorreadTime = 900;
unsigned long sleep_time = (SECONDS_PER_CICLE - tempwakeupTime/1000 - sensorreadTime*2.0/1000); // Sleep time between reads (in milliseconds)
unsigned long cicles=0;

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature. 
Adafruit_INA219 ina219; //INA219 variable

//Mysensors Messages initialization
MyMessage msgB(BVOLT_ID,V_VOLTAGE);
MyMessage msgS(SVOLT_ID,V_VOLTAGE);
MyMessage msgT(TEMP_ID,V_TEMP);
MyMessage msgR(RADIOTEMP_ID,V_TEMP);
MyMessage msgQ(RSSI_ID,V_VOLTAGE);
MyMessage msgI(CURR_ID,V_CURRENT);
MyMessage message;

void before()  
{
  //disable watchdog timer
  MCUSR = 0;
  pinMode(PULSEPIN, INPUT);
  Serial.begin(BAUD_RATE);
  Serial.println(F("begin"));

  //Setup power level
  //_radio.setPowerLevel(30);
  
  //Setup pin modes for proper operation
  pinMode(ONE_WIRE_VCC, OUTPUT);
  wake_sensors ();
  //blink LED on power up
  pinMode(13,OUTPUT);
  for (i = 0 ; i<LED_POWERUP_COUNT ;i++) {
    Serial.print(".");
    digitalWrite (13, HIGH);
    delay(LED_DELAY);
    digitalWrite (13, LOW);
    delay(LED_DELAY);
    delay(LED_DELAY);
  }

  //Initialing INA219
  Serial.println(F("Initialing INA219"));
  ina219.begin();
  ina219.setCalibration_32V_1A(); //double sensitivity (normal 2A)
  //ina219.setCalibration_16V_400mA(); //even more sensitivity (too high for charging current Li-ion)

  Serial.println(F("initialing battery voltage"));
  analogReference(INTERNAL);
  analogRead(BATTERY_SENSE);
  for (i = 0; i < BATTERY_READS; i++) {
    analogRead(BATTERY_SENSE);
    Battarray[i] = analogRead(BATTERY_SENSE);
    BattarrayTotal += Battarray[i];
    Serial.print(" ");
    Serial.print((float)Battarray[i] * BATT_CALC);
  }
  Serial.println(F(""));
  Serial.print(F("Battery average"));
  Serial.print((float)BattarrayTotal/BATTERY_READS * BATT_CALC);
  Serial.println(F(""));

  CurrarrayTotal=0;
  Serial.println(F("initialing Current"));
  for (i = 0; i < BATTERY_READS; i++) {
    Currarray[i] = ina219.getCurrent_mA();
    CurrarrayTotal += Currarray[i];
    Serial.print(" ");
    Serial.print(Currarray[i]);
  }
  Serial.println(F(""));
  Serial.print(F("Current average"));
  Serial.print(CurrarrayTotal/BATTERY_READS);
  Serial.println(F(""));
  analogReference(INTERNAL);

  Serial.println(F("initialing RSSI array"));
  for (i = 0; i < RSSI_READS; i++) {
    radioRSSIarray[i] = 0;
  }
  start_sensors();
}

void setup()  
{ 
  Serial.println("");
  //activate watchdog timer
  wdt_enable(WDTO_8S);
  pinMode (BATTERY_SENSE,INPUT);
  pinMode (SOLAR_SENSE,INPUT);
  
//  metric = getConfig().isMetric;
  metric = true;
  Serial.print(F("Metric: "));
  Serial.println(metric);

  i=0;
  while (timeReceived != true) {
    Serial.println(F("Requesting Time"));
    //request time
    requestTime(); 
    heartbeat();
    wait(5000);
    if (i>4) {
      break;
    } else {
      i++;
    }
  }  
  PrintTime ();
} 

void presentation ()
{
  gwPresent ();
}

void loop() 
{
  heartbeat();
  //check if it should reboot itself
  if ((hour() == 2) && (minute() == 15 ) && (cicles > 100)) {
    Serial.println(F("Reboot time reached (2:15)"));
    asm volatile ( "jmp 0");
    wait(100);
  }

  if ((minute() == 20) && (second() <= 30)) {
    timeReceived = false;
    i=0;
    while (timeReceived != true) {
      //request time
      requestTime(); 
      heartbeat();
      wait(2000);
      PrintTime ();
      if (i>4) {
        break;
      } else {
        i++;
      }
    }
  }  
  
  readTemp ();
  readSolar();  
  readBattery();
  readRadioTemp();
  readCurrent();
  sendValues();

  cicles++;
  Serial.print(F("Waiting in cicle "));
  //displays the current cicle count
  Serial.println(cicles);
  wdsleep(sleep_time*1000);
} 

void wake_sensors () {
  //Wake the temperature sensor
  Serial.println(F("waking sensor"));
  digitalWrite (ONE_WIRE_VCC, HIGH);
}

void sleep_sensors () {
  //Sleep the temperature sensor
  Serial.println(F("Sleeping sensor"));
  digitalWrite (ONE_WIRE_VCC, HIGH);
}

void start_sensors () {
  Serial.println(F("Configuring Temperature Sensor"));
  // Startup up the OneWire library
  sensors.begin();
  sensors.setWaitForConversion(false);

  //initialize temperature resolution
  if (sensors.getResolution() != TEMP_RESOLUTION) {
    Serial.println(F("New temperature resolution configured"));
    sensors.setResolution(TEMP_RESOLUTION);
  } else {
    Serial.println(F("Temperature resolution not changed"));
  }
  sleep_sensors();
}

void readRadioTemp () {
  radioTemperature = _radio.readTemperature(0);
}

// Reads Current while averaging the last BATTERY_READS values
void readCurrent () {
  Serial.println(F("Reading Current"));
  //Reads current
  Currarray[Currindex] = ina219.getCurrent_mA();
  Serial.print(F("Current read "));
  Serial.println(Currarray[Currindex]);
  Serial.print(F("Index "));
  Serial.println(Currindex);
  //calculates the current average
  CurrarrayTotal=0;  
  for (i = 0; i < BATTERY_READS; i++) {
    CurrarrayTotal += Currarray[i];
  }
  //updates battery index
  Currindex ++;
  if (Currindex >= BATTERY_READS) {
    Currindex = 0;
  }

  current = CurrarrayTotal / BATTERY_READS;
  Serial.print(F("Current average "));
  Serial.println(current);
}

void readTemp () {
  // power up sensor
  wake_sensors ();
  wait(tempwakeupTime);

  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();
  wait(sensorreadTime);
  // Fetch
  temperature = ((metric ? sensors.getTempCByIndex(0) : sensors.getTempFByIndex(0)));
  //Fetch again
  sensors.requestTemperatures();
  wait(sensorreadTime);
  temperature = (temperature + (metric ? sensors.getTempCByIndex(0) : sensors.getTempFByIndex(0))) / 2.0;
  // disable sensor
  sleep_sensors();
  //print Temperature
  Serial.print(F("Metric:"));
  Serial.print(metric);
  Serial.print(F(" temperature: "));
  Serial.println(temperature, 3);

  // Only send data if no error
  if (temperature != -127.00 && temperature != 85.00) {
    deltatemp = temperature - lastTemperature;
    if (abs(deltatemp) < DELTA_TEMP) {
      //debug message
      Serial.println(F("Aproximatelly the same Temperature, skipping send"));
    } else {
      lastTemperature = temperature;
      nosend = CICLES_PER_UPDATE;
      temperatureError = false;
    }
  } else {
    Serial.println(F("Error reading temperature"));
    temperatureError = true;
    start_sensors();
  }
}

// Reads Battery voltage while averaging the last BATTERY_READS values
void readBattery () {
  
  Serial.println(F("Reading Battery Voltage"));
  //primes the analog converter
  analogRead(BATTERY_SENSE);
  analogRead(BATTERY_SENSE);
  analogRead(BATTERY_SENSE);
  //Reads battery voltage
  Battarray[Battindex] = analogRead(BATTERY_SENSE);
  Serial.print(F("Battery read "));
  Serial.println((float)Battarray[Battindex] * BATT_CALC);
  Serial.print(F("Index "));
  Serial.println(Battindex);
  //calculates the current average
  BattarrayTotal=0;  
  for (i = 0; i < BATTERY_READS; i++) {
    BattarrayTotal += Battarray[i];
  }
  //updates battery index
  Battindex ++;
  if (Battindex >= BATTERY_READS) {
    Battindex = 0;
  }

  BattValue = BattarrayTotal / BATTERY_READS;
  Serial.print(F("Battery analog average "));
  Serial.println(BattValue);
  Serial.print(F("Battery average "));
  Serial.println((float)BattValue * BATT_CALC);

  Vbatt = (float)BattValue * BATT_CALC;
  Batt = (Vbatt - VMIN) * 100.0 / (VDELTA);
  if (Vbatt > VMAX) {
    Batt = 100;
  } else if (Vbatt < VMIN) {
    Batt = 0;
  }

  deltavbatt = Vbatt - (BattValue * BATT_CALC);
  if (abs(deltavbatt) > DELTA_VBATT) {
    //force sending values
    nosend = CICLES_PER_UPDATE;
  }

  //print battery status
  Serial.print(F("battery: "));
  Serial.print(Batt);
  Serial.print(F(", "));
  Serial.println(Vbatt, 3);
}

void PrintTime () {
  //display current time
  Serial.print(F("Current date: "));
  Serial.print(dayStr(weekday()));
  Serial.print(F(" "));
  Serial.print(day());
  Serial.print(F("/"));
  Serial.print(monthStr(month()));
  Serial.print(F("/"));
  Serial.println(year()); 
  Serial.print(F("Current time: "));
  Serial.print(hour());
  Serial.print(F(":"));
  Serial.print(minute());
  Serial.print(F(":"));
  Serial.println(second());
}

// Reads Battery voltage while averaging the last BATTERY_READS values
void readSolar () {
  Serial.println(F("Reading Solar Voltage"));
  //primes the analog converter
  analogRead(SOLAR_SENSE);
  Serial.print(F("Solar analog "));
  Serial.println(BattValue);
  wait(5);
  analogRead(SOLAR_SENSE);
  analogRead(SOLAR_SENSE);
  //Reads battery voltage
  BattValue = analogRead(SOLAR_SENSE);
  Serial.print(F("Solar analog "));
  Serial.println(BattValue);
  //adds the current reading
  BattValue = (BattValue + analogRead(SOLAR_SENSE)) / 2;

  Serial.print(F("Solar analog Average "));
  Serial.println(BattValue);

  Sbatt = (float)BattValue * SOLAR_CALC;

  //print battery status
  Serial.print(F("Solar voltage: "));
  Serial.println(Sbatt, 3);
}

void sendValues () {
  //only sends values if they have changed or if it didn't send for 120 cycles (1 hour)
  if (nosend < CICLES_PER_UPDATE) {
    nosend++;
    return;
  }
  //reset count;
  nosend = 1;

  //print debug message
  Serial.println(F("Sending Values"));
  //send values
  if (temperatureError == false) {
    Serial.print(F("Temperature "));
    Serial.println(temperature,3);
    resend(msgT.set(temperature, 3),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
  }
  Serial.print(F("Battery Voltage "));
  Serial.println(Vbatt,3);
  resend(msgB.set(Vbatt, 3),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
  Serial.print(F("Solar Voltage "));
  Serial.println(Sbatt,3);
  resend(msgS.set(Sbatt, 3),HIGH_PRIORITY_RETRIES ,ACK_TIMEOUT);
  Serial.print(F("Radio Temperature "));
  Serial.println(radioTemperature,3);
  resend(msgR.set(radioTemperature, 0),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
  sendBatteryLevel(Batt, ack);
  Serial.print(F("Radio RSSI "));
  Serial.println(radioRSSI);
  resend(msgQ.set(radioRSSI),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
  Serial.print(F("Power Supply Current "));
  Serial.println(current,0);
  resend(msgI.set(current,0),LOW_PRIORITY_RETRIES ,ACK_TIMEOUT);
}

void gwPresent () {
  //present at beggining and every day
  if (topresent < CICLES_PER_PRESENT) {
    topresent++;
    return;
  }
  Serial.println(F("Presenting"));
  //reset count;
  topresent = 0;

  sendSketchInfo(SKETCH_INFO, SKETCH_VERSION);
  wait(1000);
  heartbeat();
  present(TEMP_ID, S_TEMP, TEMP_ID_INFO);
  wait(1000);
  heartbeat();
  present(RADIOTEMP_ID, S_TEMP, RADIOTEMP_ID_INFO);
  wait(1000);
  heartbeat();
  present(BVOLT_ID, S_MULTIMETER, BVOLT_ID_INFO);
  wait(1000);
  heartbeat();
  present(SVOLT_ID, S_MULTIMETER, SVOLT_ID_INFO);
  wait(1000);
  heartbeat();
  present(RSSI_ID, S_MULTIMETER, RSSI_ID_INFO);
  wait(1000);
  heartbeat();
  present(CURR_ID, S_MULTIMETER, CURR_ID_INFO);
  wait(1000);
  heartbeat();
}


void resend(MyMessage &msg, int repeats, int timeout)
{
  byte repeat = 0;
  byte repeatdelay = 0;
  boolean sendOK = false;

  while ((sendOK == false) and (repeat < repeats)) {
    send(msg,true);

    if (waitACK(timeout)) {
      sendOK = true;
      messagesFailed = 0;
    } else {
      sendOK = false;
      Serial.print("Retry ");
      Serial.print(repeat);
      Serial.print(" Failed ");
      Serial.println(messagesFailed);
      repeatdelay += 1;
      wdsleep(repeatdelay*500);
    }
    repeat++; 
  }
  if (sendOK == false) {
    if (messagesFailed > MESSAGES_FAILED_REBOOT) {
      asm volatile ( "jmp 0");
    }
    messagesFailed++;
  }
}


boolean waitACK (int timeout) {
  unsigned long startTime = millis();
  
  while ((millis() - startTime) < timeout) {
    wait(1);
    if (isACKed == true) {
      isACKed = false;
      Serial.print(F("Reply "));
      Serial.print(timeout);
      Serial.print(" ");
      Serial.println((millis() - startTime));
      return true;
    }
  }
  return false;
}

void receive (const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.

  //Substract the last reading
  radioRSSIarrayTotal = radioRSSIarrayTotal - radioRSSIarray[radioRSSIindex];
  //Replace last reading by current reading
  radioRSSIarray[radioRSSIindex] = _radio.RSSI;
  //Update last reading
  radioRSSIarrayTotal = radioRSSIarrayTotal + radioRSSIarray[radioRSSIindex];
  //Calculates new RSSI array index
  radioRSSIindex ++;
  if (radioRSSIindex >= RSSI_READS) {
    radioRSSIindex = 0;
  }
  //updates radio RSSI
  radioRSSI = radioRSSIarrayTotal / RSSI_READS;
  
  if (message.isAck()) {
    Serial.print(F("This is an ack from gateway. RSSI "));
    Serial.println(_radio.RSSI);
    isACKed = true;
  } else {
    Serial.print(F("Incoming change for sensor:"));
    Serial.print(message.sensor);
    Serial.print(F(" RSSI "));
    Serial.println(_radio.RSSI);
  }
}

void receiveTime(unsigned long time) {
  setTime(time);
  timeReceived = true;
}

void wdsleep(unsigned long ms) {
  unsigned long enter = hwMillis();
  #if defined(MY_REPEATER_FEATURE)
  while (hwMillis() - enter < ms) {
    wait(99);
    heartbeat();
  }
  #else
    sleep(ms);
  #endif
}

void heartbeat () {
  wdt_reset();
  pinMode(PULSEPIN, OUTPUT);
  digitalWrite(PULSEPIN, LOW);
  wait(1);
  // Return to high-Z
  pinMode(PULSEPIN, INPUT);
}

