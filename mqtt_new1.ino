//INT D5
//SDA D2
//SCL D1
//VCC 3.3v
//GND GND

const char* ssid = "JioFi3_41F25E";
const char* password = "f4xy7s5rnx";
const char* mqtt_server = "broker.hivemq.com";//iot.eclipse.org


#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <stdlib.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define BUFFER_SIZE 100
#define INTERRUPT_PIN 14
#define USE_SERIAL Serial
#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15
const int MPU_addr = 0x68;                          // I2C address of the MPU-6050
int fsrAnalogPin = A0;
int fsrReading;
bool updateFirmware = 0;                            // status flag for new firmware update
const int motor1 = D1;
const int motor2 = D2;
const int motor3 = D4;

float prev = 0;
float current = 0;

// MPU control/status vars
bool dmpReady = false;                  // set true if DMP init was successful
uint8_t mpuIntStatus;                   // holds actual interrupt status byte from MPU
uint8_t devStatus;                      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                 // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
boolean massage = false;

Quaternion q;                                           // quaternion container
MPU6050 mpu;
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

boolean flag1 = false;
boolean flag2 = false;
boolean flag3 = false;

void dmpDataReady()
{
  mpuInterrupt = true;
}


// Update these with values suitable for your network.

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long alarmDifference1;
unsigned long alarmDifference2;
unsigned long alarmDifference3;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}




int forceSensor() {
  fsrReading = analogRead(fsrAnalogPin);
  return fsrReading;
}



void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  char arr[length];
  for (int i = 0; i < length; i++) {
      arr[i] = (char)payload[i];
  }
  if(String("alarm/massage").equals(topic)){
    Serial.println("Massage");
      if(arr[0] == '1'){
        massage = true;
        Serial.println("1");
//        digitalWrite(D0, HIGH);
//        Serial.print(digitalRead(D0)+"ada");
//    digitalWrite(D6, HIGH);  
//    digitalWrite(D7, HIGH);
      }
      if(arr[0] == '0'){
        massage = false;
//        digitalWrite(D0, LOW);
//    digitalWrite(D6, LOW);  
//    digitalWrite(D7, LOW);
      }
  }
  String p_payload;
  unsigned long totaldigit = 0;

  for (int i = 0; i < length; i++) {
    p_payload.concat((char)payload[i]);

  }
  uint8_t hourpub = p_payload.substring(0, 1).toInt();
  Serial.println(hourpub);
  uint8_t minpub = p_payload.substring(2, 3).toInt();
  Serial.println(minpub);
  unsigned long hourmilli = (unsigned long)hourpub * 60 * 60 * 1000;
  Serial.println(hourmilli);
  unsigned long minmilli = (unsigned long)minpub * 60 * 1000;
  Serial.println(minmilli);
  totaldigit = hourmilli + minmilli-50000;
  
  Serial.println(totaldigit);
  //  char arr[length];
  //  Serial.print("Message arrived [");
  //  Serial.print(topic);
  //  Serial.print("] ");
  //  long totaldigit = 0;
  //  unsigned int temp1 = length;
  //  for (int i = 0; i < length; i++) {
  //    arr[i] = (char)payload[i];
  //    Serial.print(arr[i]);
  //    Serial.println("Character or int");
  //    Serial.println(arr[i]);
  //    long temp = arr[i] - 48;
  //    Serial.println("Value of long");
  //    Serial.println(temp);
  //    totaldigit += temp * pow(10, temp1 - 1);
  //    temp1--;
  //    Serial.println("Total digit");
  //    Serial.println(totaldigit);
  //
  //  }
  alarmDifference1 = totaldigit + millis();
  Serial.println("MQTT data recieved");
  Serial.println("Current Millis:");
  Serial.println(millis());
  Serial.println("Alarm millis:");
  Serial.println(alarmDifference1);
  alarmDifference2 =  30000 + millis();
  alarmDifference3 = 15000 + millis();

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-13";
    //clientId += String(random(0xffff), HEX);
    clientId = "pillow";
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //      client.publish("outTopic", "hello world");
      // ... and resubscribe

      client.subscribe("last/tym");
      client.subscribe("alarm/massage");
      client.publish("full/tym","hello");
      //client.subscribe("presence");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(50);
    }
  }
}

void program()
{
  //client.loop();
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  //client.loop();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (indicates code efficiency should be improved to handle the overflow)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    //client.loop();
    // reset FIFO buffer
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    //client.loop();
    // wait for correct available data length
    while (fifoCount < packetSize)
    {
      fifoCount = mpu.getFIFOCount();
      //client.loop();
    }
    //client.loop();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    //client.loop();

    // track FIFO count here in case there is > 1 packet available
    fifoCount -= packetSize;

    //    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //    data = data + String(q.w, 4) + ";" + String(q.x, 4) + ";" + String(q.y, 4) + ";" + String(q.z, 4);
    //    Serial.println(data);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //client.loop();
    prev = current;
    //        Serial.print("ypr\t");
    //        Serial.print(ypr[0] * 180 / M_PI);
    //        Serial.print("\t");
    //        Serial.print(ypr[1] * 180 / M_PI);
    //        Serial.print("\t");
    //        Serial.println(ypr[2] * 180 / M_PI);
    current = ypr[1] * 180 / M_PI;
    char charVal[10];
    float floatVal = (current - prev) * 100;
    dtostrf(floatVal,4,4,charVal);
    client.publish("chart/topic",charVal);
    //client.loop();
     //Serial.println(charVal);
    //    client.publish("outTopic", (current - prev) * 100);  //CANNOT PUBLISH FLOAT
  }
  else
  {
    //Serial.println("Skipped");
  }
}

void initializeDMP(void) { //Begin I2C data transfer
  Wire.begin(4, 5);                 // SCL <==> D1 , SDA <==> D2
  Wire.setClock(400000);            // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);                 // PWR_MGMT_1 register
  Wire.write(0);                    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //Initialize DMP
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  delay(500);

  // Gyro offsets.As the sensor bby default will have some offset value
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0)
  {
    // turn on the DMP
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set DMP Ready flag so the main loop() function can use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Needed to avoid boot crashing.Should investigate further.Since its one time delay during setup won't affect the performance of the loop

}
void setup() {
  pinMode(D0, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.subscribe("last/tym");
  client.setCallback(callback);
  client.publish("full/tym","hello");
  client.subscribe("alarm/massage");
  initializeDMP();
}

void loop() {
  currentTime = millis();
  if (!client.connected() && !flag1 && !flag2) {
    reconnect();
  }
  client.loop();
  //Suscribe

  //Serial.println(alarmDifference - currentTime);

  //Set motor high
  int frsensor = forceSensor();
  Serial.println(frsensor);
  if ((alarmDifference1 - currentTime <= 5000 && frsensor > 50) || massage == true) {
    Serial.println("Alarm!");
    digitalWrite(D0, HIGH);
    delay(2000);
    digitalWrite(D6, HIGH);
    delay(2000);
    digitalWrite(D7, HIGH);
    frsensor = forceSensor();
    flag1 = true;
  }
  
//  if ((alarmDifference2 - currentTime <= 1000) && (frsensor > 3) && flag1) {
//    digitalWrite(D6, HIGH);
//    frsensor = forceSensor();
//    Serial.println("Alarm 2");
//    flag2 = true;
//  }
//
//  if ((alarmDifference3 - currentTime <= 1000) && (frsensor > 3) && flag2) {
//    digitalWrite(D7, HIGH);
//    Serial.println("Alarm 3");
//  }
  frsensor = forceSensor();
  if ((frsensor < 50) || massage == false) {
    digitalWrite(D0, LOW);
    digitalWrite(D6, LOW);
    digitalWrite(D7, LOW);
    flag1 = false;
    flag2 = false;
    flag3 = false;
  }
  program();

}


