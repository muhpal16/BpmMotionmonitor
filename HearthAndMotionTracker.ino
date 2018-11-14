#include "application.h"
#include "MAX30105.h"
#include "MPU6050.h"
#include "heartRate.h"
#include "MQTT.h"

MAX30105 particleSensor;
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time the last beat occurred

const unsigned long durationBetweenPublishes = 60 * 1000UL;
unsigned long lastPublished = 0;

float BPM;
int lookingAtScreen = 1;
int tiltingForward = 0;
int tiltingToTheSide = 0;
int tiltingBackward = 0;
int lookingRight = 0;
int lookingLeft = 0;
int beatAvg;
int avgBPMPublished;
long irValue;

const char* topic = "v1/devices/me/telemetry";

MQTT client("demo.thingsboard.io", 1883, callback);

void callback(char* topic, byte* payload, unsigned int length) {

  // Bruker denne for å sjekke mqtt forbindelsen.
  // bruke denne til å stille hvor ofte man skal få bpm målinger.
  char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;

    if (!strcmp(p, "RED"))
        RGB.color(0, 255, 0);
    else if (!strcmp(p, "GREEN"))
        RGB.color(255, 0, 0);
    else if (!strcmp(p, "BLUE"))
        RGB.color(0, 0, 255);
    else
        RGB.color(255, 255, 255);
    delay(1000);
}


void setup()
{
  //RGB.control(true);
  Serial.begin(115200);
  Serial.println("Initializing...");

  client.connect("photon01", "8P317bHlPZuyEeBNX62s", NULL);


  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  particleSensor.setup(); //Configure sensor with default settings
  accelgyro.initialize();
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}



void loop()
{

  if (client.isConnected()){
        client.loop();
  }
  irValue = particleSensor.getIR();
  Serial.println(irValue); //Send raw data to plotter
  Serial.println();

  if (checkForBeat(irValue) == true)
  {
    //Sensing beat
    long delta = millis() - lastBeat;
    lastBeat = millis();
    unsigned long now = millis();

    BPM = 60 / (delta / 1000.0);

    if (BPM < 255 && BPM > 20 && irValue > 60000)
    {
      rates[rateSpot++] = (byte)BPM; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
        beatAvg /= RATE_SIZE;

      if(now - lastPublished >= durationBetweenPublishes) {
      //Take average of readings

      avgBPMPublished = beatAvg;
      char buf[16];

      sprintf(buf, "%d", avgBPMPublished);
      const char* avgBPMmqttPublished = buf;
      client.publish("outTopic/bpm",avgBPMmqttPublished);
      Particle.publish("avg BPM: ", String(avgBPMPublished));
      lastPublished = millis();
      }
    } else {
      if(now - lastPublished >= durationBetweenPublishes) {
        char buf[16];

        sprintf(buf, "%d", avgBPMPublished);
        const char* avgBPMmqttPublished = buf;
        //endre den under til no finger
        client.publish("outTopic/bpm",avgBPMmqttPublished);
        Particle.publish("avg BPM: ", String("No finger?"));
        lastPublished = millis();
      }
    }
  }
    // Serial.print("IR=");
    // Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(BPM);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
    Serial.print(", avgBPMPublished=");
    Serial.print(avgBPMPublished);

    if (irValue < 50000) {
      Serial.print(" No finger?");
      beatAvg = 0;
      BPM = 0;
    } else {
      /*
      Serial.print("IR= ");
      Serial.println(irValue);
      Serial.print("BPM= ");
      Serial.println(beatsPerMinute);
      Serial.print("Avg BPM= ");
      Serial.println(beatAvg);
      Serial.println();
      */
      ///////////////////// JSON PAYLOAD STRING
      String payload = "{";
      payload += "\"Looking_At_the_scree\":"; payload += String(lookingAtScreen);
      payload += ",";
      payload += "\"BPM\":"; payload += String(BPM);
      payload += ",";
      payload += "\"AVG_BPM\":"; payload += String(beatAvg);
    //  payload += ",";
    //  payload += "\"movement\":"; payload += "Sitting still!";
      payload += "}";
      //Serial.print(movement);
      //const char* buf = payload.c_str();
      char payload_buffer[100];
      payload.toCharArray(payload_buffer, 100);
      Serial.println(payload_buffer);
      client.publish(topic, payload_buffer);
    }

    Serial.println();

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    /*
    Serial.print("a/g: ax: \t");
    Serial.print(ax); Serial.print("AY: \t");
    Serial.print(ay); Serial.print("AZ: \t");
    Serial.print(az); Serial.print(" GX: \t");
    Serial.print(gx); Serial.print(" GY:\t");
    Serial.print(gy); Serial.print(" GZ: \t");
    Serial.println(gz);

    Serial.print("\t ay: ");Serial.print(ay);
    Serial.print("\t ax: ");Serial.print(ax);
    Serial.print("\t az: ");Serial.print(az);
    */
    Serial.print("Looking at screen: ");
    Serial.print(lookingAtScreen);
    Serial.print("    Looking at left: ");
    Serial.print(lookingLeft);
    Serial.print("    Looking at right: ");
    Serial.print(lookingRight);
    if(ay >= 2000){
      //digitalWrite(movement_tilt_front, HIGH);
      tiltingBackward = 1;
      Particle.publish("Leaning backwards! Not looking at the screen");
      lookingAtScreen = 0;
      client.publish("outTopic/Motion","Leaning forward");
      delay(100);
    }

     else if(ay <= -9000){
      //digitalWrite(movement_tilt_back, HIGH);
      tiltingForward = 1;
      lookingAtScreen = 0;
      Particle.publish("Leaning forward! Not looking at the screen");
      client.publish("outTopic/Motion","Leaning backwards");
      delay(100);
    } else if(gz >= 5000){
      // Particle.publish("Head rotation: Left!");
       lookingLeft = lookingLeft + 1;
       lookingRight = lookingRight - 1;
       delay(500);
       lookingAtScreen = 0;
       if(lookingLeft == 1 && lookingRight == -1) {
         lookingAtScreen = 0;
         Particle.publish("Looking to the left! Not looking at the screen");
         delay(200);
      }
      // client.subscribe("inTopic/message");

     }
     else if(gz <= -5000){
       lookingRight = lookingRight + 1;
       lookingLeft = lookingLeft - 1;
       delay(500);
       if(lookingRight == 1 && lookingLeft == -1) {
         lookingAtScreen = 0;
         Particle.publish("Looking to the right! Not looking at the screen");
         delay(200);
      }
    //   Particle.publish("Head rotation: Right!");
    //   client.publish("Head rotation:","Right!");
    //   client.subscribe("inTopic/message");


     }
     else if(ay <= 6000 && ay >= -8000){
       if(lookingRight == 0 && lookingLeft == 0) {
         lookingAtScreen = 1;
       }
      tiltingBackward = 0;
      tiltingToTheSide = 0;
      tiltingForward = 0;

    }


    /*else if(ax <= -6000){
      //digitalWrite(movement_tilt_left, HIGH);
      tiltingToTheSide = 1;
      Particle.publish("Leaning to the right!");
      client.publish("outTopic/Motion","Leaning left");
      delay(100);
    } else if(ax >= 6000){
      //digitalWrite(movement_tilt_left, HIGH);
      tiltingToTheSide = 1;
      Particle.publish("Leaning to the left!");
      client.publish("outTopic/Motion","Leaning left");
      delay(100);
    }

    */

}
