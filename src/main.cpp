/* Required for Accelerometer Code*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
/* Required for BLE App Integration */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* ----- Sensor / Alert Config ----- */
#define LOOP_DELAY_MS 100
#define ALERT_FREQ_HZ 2000
#define ALERT_DURATION_MS 200

const float TICK_RATE = LOOP_DELAY_MS / 1000.0f; // Seconds per loop
const float IMPACT_THRESHOLD = 75.0f; // m/s^3
const float HIGH_TEMP_F = 140.0f;

/* ----- BLE Config ----- */
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define SENSOR_CHAR_UUID "12345678-1234-5678-1234-56789abcdef1"
#define IMPACT_CHAR_UUID "12345678-1234-5678-1234-56789abcdef2"

/* ----- Globals ------ */
Adafruit_MPU6050 mpu;

bool firstReading = true;
float prevX = 0.0f, prevY = 0.0f, prevZ = 0.0f;

BLECharacteristic* sensorChar = nullptr;
BLECharacteristic* impactChar = nullptr;
bool deviceConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    Serial.println("BLE: Client Connected");
  }
  void onDisconnect(BLEServer* server) override {
    deviceConnected = false;
    Serial.println("BLE: Client Disconnected, Re-Advertising");
    server->getAdvertising()->start();
  }
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("Booting SafetyVestinator...");

  // I2C + MPU6050
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check Wiring.");
    while (true) {
      delay(1000);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 Ready!");

  // BLE
  BLEDevice::init("SafetyVestinator");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  sensorChar = service->createCharacteristic(
    SENSOR_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);
  sensorChar->addDescriptor(new BLE2902());

  impactChar = service->createCharacteristic(
    IMPACT_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);
  impactChar->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* advertising = server->getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();
  Serial.println("BLE Advertising Started!");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  bool impactDetected = false;
  if (!firstReading) {
    float jerkX = fabsf(ax - prevX) / TICK_RATE;
    float jerkY = fabsf(ay - prevY) / TICK_RATE;
    float jerkZ = fabsf(az - prevZ) / TICK_RATE;

    Serial.print("Jerk X: " + String(jerkX)); 
    Serial.print(", Y: " + String(jerkY));    
    Serial.print(", Z: " + String(jerkZ));    
    Serial.println(" m/s^3");

    if (jerkX > IMPACT_THRESHOLD ||
        jerkY > IMPACT_THRESHOLD ||
        jerkZ > IMPACT_THRESHOLD) {
          Serial.println(" IMPACT DETECTED ");
    }
  }

  prevX = ax;
  prevY = ay;
  prevZ = az;
  firstReading = false;

  Serial.print("Acceleration X: " + String(ax));
  Serial.print(", Y: " + String(ay));
  Serial.print(", Z: " + String (az));
  Serial.println(" m/s^2");

  Serial.print("Rotation X: " + String(g.gyro.x));
  Serial.print(", Y: " + String(g.gyro.y));
  Serial.print(", Z: " + String(g.gyro.z));
  Serial.println(" rad/s");

  float tempF = (temp.temperature * 9.0f / 5.0f) + 32.0f;

  Serial.print("Temperature: " + String(tempF));
  Serial.println(" degF");

  if (tempF > HIGH_TEMP_F) {
    Serial.println(" HIGH TEMPERATURE WARNING ");
  }

  // BLE Transmission
  if (deviceConnected) {
    float payload[7] = {
      ax, ay, az,
      g.gyro.x, g.gyro.y, g.gyro.z,
      tempF
    };
    sensorChar->setValue((uint8_t*)payload, sizeof(payload));
    sensorChar->notify();

    if (impactDetected) {
      uint8_t flag = 1;
      impactChar->setValue(&flag, 1);
      impactChar->notify();
    }
  }

  Serial.println("");
  delay(LOOP_DELAY_MS);
}
