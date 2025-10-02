#include "HardwareSerial.h"
#include "NimBLE2904.h"
#include "NimBLEAdvertisedDevice.h"
#include "NimBLEAdvertisementData.h"
#include "NimBLEAdvertising.h"
#include "NimBLECharacteristic.h"
#include "NimBLEClient.h"
#include "NimBLEConnInfo.h"
#include "NimBLEDescriptor.h"
#include "NimBLELocalValueAttribute.h"
#include "NimBLERemoteCharacteristic.h"
#include "NimBLERemoteService.h"
#include "NimBLEServer.h"
#include "NimBLEService.h"
#include "NimBLEUUID.h"
#include "WString.h"
#include "esp32-hal-ledc.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <cstdint>
#include <PID_v1.h>
#include <Preferences.h>

#define T_SERVICE "00A10074-6865-726D-6F77-6F726B730D0A"
#define T_CHAR "00A11274-6865-726D-6F77-6F726B730D0A"

#define S_SERVICE "8a07b500-922f-4ef6-9a04-4d80509cd01e"
static uint32_t scanTimeMS = 5000;

NimBLEClient *client;

NimBLEServer *server;

NimBLEService *servService;

NimBLECharacteristic *tChar;
NimBLECharacteristic *pChar;
NimBLECharacteristic *iChar;
NimBLECharacteristic *dChar;
NimBLECharacteristic *fanMaxChar;
NimBLECharacteristic *fanPuffChar;
NimBLECharacteristic *fanRestChar;
//NimBLECharacteristic *testOutput;


//Define Variables we'll be connecting to
double setpoint, input, output;

//Define the aggressive and conservative Tuning Parameters
double consKp=0.0, consKi=0.0, consKd=0.0, maxFan=0.0;

double fanPuffDuration = 6.0;
double fanRestDuration = 20.0;

//Specify the links and initial tuning parameters
PID tempPID(&input, &output, &setpoint, consKp, consKi, consKd, P_ON_E, DIRECT);


uint32_t fanOutput = 0;
uint32_t flapOutput = 0;


Preferences settings;


enum FanState { FAN_IDLE, FAN_PUFFING, FAN_RESTING };
FanState currentFanState = FAN_IDLE;
unsigned long fanCycleStartTime = 0;
uint32_t fanSpeedForPuff = 0;


// Flag to trigger PID computation
volatile bool newTempData = false;


uint32_t msToDuty(uint32_t pulseWidth) {
  const double PWM_PERIOD_US = 1000000.0f / 333;
  const double MAX_DUTY_CYCLE = 1023.0f;
  double dutyRatio = static_cast<double>(pulseWidth) / PWM_PERIOD_US;
  return static_cast<uint32_t>(dutyRatio * MAX_DUTY_CYCLE);
}


void startAdvertising() {
  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData;
  advData.setName("SmokerCTRL");
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.addServiceUUID(servService->getUUID());
  advertising->setAdvertisementData(advData);
  advertising->setScanResponseData(advData);
  advertising->enableScanResponse(true);
  advertising->start();
}


class ServerCallbacks : public NimBLEServerCallbacks {

  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
      Serial.println("Client connected");
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
      Serial.println("Client disconnected, restarting advertising...");
      startAdvertising();
  }
};

void temperatureNotify(NimBLERemoteCharacteristic* pCharacteristic, unsigned char *arg, unsigned long warg, bool varg) {
  String temp_str = pCharacteristic->getValue();
  float temp = temp_str.toFloat();
  temp -= 32;
  temp /= 1.8f;
  
  input = temp;

  newTempData = true;
}

void setup()
{
  Serial.begin(9600);

  settings.begin("SmokerCtrl");

  //FAN
  ledcAttachPin(25, 0);
  ledcSetup(0, 6000, 10);
  ledcWrite(0, 1023);

  //SERVO
  ledcAttachPin(33, 1);
  ledcSetup(1, 333, 10);
  ledcWrite(1, msToDuty(760));

  consKp = settings.getString("consKp", "0.0").toDouble();
  consKi = settings.getString("consKi", "0.0").toDouble();
  consKd = settings.getString("consKd", "0.0").toDouble();
  setpoint = settings.getString("target", "0.0").toDouble();
  maxFan = settings.getString("fanMax", "0.0").toDouble();
  fanPuffDuration = settings.getString("fanPuff", "6.0").toDouble();
  fanRestDuration = settings.getString("fanRest", "20.0").toDouble();

  

  Serial.printf("Starting NimBLE Client\n");

  /** Initialize NimBLE and set the device name */
  NimBLEDevice::init("SmokerCTRL");

  NimBLEDevice::setPower(3);

  server = NimBLEDevice::createServer();
  client = NimBLEDevice::createClient();


  servService = server->createService(S_SERVICE);

  tChar = servService->createCharacteristic("8a07b501-922f-4ef6-9a04-4d80509cd01e", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  NimBLE2904* tType = tChar->create2904();
  NimBLEDescriptor* tName = tChar->createDescriptor(static_cast<uint16_t>(0x2901));
  tType->setFormat(NimBLE2904::FORMAT_UTF8);
  tName->setValue("Target Temp");

  tChar->setValue(settings.getString("target", "0.0"));

  pChar = servService->createCharacteristic("8a07b502-922f-4ef6-9a04-4d80509cd01e", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  NimBLE2904* pType = pChar->create2904();
  NimBLEDescriptor* pName = pChar->createDescriptor(static_cast<uint16_t>(0x2901));
  pType->setFormat(NimBLE2904::FORMAT_UTF8);
  pName->setValue("Proportional");
  
  pChar->setValue(settings.getString("consKp", "0.0"));

  iChar = servService->createCharacteristic("8a07b503-922f-4ef6-9a04-4d80509cd01e", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  NimBLE2904* iType = iChar->create2904();
  NimBLEDescriptor* iName = iChar->createDescriptor(static_cast<uint16_t>(0x2901));
  iType->setFormat(NimBLE2904::FORMAT_UTF8);
  iName->setValue("Integral");

  iChar->setValue(settings.getString("consKi", "0.0"));

  dChar = servService->createCharacteristic("8a07b504-922f-4ef6-9a04-4d80509cd01e", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  NimBLE2904* dType = dChar->create2904();
  NimBLEDescriptor* dName = dChar->createDescriptor(static_cast<uint16_t>(0x2901));
  dType->setFormat(NimBLE2904::FORMAT_UTF8);
  dName->setValue("Derivative");

  dChar->setValue(settings.getString("consKd", "0.0"));

  fanMaxChar = servService->createCharacteristic("8a07b505-922f-4ef6-9a04-4d80509cd01e", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  NimBLE2904* fanMaxType = fanMaxChar->create2904();
  NimBLEDescriptor* fanMaxName = fanMaxChar->createDescriptor(static_cast<uint16_t>(0x2901));
  fanMaxType->setFormat(NimBLE2904::FORMAT_UTF8);
  fanMaxName->setValue("Max Fan");

  fanMaxChar->setValue(settings.getString("fanMax", "100.0"));

  fanPuffChar = servService->createCharacteristic("8a07b506-922f-4ef6-9a04-4d80509cd01e", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  NimBLE2904* fanPuffType = fanPuffChar->create2904();
  NimBLEDescriptor* fanPuffName = fanPuffChar->createDescriptor(static_cast<uint16_t>(0x2901));
  fanPuffType->setFormat(NimBLE2904::FORMAT_UTF8);
  fanPuffName->setValue("Fan Puff Duration");
  
  fanPuffChar->setValue(settings.getString("fanPuff", "6.0"));

  fanRestChar = servService->createCharacteristic("8a07b507-922f-4ef6-9a04-4d80509cd01e", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  NimBLE2904* fanRestType = fanRestChar->create2904();
  NimBLEDescriptor* fanRestName = fanRestChar->createDescriptor(static_cast<uint16_t>(0x2901));
  fanRestType->setFormat(NimBLE2904::FORMAT_UTF8);
  fanRestName->setValue("Fan Rest Duration");

  fanRestChar->setValue(settings.getString("fanRest", "20.0"));

  
  servService->start();

  startAdvertising();

  NimBLEDevice::getServer()->setCallbacks(new ServerCallbacks());


  NimBLEScan* pScan = NimBLEDevice::getScan();


  Serial.printf("Starting Scan\n");


  uint8_t foundSuitable = 0;

  while (foundSuitable == 0) {
    NimBLEScanResults results = pScan->getResults(10 * 1000);
    Serial.printf("Found Devices: %u\n", results.getCount());
    for (int i = 0; i < results.getCount(); i++) {
      const NimBLEAdvertisedDevice *device = results.getDevice(i);

      Serial.printf("Device: \"%s\" ID: %s ServiceCount: %u Scanable: %u ", device->getName().c_str(), device->getAddress().toString().c_str(), device->getServiceUUIDCount(), device->isScannable());

      if (device->getAddress().toString().find("a0:dd:6c:0c:a5:3e") != std::string::npos) {
        Serial.printf("Suitable");
        if (client->connect(device)) {
          foundSuitable = 1;
          Serial.printf("\nConnected\n");
          if (client->secureConnection()) {
            Serial.printf("Secured\n");
          }
          break;
        };
      }
      Serial.printf("\n");
    }
  }

  NimBLERemoteService *service = client->getService(NimBLEUUID(T_SERVICE));

  NimBLERemoteCharacteristic *characteristic = service->getCharacteristic(NimBLEUUID(T_CHAR));

  if (characteristic->canNotify()) {
    characteristic->subscribe(true, temperatureNotify);
  }

  tempPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(0, 511);
  tempPID.SetTunings(consKp, consKi, consKd);
}

void loop()
{
  String tempTar = tChar->getValue();
  String pNew = pChar->getValue();
  String iNew = iChar->getValue();
  String dNew = dChar->getValue();
  String maxFanNew = fanMaxChar->getValue();
  String fanPuffNew = fanPuffChar->getValue();
  String fanRestNew = fanRestChar->getValue();


  if (abs(tempTar.toDouble() - setpoint) > 0.01
    || abs(pNew.toDouble() - consKp) > 0.001
    || abs(iNew.toDouble() - consKi) > 0.0001
    || abs(dNew.toDouble() - consKd) > 0.001
    || abs(maxFanNew.toDouble() - maxFan) > 0.001
    || abs(fanPuffNew.toDouble() - fanPuffDuration) > 0.01
    || abs(fanRestNew.toDouble() - fanRestDuration) > 0.01) 
  {

    Serial.printf("Saving Params T:%s P:%s I:%s D:%s F:%s\n", tempTar.c_str(), pNew.c_str(), iNew.c_str(), dNew.c_str(), maxFanNew.c_str());
    settings.putString("target", tempTar);
    settings.putString("consKp", pNew);
    settings.putString("consKi", iNew);
    settings.putString("consKd", dNew);
    settings.putString("fanMax", maxFanNew);
    settings.putString("fanPuff", fanPuffNew);
    settings.putString("fanRest", fanRestNew);

    consKp = pNew.toDouble();
    consKi = iNew.toDouble();
    consKd = dNew.toDouble();
    setpoint = tempTar.toDouble();
    maxFan = maxFanNew.toDouble();
    fanPuffDuration = fanPuffNew.toDouble();
    fanRestDuration = fanRestNew.toDouble();

    tempPID.SetTunings(consKp, consKi, consKd);
  }


  if (newTempData) {
    tempPID.Compute();
    Serial.printf("Temp: %f/%f, Output: %f\n", input, setpoint, output);
    newTempData = false;
  }

  // map flap servo to 0 to 256 of output. flap is 1300 open and 2015 closed
  flapOutput = 2015 - ((constrain(output, 0.0, 256.0)/256)*715);
  ledcWrite(1, msToDuty(flapOutput));



  // --- Fan State Machine ---
  
  const double FAN_THRESHOLD = 255.0;
  unsigned long currentTime = millis();

  switch(currentFanState) {
    case FAN_IDLE:
      // If the fan is idle, check if PID output is high enough to start a puff cycle.
      if (output > FAN_THRESHOLD) {
        Serial.println("Demand high, starting fan puff cycle.");
        currentFanState = FAN_PUFFING;
        fanCycleStartTime = currentTime;

        // Calculate fan speed for this puff based on PID output
        double fan_proportion = (output - FAN_THRESHOLD) / (511.0 - FAN_THRESHOLD);
        uint32_t maxFanDuty = 1023 * (maxFan / 100.0);
        double min_fan_duty = 0.15 * maxFanDuty;
        double fan_duty_range = maxFanDuty - min_fan_duty;
        fanSpeedForPuff = min_fan_duty + (fan_proportion * fan_duty_range);

        ledcWrite(0, 1023 - constrain(fanSpeedForPuff, 0, maxFanDuty)); // Turn fan ON (active low)
      }
      break;

    case FAN_PUFFING:
      // The fan is on. Check if it's time to turn it off and start resting.
      if (currentTime - fanCycleStartTime >= fanPuffDuration*1000) {
        Serial.println("Puff complete. Starting rest period.");
        currentFanState = FAN_RESTING;
        ledcWrite(0, 1023); // Turn fan OFF
      }
      break;

    case FAN_RESTING:
      // The fan is off. Check if the rest period is over.
      if (currentTime - fanCycleStartTime >= ((fanPuffDuration*1000) + (fanRestDuration*1000))) {
        Serial.println("Rest complete. Fan is idle.");
        currentFanState = FAN_IDLE; // Cycle is finished, ready for the next one.
      }
      break;
  }

  // Safety Override: If temperature overshoots and PID output drops, immediately stop the fan.
  if (output < FAN_THRESHOLD && currentFanState != FAN_IDLE) {
    Serial.println("PID demand dropped, stopping fan cycle.");
    currentFanState = FAN_IDLE;
    ledcWrite(0, 1023); // Turn fan OFF
  }
  



   delay(100);
        
}
