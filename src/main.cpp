#include "main.h"

#include <Arduino.h>
#include <SD.h>

// Pins
const uint8_t kPinCapacitor = A7;           // Pin measuring capacitor voltage
const uint8_t kPinChargeDischarge = 2;      // Pin controlling relay connecting Cap to bat or coils
const uint8_t kPinCoilSwitch = 6;           // Pin controlling relay connecting coil 1 or 2
const uint8_t kPinSensingCoil = A5;         // Pin measuring sensing coil voltage

// Settings
const float kCountsToVolts = 0.00488;       // Factor translating ADC counts to Volts; Vref/1024
const float kVmax = 2.3;                    // Maximum capacitor voltage
const uint16_t kChargingInterval = 5000;    // Charge for [ms] before measuring voltage
const uint16_t kDischargeCycleDelay = 100;  // Wait for [ms] after a measurement cycle
const uint16_t kDischargeTime = 1000;       // Discharge for a total of [ms]
const uint16_t kMaxChargeCycles = 10000;    // Max number of charging cycles before giving up
const uint16_t kResultsArrayLength = 30;    // Number of measurements
const uint16_t kSensorDelay = 1200;         // time to wait before measuring sensor volts; [us]
const uint16_t kSensorInterval = 1;         // Interval between measurements; [us]
const uint16_t kTransientDelay = 10;        // Amount of [ms] to wait for transients to subside

// change this to match your SD shield or module;
//     Arduino Ethernet shield: pin 4
//     Adafruit SD shields and modules: pin 10
//     Sparkfun SD shield: pin 8
const uint8_t chipSelect = 10;

bool coil1_active = true;                   // Specifies which coil should be used for discharge
File dataFile;

void setup() {
    Serial.begin(9600);

    Serial.print("Initializing SD card...\n");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(SS, OUTPUT);

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
    }
    Serial.println("Card initialized");

    pinMode(kPinChargeDischarge, OUTPUT);
    pinMode(kPinCoilSwitch, OUTPUT);
    pinMode(kPinCapacitor, INPUT);
    pinMode(kPinSensingCoil, INPUT);
}

void loop() {
    ChargeCapacitor(kVmax, kChargingInterval);
    float* results = Measure(coil1_active, kSensorDelay, kSensorInterval, kDischargeTime);
    // WriteResultsToSD(results);
    delay(kDischargeCycleDelay);
    delete[] results;
    coil1_active = !coil1_active;
}

/**
 * Charge capacitor until it exceeds vmin
 *
 * @param vmin Capacitor must exceed this voltage to stop charging.
 * @param interval Charge capacitor for [ms] before measuring again.
 * @return float Final capacitor voltage.
 */
float ChargeCapacitor(const float vmin, const unsigned int interval) {
    float cap_volts = 0;
    Serial.print("Charging capacitor\n");
    for (uint16_t i = 0; i < kMaxChargeCycles; i++) {
        cap_volts = analogRead(kPinCapacitor) * kCountsToVolts;

        Serial.print("Cycle ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(cap_volts);
        Serial.print(" V\n");

        if (cap_volts > vmin) {
            Serial.print("Capacitor charged; final voltage: ");
            Serial.print(cap_volts);
            Serial.print(" V\n");
            return cap_volts;
        }

        delay(interval);
    }
    Serial.print("kMaxChargeCycles reached! Aborting charging at ");
    Serial.print(cap_volts);
    Serial.print(" V");
    return cap_volts;
}

/**
 * Discharge capacitor into a coil and measure voltage on sensing coil
 *
 * @param coil1_active Whether to energize coil 1.
 * @param initial_delay Wait for [us] before starting measurements.
 * @param interval Interval [us] between measurements.
 * @param total_time Total discharge time.
 * @return float* An array of the measured voltages.
 */
float* Measure(const bool coil1_active, const unsigned int initial_delay,
               const unsigned int interval, const unsigned int total_time) {
    float* results = new float[kResultsArrayLength];

    Serial.print("Preparing to measure\n");
    if (coil1_active == 1) {
        Serial.print("Coil 1 selected\n");
    } else {
        digitalWrite(kPinCoilSwitch, HIGH);
        Serial.print("Coil 2 selected\n");
    }
    delay(kTransientDelay);

    Serial.print("Discharging capacitor\n");
    digitalWrite(kPinChargeDischarge, HIGH);
    delayMicroseconds(initial_delay);

    unsigned long tstart = millis();
    for (uint16_t j = 0; j < kResultsArrayLength; j++) {
        results[j] = analogRead(kPinSensingCoil) * kCountsToVolts;
        delayMicroseconds(interval);
    }
    unsigned long tcheck = millis() - tstart;

    Serial.print("Measurement finished, discharging up to specified time\n");
    // finish up the discharge (to make sure all the ferrofluid is on one side?)
    delay(kDischargeTime - tcheck);
    digitalWrite(kPinChargeDischarge, LOW);
    digitalWrite(kPinCoilSwitch, LOW);  // always turn off relays to reduce current (56mA)

    Serial.print("Results: ");
    for (uint16_t j = 0; j < kResultsArrayLength; j++) {
        Serial.print(results[j], 4);
        Serial.print(" ");
    }
    Serial.print("\n");

    File dataFile = SD.open("data.csv", FILE_WRITE);
    if (dataFile) {
        dataFile.print(millis());
        for (uint16_t j = 0; j < kResultsArrayLength; j++) {
            dataFile.print(results[j], 4);
            if (j < kResultsArrayLength - 1) {
                dataFile.print(",");
            }
        }
        dataFile.print("\n");
        dataFile.close();
        Serial.print("Data written to SD card.\n");
    } else {
        Serial.print("Error opening file!\n");
    }

    return results;
}
