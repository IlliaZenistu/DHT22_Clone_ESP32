/**
 * DHT22_Clone.h - DHT22 sensor library for ESP32 with clone/counterfeit sensor support
 * 
 * This library correctly reads negative temperatures from both original and
 * clone DHT22 sensors on ESP32/ESP32-S2/ESP32-S3/ESP32-C3.
 * 
 * Uses the ESP32 hardware RMT peripheral for reliable timing,
 * unaffected by WiFi/BLE interrupts.
 * 
 * Clone DHT22 sensors encode negative temperatures using two's complement,
 * while original sensors use a sign bit (bit 15). This library handles both.
 * 
 * Original: -2.5°C = 0x8019 (bit 15 = sign, bits 14-0 = 25)
 * Clone:    -2.5°C = 0xFFE7 (two's complement of -25)
 * 
 * https://github.com/YOUR_USERNAME/DHT22_Clone_ESP32
 * 
 * License: MIT
 */

#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

// Error codes
enum DHT22Clone_Error : uint8_t {
  DHT22_OK        = 0,  // Success
  DHT22_DRIVER    = 1,  // RMT driver error
  DHT22_TIMEOUT   = 2,  // No response from sensor
  DHT22_NACK      = 3,  // Invalid ACK pulse
  DHT22_BAD_DATA  = 4,  // Invalid pulse timing in data
  DHT22_CHECKSUM  = 5,  // Checksum mismatch
  DHT22_UNDERFLOW = 6,  // Too few bits received
  DHT22_OVERFLOW  = 7   // Too many bits received
};

// Sensor encoding type
enum DHT22Clone_Type : uint8_t {
  DHT22_AUTO     = 0,  // Auto-detect encoding (default, recommended)
  DHT22_ORIGINAL = 1,  // Force original encoding (sign bit)
  DHT22_CLONE    = 2   // Force clone encoding (two's complement)
};

struct DHT22Clone_Result {
  float temperature;
  float humidity;
  uint8_t error;
  uint8_t raw[5];       // Raw bytes from sensor for debugging
};

class DHT22Clone {
public:
  DHT22Clone(uint8_t pin, DHT22Clone_Type type = DHT22_AUTO);
  
  /**
   * Read temperature and humidity from the sensor.
   * 
   * @return DHT22Clone_Result with temperature (°C), humidity (%), 
   *         error code, and raw bytes
   */
  DHT22Clone_Result read();
  
  /**
   * Get the last error code.
   */
  uint8_t getLastError() const { return _lastError; }
  
  /**
   * Get the last temperature reading.
   */
  float getTemperature() const { return _temperature; }
  
  /**
   * Get the last humidity reading.
   */
  float getHumidity() const { return _humidity; }
  
  /**
   * Get human-readable error string.
   */
  static const char* errorToString(uint8_t error);

private:
  uint8_t _pin;
  DHT22Clone_Type _type;
  uint8_t _lastError;
  float _temperature;
  float _humidity;
  
  float parseTemperature(uint8_t byte2, uint8_t byte3);
};
