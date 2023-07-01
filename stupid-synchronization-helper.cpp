#include "./stupid-synchronization-helper.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_system.h"

void memory_barrier() {
  __sync_synchronize();
  delay(1);
}

