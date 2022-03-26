#include "pico/stdlib.h"

#include "servo2040.hpp"
#include "button.hpp"

/*
An example of applying a wave pattern to a group of servos and the LEDs.

Press "Boot" to exit the program.
*/

using namespace plasma;
using namespace servo;

// The speed that the LEDs will cycle at
const uint SPEED = 5;

// The brightness of the LEDs
constexpr float BRIGHTNESS = 0.4;

// How many times to update LEDs and Servos per second
const uint UPDATES = 50;

// How far from zero to move the servos
constexpr float SERVO_EXTENT = 80.0f;

// Create a servo cluster for pins 0 to 7, using PIO 0 and State Machine 0
const uint START_PIN = servo2040::SERVO_1;
const uint END_PIN = servo2040::SERVO_8;
const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;
ServoCluster servos = ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);

// Create the LED bar, using PIO 1 and State Machine 0
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

// Create the user button
Button user_sw(servo2040::USER_SW);


int main() {
  stdio_init_all();

  // Initialise the servo cluster
  servos.init();

  // Start updating the LED bar
  led_bar.start();

  float offset = 0.0f;

  // Make rainbows until the user button is pressed
  while(!user_sw.raw()) {

    offset += (float)SPEED / 1000.0f;

    // Update all the LEDs
    for(auto i = 0u; i < servo2040::NUM_LEDS; i++) {
        float hue = (float)i / (float)(servo2040::NUM_LEDS * 4);
        led_bar.set_hsv(i, hue + offset, 1.0f, BRIGHTNESS);
    }

    // Update all the Servos
    for(auto i = 0u; i < servos.get_count(); i++) {
        float angle = (((float)i / (float)servos.get_count()) + offset) * (float)M_TWOPI;
        servos.set_value(i, sin(angle) * SERVO_EXTENT, false);
    }
    // We have now set all the servo values, so load them
    servos.load();

    sleep_ms(1000 / UPDATES);
  }

  // Stop all the servos
  servos.disable_all();

  // Turn off the LED bar
  led_bar.clear();

  // Sleep a short time so the clear takes effect
  sleep_ms(100);
}