# Automation 2040W MicroPython <!-- omit in toc -->

This library offers convenient functions for interacting with your new [Pimoroni Automation 2040W](https://shop.pimoroni.com/products/automation-2040-w), an all-in-one, Pico W powered industrial/automation controller with 2.4GHz wireless connectivity, relays and a plethora of inputs and outputs.

## Table of Content
- [Automation 2040W Class](#automation-2040w-class)
  - [User Switches and LEDs](#user-switches-and-leds)
  - [Connectivity LED](#connectivity-led)
  - [Actuating the Relays](#actuating-the-relays)
  - [Setting the Outputs](#setting-the-outputs)
  - [Reading the Inputs](#reading-the-inputs)
  - [Reading the ADCs](#reading-the-adcs)
  - [Extra GPIOs](#extra-gpios)
  - [Software Reset](#software-reset)
  - [Function Reference](#function-reference)


## Automation 2040W Class

The `Automation2040W` class deals with the initialisation of each of the board's functions. To create one, import the `automation` module, then define a new `board` variable:

```python
import automation

board = automation.Automation2040W()
```

From here, all features of Automation 2040W can be accessed by calling functions on `board`. In addition, when using Qwiic / Stemma QT devices, the I2C channel to use can be accessed with `board.i2c`.


### User Switches and LEDs

Automation 2040W has two handy switches onboard, with neighbouring LEDs, offering a tactile way to interact with your program and be notified of actions that need attention.

To read one of the switches, call `.switch_pressed(switch)`, where `switch` is a value from `0` to `NUM_SWITCHES - 1`. This returns `True` when the specified switch is pressed, and `False` otherwise.

To set a switch's neighbouring LED, call `.switch_led(switch, brightness)`, where `switch` is a value from `0` to `NUM_SWITCHES - 1`, and `brightness` is either `True`, `False`, or a number from `0.0` to `100.0`.


To make it easier to use a specific switch or it's LED, the `automation` module contains these handy constants:
* `SWITCH_A` = `0`
* `SWITCH_B` = `1`


### Connectivity LED

In addition to the Switch LEDs, Automation 2040W has a user-controllable connectivity LED near the top-right of the board.

To set this led, call `.conn_led(brightness)`, where `brightness` is either `True`, `False`, or a number from `0.0` to `100.0`.


### Actuating the Relays

Three relays are featured on Automation 2040W. By default these are in a released state, which connects the terminal labelled `NC` to `COM`. By actuating them, a connection from `NO` to `COM` can be made instead.

A relay can be actuated by calling `.actuate_relay(relay)`, or released by calling `.release_relay(relay)`. Additionally the actuated state can be set by providing a boolean to the `actuate` parameter of `.relay(relay, actuate)`.

The state of each relay can be read by calling `.relay(relay)`. This returns `True` if the relay is actuated, and `False` if it is released. The actuation state is also reflected by LEDs that neighbour each relay.

For all these functions, `relay` is a value from `0` to `NUM_RELAYS - 1`. To control a specific relay, the `automation` module contains these handy constants:
* `RELAY_1` = `0`
* `RELAY_2` = `1`
* `RELAY_3` = `2`


### Setting the Outputs

Three sourcing outputs, capable of 2A+, are present on Automation 2040W.

An output can be controlled by calling `.output(output, value)`, where `output` is a value from `0` to `NUM_OUTPUTS - 1`, and `value` is `True` or `False`.

The state of an output can be read by calling `.output(output)`, where `output` is a value from `0` to `NUM_OUTPUTS - 1`. This returns `True` if the output is active, and `False` if it is inactive. The state is also reflected by LEDs that neighbour each output terminal.

To control a specific output, the `automation` module contains these handy constants:
* `OUTPUT_1` = `0`
* `OUTPUT_2` = `1`
* `OUTPUT_3` = `2`


### Reading the Inputs

Automation 2040W has four buffered digital inputs. These can be read by calling `.read_input(input)`, where `input` is a value from `0` to `NUM_INPUTS - 1`.

To read a specific input, the `automation` module contains these handy constants:
* `INPUT_1` = `0`
* `INPUT_2` = `1`
* `INPUT_3` = `2`
* `INPUT_4` = `3`


### Reading the ADCs

Automation 2040W has three analog inputs, capable of reading up to 40V. The voltage on these can be read by calling `.read_adc(adc)`, where `adc` is a value from `0` to `NUM_ADCS - 1`.

To read a specific adc, the `automation` module contains these handy constants:
* `ADC_1` = `0`
* `ADC_2` = `1`
* `ADC_3` = `2`


### Extra GPIOs

On the left hand side of Automation 2040W are three GPIO pins. These are 3.3V logic only, and are connected to GP0, GP1, and GP2 of the Pico W. These pins can be referenced in code using `0`, `1`, and `2`, or by one of these handy constants on the `automation` module:
* `GP0` = `0`
* `GP1` = `1`
* `GP2` = `2`

There is also a `NUM_GPIOS` for times when any iteration needs to be performed.


### Software Reset

If there is a need to put Automation 2040W back into a known safe-state, without resorting to the hardware reset switch, then `.reset()` can be called. This deactivates all outputs, releases all relays, and turns off all user-controllable LEDs.


### Function Reference

Here is the complete list of functions available on the `Automation2040W` class:

```python
Automation2040W()
conn_led(brightness)
switch_pressed(switch)
switch_led(switch, brightness)
relay(relay)
relay(relay, actuate)
actuate_relay(relay)
release_relay(relay)
output(output)
output(output, value)
read_input(input)
read_adc(adc)
reset()
```
