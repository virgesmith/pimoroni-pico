import gc
from machine import Pin, PWM
from pimoroni_i2c import PimoroniI2C
from pimoroni import Analog

# IO Pin Constants
GP0 = 0
GP1 = 1
GP2 = 2

# Index Constants
RELAY_1 = 0
RELAY_2 = 1
RELAY_3 = 2

OUTPUT_1 = 0
OUTPUT_2 = 1
OUTPUT_3 = 2

ADC_1 = 0
ADC_2 = 1
ADC_3 = 2

INPUT_1 = 0
INPUT_2 = 1
INPUT_3 = 2
INPUT_4 = 3

SWITCH_A = 0
SWITCH_B = 1

# Count Constants
NUM_GPIOS = 3
NUM_RELAYS = 3
NUM_OUTPUTS = 3
NUM_ADCS = 3
NUM_INPUTS = 4
NUM_SWITCHES = 2


class Automation2040W():
    CONN_LED_PIN = 3
    I2C_SDA_PIN = 4
    I2C_SCL_PIN = 5
    ADC_LED_PINS = (6, 7, 8)
    RELAY_PINS = (9, 10, 11)
    USER_SW_PINS = (12, 13)
    USER_LED_PINS = (14, 15)
    OUTPUT_PINS = (16, 17, 18)
    IN_BUFFERED_PINS = (19, 20, 21, 22)
    ADC_PINS = (26, 27, 28)

    VOLTAGE_GAIN = 0.06  # 56 / (56 + 820)
    VOLTAGE_OFFSET = -0.06
    MAX_ADC_LED_VOLTAGE = 45.0

    def __init__(self):
        # Free up hardware resources
        gc.collect()

        # Set up the i2c for Qw/st
        self.i2c = PimoroniI2C(self.I2C_SDA_PIN, self.I2C_SCL_PIN, 100000)

        # Set up the relay pins
        self.__relays = []
        for i in range(NUM_RELAYS):
            self.__relays.append(Pin(self.RELAY_PINS[i], Pin.OUT))
            self.release_relay(i)

        # Set up the output pins
        self.__outputs = []
        for i in range(NUM_OUTPUTS):
            self.__outputs.append(Pin(self.OUTPUT_PINS[i], Pin.OUT))
            self.output(i, False)

        # Set up the input pins
        self.__inputs = []
        for i in range(NUM_INPUTS):
            self.__inputs.append(Pin(self.IN_BUFFERED_PINS[i], Pin.IN))

        # Set up the adc pins and leds
        self.__analogs = []
        self.__adc_led_pwms = []
        for i in range(NUM_ADCS):
            self.__analogs.append(Analog(self.ADC_PINS[i], self.VOLTAGE_GAIN, offset=self.VOLTAGE_OFFSET))
            led_pwm = PWM(Pin(self.ADC_LED_PINS[i]))
            led_pwm.freq(1000)
            led_pwm.duty_u16(0)
            self.__adc_led_pwms.append(led_pwm)

        # Set up the user switches
        self.__switches = []
        self.__switch_led_pwms = []
        for i in range(NUM_SWITCHES):
            self.__switches.append(Pin(self.USER_SW_PINS[i], Pin.IN, Pin.PULL_UP))
            led_pwm = PWM(Pin(self.USER_LED_PINS[i]))
            led_pwm.freq(1000)
            led_pwm.duty_u16(0)
            self.__switch_led_pwms.append(led_pwm)

        # Set up the connectivity LED
        self.__conn_led_pwm = PWM(Pin(self.CONN_LED_PIN))
        self.__conn_led_pwm.freq(1000)
        self.__conn_led_pwm.duty_u16(0)

    def conn_led(self, brightness):
        if brightness is True:
            self.__conn_led_pwm.duty_u16(65535)
        elif brightness is False:
            self.__conn_led_pwm.duty_u16(0)
        elif brightness < 0.0 or brightness > 100.0:
            raise ValueError("brightness out of range. Expected 0 to 100, or True or False")
        else:
            gamma = 2.8
            value = int(pow(brightness / 100.0, gamma) * 65535 + 0.5)
            self.__conn_led_pwm.duty_u16(value)

    def switch_pressed(self, switch):
        if switch < 0 or switch > NUM_SWITCHES:
            raise ValueError("switch out of range. Expected SWITCH_A (0) or SWITCH_B (1)")
        return not self.__switches[switch].value()

    def switch_led(self, switch, brightness):
        if switch < 0 or switch > NUM_SWITCHES:
            raise ValueError("switch out of range. Expected SWITCH_A (0) or SWITCH_B (1)")

        if brightness is True:
            self.__switch_led_pwms[switch].duty_u16(65535)
        elif brightness is False:
            self.__switch_led_pwms[switch].duty_u16(0)
        elif brightness < 0.0 or brightness > 100.0:
            raise ValueError("brightness out of range. Expected 0 to 100, or True or False")
        else:
            gamma = 2.8
            value = int(pow(brightness / 100.0, gamma) * 65535 + 0.5)
            self.__switch_led_pwms[switch].duty_u16(value)

    def relay(self, relay, actuate=None):
        if relay < 0 or relay > NUM_RELAYS:
            raise ValueError("relay out of range. Expected RELAY_1 (0), RELAY_2 (1), or RELAY_3 (2)")

        if actuate is None:
            return self.__relays[relay].value()

        self.__relays[relay].value(actuate)

    def actuate_relay(self, relay):
        if relay < 0 or relay > NUM_RELAYS:
            raise ValueError("relay out of range. Expected RELAY_1 (0), RELAY_2 (1), or RELAY_3 (2)")
        self.__relays[relay].on()

    def release_relay(self, relay):
        if relay < 0 or relay > NUM_RELAYS:
            raise ValueError("relay out of range. Expected RELAY_1 (0), RELAY_2 (1), or RELAY_3 (2)")
        self.__relays[relay].off()

    def output(self, output, value=None):
        if output < 0 or output > NUM_OUTPUTS:
            raise ValueError("output out of range. Expected OUTPUT_1 (0), OUTPUT_2 (1), or OUTPUT_3 (2)")

        if value is None:
            return self.__outputs[output].value()

        self.__outputs[output].value(value)

    def read_input(self, input):
        if input < 0 or input > NUM_INPUTS:
            raise ValueError("input out of range. Expected INPUT_1 (0), INPUT_2 (1), INPUT_3 (2), or INPUT_4 (3)")
        return self.__inputs[input].value()

    def read_adc(self, adc):
        if adc < 0 or adc > NUM_ADCS:
            raise ValueError("adc out of range. Expected ADC_1 (0), ADC_2 (1), or ADC_3 (2)")

        voltage = self.__analogs[adc].read_voltage()
        gamma = 2.8
        value = int(pow(voltage / self.MAX_ADC_LED_VOLTAGE, gamma) * 65535 + 0.5)
        self.__adc_led_pwms[adc].duty_u16(value)
        return voltage

    def reset(self):
        # Reset the relays
        for i in range(NUM_RELAYS):
            self.release_relay(i)

        # Reset the outputs
        for i in range(NUM_OUTPUTS):
            self.output(i, False)

        # Reset the adc LEDs
        for i in range(NUM_ADCS):
            self.__adc_led_pwms[i].duty_u16(0)

        # Reset the switch LEDs
        for i in range(NUM_SWITCHES):
            self.__switch_led_pwms[i].duty_u16(0)

        # Reset the connectivity LED
        self.__conn_led_pwm.duty_u16(0)
