# This example takes the temperature from the Pico's onboard temperature sensor, and displays it on Pico Display Pack, along with a little pixelly graph.
# It's based on the thermometer example in the "Getting Started with MicroPython on the Raspberry Pi Pico" book, which is a great read if you're a beginner!

import machine
import utime

import st7789
from pimoroni import RGBLED

display = st7789.ST7789(st7789.DISPLAY_PICO_DISPLAY, rotate=0)
display.set_palette_mode(st7789.PALETTE_USER)

# Set the display backlight to 50%
display.set_backlight(0.5)

WIDTH, HEIGHT = display.get_bounds()

BLACK = display.create_pen(0, 0, 0)
WHITE = display.create_pen(255, 255, 255)
TEMPERATURE = WHITE + 1

led = RGBLED(6, 7, 8)

# reads from Pico's temp sensor and converts it into a more manageable number
sensor_temp = machine.ADC(4)
conversion_factor = 3.3 / (65535)
temp_min = 10
temp_max = 30
bar_width = 5

temperatures = []

colors = [(0, 0, 255), (0, 255, 0), (255, 255, 0), (255, 0, 0)]


def temperature_to_color(temp):
    temp = min(temp, temp_max)
    temp = max(temp, temp_min)

    f_index = float(temp - temp_min) / float(temp_max - temp_min)
    f_index *= len(colors) - 1
    index = int(f_index)

    if index == len(colors) - 1:
        return colors[index]

    blend_b = f_index - index
    blend_a = 1.0 - blend_b

    a = colors[index]
    b = colors[index + 1]

    return [int((a[i] * blend_a) + (b[i] * blend_b)) for i in range(3)]


while True:
    # fills the screen with black
    display.set_pen(BLACK)
    display.clear()

    # the following two lines do some maths to convert the number from the temp sensor into celsius
    reading = sensor_temp.read_u16() * conversion_factor
    temperature = 27 - (reading - 0.706) / 0.001721

    temperatures.append(temperature)

    # shifts the temperatures history to the left by one sample
    if len(temperatures) > WIDTH // bar_width:
        temperatures.pop(0)

    i = 0
    for t in temperatures:
        # chooses a pen colour based on the temperature and update the palette entry
        display.set_palette(TEMPERATURE, st7789.RGB565(*temperature_to_color(t)))
        display.set_pen(TEMPERATURE)

        # draws the reading as a tall, thin rectangle
        display.rectangle(i, HEIGHT - (round(t) * 4), bar_width, HEIGHT)

        # the next tall thin rectangle needs to be drawn
        # "bar_width" (default: 5) pixels to the right of the last one
        i += bar_width

    # heck lets also set the LED to match
    led.set_rgb(*temperature_to_color(temperature))

    # draws a white background for the text
    display.set_pen(WHITE)
    display.rectangle(1, 1, 100, 25)

    # writes the reading as text in the white rectangle
    display.set_pen(BLACK)
    display.text("{:.2f}".format(temperature) + "c", 3, 3, 0, 3)

    # time to update the display
    display.update()

    # waits for 5 seconds
    utime.sleep(5)
