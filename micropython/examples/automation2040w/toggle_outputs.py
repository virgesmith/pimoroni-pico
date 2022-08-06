import time
from automation import Automation2040W, SWITCH_A, NUM_OUTPUTS

"""
Demonstrates how to toggle each of Automation 2040 W's output terminals.

Press "A" to exit the program.
"""

TIME_PER_TOGGLE = 0.5               # How much time to wait between each toggle (in seconds)
OUTPUT_NAMES = ("O1", "O2", "O3")   # The friendly names to give each digital output

# Create a new Automation2040W
board = Automation2040W()

# Enable the LED of the switch used to exit the loop
board.switch_led(SWITCH_A, 50)  # Half Brightness

toggle = True
index = 0

# Toggle the outputs until the user switch is pressed
while not board.switch_pressed(SWITCH_A):

    # Toggle an output
    board.output(index, toggle)

    # Print the state of all outputs
    for i in range(NUM_OUTPUTS):
        print(OUTPUT_NAMES[i], " = ", board.output(i), sep="", end=", ")

    # Print a new line
    print()

    index += 1                  # Move on to the next output
    if index >= NUM_OUTPUTS:
        index = 0               # Go back to the first output
        toggle = not toggle     # Invert the toggle value

    time.sleep(TIME_PER_TOGGLE)

# Put the board back into a safe state
board.reset()
