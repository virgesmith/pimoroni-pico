# cmake file for Pimoroni Enviro with Raspberry Pi Pico W
set(MICROPY_BOARD PICO_W)

# The C malloc is needed by cyw43-driver Bluetooth and Pimoroni Pico modules
set(MICROPY_C_HEAP_SIZE 4096)

set(MICROPY_PY_LWIP ON)
set(MICROPY_PY_NETWORK_CYW43 ON)

# Bluetooth
set(MICROPY_PY_BLUETOOTH ON)
set(MICROPY_BLUETOOTH_BTSTACK ON)
set(MICROPY_PY_BLUETOOTH_CYW43 ON)

# Board specific version of the frozen manifest
set(MICROPY_FROZEN_MANIFEST ${MICROPY_BOARD_DIR}/manifest.py)