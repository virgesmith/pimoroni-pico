# micropython build

[reverse-engineered from the pimoroni-pico [github workflow](./.github/workflows/micropython.yml)]

Paths may need adjusting depending on your layout. Below assumes `micropython` and `pimoroni-pico` share an immediate parent directory.

## get the micropython source and initialise the submodules

```sh
git clone git@github.com:micropython/micropython
cd micropython/
git submodule update --init
cd lib/pico-sdk/
git submodule update --init
cd ../..
```

## build the bytecode cross-compiler

```sh
cd mpy-cross/
make
cd ..
```

## build custom micropython for the target

The micropython code seems to push compilers to the limit. Works with gcc 12.2 (see https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) with no modifications to micropython itself and a slight modifications to pimoroni-pico.

Ensure the right compiler is being used, e.g.

```sh
export PATH=/opt/arm-gnu-toolchain-12.2.mpacbti-rel1-x86_64-arm-none-eabi/bin:$PATH
```

`USER_C_MODULES` points to the custom modules in pimoroni-pico. Code below uses `PICO` (the standard pico 2040 board), other options are (replace as necessary):
- `PICO_W`
- `PICO_ENVIRO` (might require a patch to pico-sdk?)
- ...

Then e.g.

```sh
cd ports/rp2/
cmake -S . -B build-PICO -DPICO_BUILD_DOCS=0 -DUSER_C_MODULES=../../../pimoroni-pico/micropython/modules/micropython-pico.cmake  -DMICROPY_BOARD_DIR=../../../pimoroni-pico/micropython/board/PICO -DMICROPY_BOARD=PICO
cmake --build build-PICO -j
```


### pico enviro

needs patch..?

*Et voila!* image is at `ports/rp2/build-PICO/firmware.uf2`.
