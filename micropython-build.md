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

`USER_C_MODULES` points to the custom modules in pimoroni-pico. Code below uses `PICO` (the standard pico 2040 board), other options are (replace as necessary):
- `PIMORONI_TINY2040`
- `PIMORONI_PICOLIPO_4MB`
- `PIMORONI_PICOLIPO_16MB`

```sh
cd ports/rp2/
cmake -S . -B build-PICO -DPICO_BUILD_DOCS=0 -DUSER_C_MODULES=../../../pimoroni-pico/micropython/modules/micropython-pico.cmake -DMICROPY_BOARD=PICO -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
cmake --build build-PICO -j
```
(`ccache` can be installed from e.g. apt)

*Et voila!* image is at `ports/rp2/build-PICO/firmware.uf2`.
