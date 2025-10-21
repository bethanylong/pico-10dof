pico-10dof
==========

Raspberry Pi Pico 2 firmware for a 10 degrees-of-freedom (DOF) inertial measurement unit (IMU).

Sensors used:
- MPU6050 accelerometer/gyroscope (6DOF) for tilt and rotation sensing
- QMC5883L magnetometer (3DOF) for heading sensing
- BMP280 barometer (1DOF?) for elevation sensing


Code quality
------------

- Garbage


Usage
-----

As close as I can remember, this is how you build and flash it. I have not tested these steps recently from start to finish. Enjoy the adventure!

1. Install picotool and set up Pico C SDK
2. Plug in Pico 2
3. `mkdir build && cd build`
4. `cmake .. -DPICO_BOARD=pico2`
5. `cd 10dof`
6. `make -j4 && picotool load -f 10dof.uf2`


Credits
-------

- DeepSeek
