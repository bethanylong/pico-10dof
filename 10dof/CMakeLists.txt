add_executable(10dof
        mpu6050.c
        qmc5883l.c
        bmp280.c
        10dof.c
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(10dof pico_stdlib hardware_i2c)

# blong
# enable usb output, disable uart output
pico_enable_stdio_usb(10dof 1)
pico_enable_stdio_uart(10dof 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(10dof)

# add url via pico_set_program_url
example_auto_set_url(10dof)
