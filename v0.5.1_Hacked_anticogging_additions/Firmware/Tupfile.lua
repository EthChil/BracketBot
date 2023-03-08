
tup.include('build.lua')

-- If we simply invoke python or python3 on a pristine Windows 10, it will try
-- to open the Microsoft Store which will not work and hang tup instead. The
-- command "python --version" does not open the Microsoft Store.
-- On some systems this may return a python2 command if Python3 is not installed.
function find_python3()
    success, python_version = run_now("python --version 2>&1")
    if success and string.match(python_version, "Python 3") then return "python -B" end
    success, python_version = run_now("python3 --version 2>&1")
    if success and string.match(python_version, "Python 3") then return "python3 -B" end
    error("Python 3 not found.")
end

python_command = find_python3()
print('Using python command "'..python_command..'"')

tup.frule{inputs={'fibre/cpp/interfaces_template.j2'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template %f --output %o', outputs='autogen/interfaces.hpp'}
tup.frule{inputs={'fibre/cpp/function_stubs_template.j2'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template %f --output %o', outputs='autogen/function_stubs.hpp'}
tup.frule{inputs={'fibre/cpp/endpoints_template.j2'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --generate-endpoints ODrive --template %f --output %o', outputs='autogen/endpoints.hpp'}
tup.frule{inputs={'fibre/cpp/type_info_template.j2'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template %f --output %o', outputs='autogen/type_info.hpp'}

-- Note: we currently check this file into source control for two reasons:
--  - Don't require tup to run in order to use odrivetool from the repo
--  - On Windows, tup is unhappy with writing outside of the tup directory
-- TODO: use CI to verify that on PRs the enums.py file is consistent with the YAML.
--tup.frule{command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template enums_template.j2 --output ../tools/odrive/enums.py'}

tup.frule{
    command=python_command..' ../tools/odrive/version.py --output %o',
    outputs={'autogen/version.c'}
}


-- Switch between board versions
boardversion = tup.getconfig("BOARD_VERSION")
if boardversion == "v3.1" then
    boarddir = 'Board/v3' -- currently all platform code is in the same v3.3 directory
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=1"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.2" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=2"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.3" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=3"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.4-24V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=4"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.4-48V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=4"
    FLAGS += "-DHW_VERSION_VOLTAGE=48"
elseif boardversion == "v3.5-24V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=5"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.5-48V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=5"
    FLAGS += "-DHW_VERSION_VOLTAGE=48"
elseif boardversion == "v3.6-24V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=6"
    FLAGS += "-DHW_VERSION_VOLTAGE=24"
elseif boardversion == "v3.6-56V" then
    boarddir = 'Board/v3'
    FLAGS += "-DHW_VERSION_MAJOR=3 -DHW_VERSION_MINOR=6"
    FLAGS += "-DHW_VERSION_VOLTAGE=56"
elseif boardversion == "" then
    error("board version not specified - take a look at tup.config.default")
else
    error("unknown board version "..boardversion)
end
buildsuffix = boardversion

-- USB I/O settings
if tup.getconfig("USB_PROTOCOL") == "native" or tup.getconfig("USB_PROTOCOL") == "" then
    FLAGS += "-DUSB_PROTOCOL_NATIVE"
elseif tup.getconfig("USB_PROTOCOL") == "native-stream" then
    FLAGS += "-DUSB_PROTOCOL_NATIVE_STREAM_BASED"
elseif tup.getconfig("USB_PROTOCOL") == "stdout" then
    FLAGS += "-DUSB_PROTOCOL_STDOUT"
elseif tup.getconfig("USB_PROTOCOL") == "none" then
    FLAGS += "-DUSB_PROTOCOL_NONE"
else
    error("unknown USB protocol")
end

-- UART I/O settings
if tup.getconfig("UART_PROTOCOL") == "native" then
    FLAGS += "-DUART_PROTOCOL_NATIVE"
elseif tup.getconfig("UART_PROTOCOL") == "ascii" or tup.getconfig("UART_PROTOCOL") == "" then
    FLAGS += "-DUART_PROTOCOL_ASCII"
elseif tup.getconfig("UART_PROTOCOL") == "stdout" then
    FLAGS += "-DUART_PROTOCOL_STDOUT"
elseif tup.getconfig("UART_PROTOCOL") == "none" then
    FLAGS += "-DUART_PROTOCOL_NONE"
else
    error("unknown UART protocol "..tup.getconfig("UART_PROTOCOL"))
end

-- GPIO settings
if tup.getconfig("STEP_DIR") == "y" then
    if tup.getconfig("UART_PROTOCOL") == "none" then
        FLAGS += "-DUSE_GPIO_MODE_STEP_DIR"
    else
        error("Step/dir mode conflicts with UART. Set CONFIG_UART_PROTOCOL to none.")
    end
end

-- Compiler settings
if tup.getconfig("STRICT") == "true" then
    FLAGS += '-Werror'
end

-- C-specific flags
FLAGS += '-D__weak="__attribute__((weak))"'
FLAGS += '-D__packed="__attribute__((__packed__))"'
FLAGS += '-DUSE_HAL_DRIVER'
FLAGS += '-DSTM32F405xx'

FLAGS += '-mthumb'
FLAGS += '-mcpu=cortex-m4'
FLAGS += '-mfpu=fpv4-sp-d16'
FLAGS += '-mfloat-abi=hard'
FLAGS += { '-Wall', '-Wdouble-promotion', '-Wfloat-conversion', '-fdata-sections', '-ffunction-sections'}

-- linker flags
LDFLAGS += '-T'..boarddir..'/STM32F405RGTx_FLASH.ld'
LDFLAGS += '-L'..boarddir..'/Drivers/CMSIS/Lib' -- lib dir
LDFLAGS += '-lc -lm -lnosys -larm_cortexM4lf_math' -- libs
LDFLAGS += '-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections'
LDFLAGS += '-Wl,--undefined=uxTopUsedPriority'

-- debug build
if tup.getconfig("DEBUG") == "true" then
    FLAGS += '-g -gdwarf-2'
    OPT += '-Og'
else
    OPT += '-O2'
end

-- common flags for ASM, C and C++
OPT += '-ffast-math -fno-finite-math-only'
tup.append_table(FLAGS, OPT)
tup.append_table(LDFLAGS, OPT)

toolchain = GCCToolchain('arm-none-eabi-', 'build', FLAGS, LDFLAGS)


-- Load list of source files Makefile that was autogenerated by CubeMX
vars = parse_makefile_vars(boarddir..'/Makefile')
all_stm_sources = (vars['C_SOURCES'] or '')..' '..(vars['CPP_SOURCES'] or '')..' '..(vars['ASM_SOURCES'] or '')
for src in string.gmatch(all_stm_sources, "%S+") do
    stm_sources += boarddir..'/'..src
end
for src in string.gmatch(vars['C_INCLUDES'] or '', "%S+") do
    stm_includes += boarddir..'/'..string.sub(src, 3, -1) -- remove "-I" from each include path
end

-- TODO: cleaner separation of the platform code and the rest
stm_includes += '.'
stm_includes += 'Drivers/DRV8301'
stm_sources += boarddir..'/Src/syscalls.c'
build{
    name='stm_platform',
    type='objects',
    toolchains={toolchain},
    packages={},
    sources=stm_sources,
    includes=stm_includes
}


build{
    name='ODriveFirmware',
    toolchains={toolchain},
    --toolchains={LLVMToolchain('x86_64', {'-Ofast'}, {'-flto'})},
    packages={'stm_platform'},
    sources={
        'Drivers/DRV8301/drv8301.c',
        'MotorControl/utils.cpp',
        'MotorControl/arm_sin_f32.c',
        'MotorControl/arm_cos_f32.c',
        'MotorControl/low_level.cpp',
        'MotorControl/nvm.c',
        'MotorControl/axis.cpp',
        'MotorControl/motor.cpp',
        'MotorControl/thermistor.cpp',
        'MotorControl/encoder.cpp',
        'MotorControl/endstop.cpp',
        'MotorControl/controller.cpp',
        'MotorControl/sensorless_estimator.cpp',
        'MotorControl/trapTraj.cpp',
        'MotorControl/main.cpp',
        'communication/can_simple.cpp',
        'communication/communication.cpp',
        'communication/ascii_protocol.cpp',
        'communication/interface_uart.cpp',
        'communication/interface_usb.cpp',
        'communication/interface_can.cpp',
        'communication/interface_i2c.cpp',
        'fibre/cpp/protocol.cpp',
        'FreeRTOS-openocd.c',
        'autogen/version.c'
    },
    includes={
        'Drivers/DRV8301',
        'MotorControl',
        'fibre/cpp/include',
        '.',
        "doctest"
    }
}

if tup.getconfig('DOCTEST') == 'true' then
    TEST_INCLUDES = '-I. -I./MotorControl -I./fibre/cpp/include -I./Drivers/DRV8301 -I./doctest'
    tup.foreach_rule('Tests/*.cpp', 'g++ -O3 -std=c++17 '..TEST_INCLUDES..' -c %f -o %o', 'Tests/bin/%B.o')
    tup.frule{inputs='Tests/bin/*.o', command='g++ %f -o %o', outputs='Tests/test_runner.exe'}
    tup.frule{inputs='Tests/test_runner.exe', command='%f'}
end
