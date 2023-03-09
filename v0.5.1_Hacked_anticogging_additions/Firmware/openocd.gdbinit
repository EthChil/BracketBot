target remote | openocd -f "interface/stlink.cfg" -f "target/stm32f4x_stlink.cfg" -c "gdb_port pipe; log_output openocd.log"
monitor reset halt
