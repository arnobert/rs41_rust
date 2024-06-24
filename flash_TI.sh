cargo build
cargo objcopy -- -O ihex hello-world.hex
openocd -f interface/ftdi/luminary-icdi.cfg -c "transport select swd"  -f target/stm32f1x.cfg -c "program hello-world.hex verify reset exit"

