glasgow run swd-openocd --port A --pin-swclk 0 --pin-swdio 1 --pin-srst 2 -V 3.3 tcp:localhost:2222

openocd -c 'adapter driver remote_bitbang; transport select swd' -c 'remote_bitbang port 2222' -f target/stm32f1x.cfg # -c "program hello-world.hex verify reset exit"

