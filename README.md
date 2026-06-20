This flight controller is based on the [Carbon Aeronautics](https://github.com/CarbonAeronautics?tab=repositories) project. The code is structured like popular flight controllers, but works at a more basic level. Some of the hardware components are out of stock, so I modified the main board to support the alternative hardware.

1. SBUs is connected to pin 0, the original pin 14 is disconnected
2. Buzz driver is connected to pin 23
3. It uses 2S Lipo BLHeli_S ESCs
4. It uses 2S EXT1204-KV5000 Happymodel Motor
5. It uses FPV UBEC 5V to power the main board
6. It uses Taranis transmitter
7. It uses 900MHz R9 MM-OTA SBUS receiver
8. It uses Buzz driver PAM8904
9. It uses MicroSD card socket (WM14405CT-ND) for Teensy 4.0

I published a post about the motor mixer derivation, you can check it [here](https://marcelino-portfolio.netlify.app/posts/).
