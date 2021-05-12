sudo python ~/esp/esptool/esptool.py --baud 115200 --port /dev/tty.usbserial-1410 write_flash -fm dio 0x00000 ~/RK/RK8266/builds/23/0x00000.bin

