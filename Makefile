all:
	python3 ./cm3sd.py -e -w sample.hex -r /dev/serial/by-id/usb-SEGGER_J-Link_*-if00

ctrl:
	python ./cm3sd.py -e -w sample.hex -r /dev/serial/by-id/usb-SEGGER_J-Link_*-if00
