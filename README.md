adi-cm3sd
=========

Flash download utility and simple terminal program for Cortex-M3 based
ADuCxxx devices.  This tool supports the Analog Devices Serial
Download Protocol as described in
[AN-1160](http://www.analog.com/static/imported-files/application_notes/AN-1160.pdf).

Usage
-----

    usage: cm3sd.py [-h] [--baudrate BAUD] [--erase] [--write HEX] [--reset]
                    [--terminal] [--all HEX]
                    DEVICE
    
    Flash download utility and simple terminal program for Cortex-M3 based ADuCxxx
    devices.
    
    positional arguments:
      DEVICE                Serial device
    
    optional arguments:
      -h, --help            show this help message and exit
      --baudrate BAUD, -b BAUD
                            Baudrate (default: 115200)
    
    Actions:
      Actions are performed in the order listed below.
    
      --erase, -e           Bulk erase (default: False)
      --write HEX, -w HEX   Hex file to write (may be repeated) (default: [])
      --reset, -r           Reset and run (default: False)
      --terminal, -t        Interactive terminal (default: False)
      --all HEX, -a HEX     Same as -e -w HEX -r -t (default: None)

Example
-------

    $ ./cm3sd.py -a hello-world.hex /dev/serial/by-id/usb-SEGGER_J-Link_000541012345-if00
    Hold BOOT and press RESET.
    Waiting for bootloader............ ok
    Chip: 'ADuCRF101  128 ' revision 'E30'
    Bulk erase... ok
    File: hello-world.hex (40332 bytes, 0x00000000-0x00009d8f)
    Flashing... 40332/40332 ok
    Resetting... ok
    /dev/serial/by-id/usb-SEGGER_J-Link_000541000394-if00, 115200 baud
    ^C to exit
    ----------
    Contiki-2.6-1551-g49dcb27682ce started
    Using LFOSC for rtimer (34000 Hz)
    Rime started with address 2.1.0.0.0.0.0.0
    RF channel set to 915000000 Hz
    MAC CSMA RDC nullrdc NETWORK Rime
    Hello, world
    ^C
    $
