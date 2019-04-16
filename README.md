# android_serial_tool
a command line tool to read/write serial port

# Usage:
---
*	./serial_tool /dev/ttyS1 read
*	./serial_tool /dev/ttyS1 write abcdefg
*	./serial_tool /dev/ttyS1 test 0x01 0x02 0x03

# Note:
---
*	1) for read, you need check the result in another logcat console
*	2) write a string to serial port
*	3) write a serial Hex num to the serial port

