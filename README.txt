Contributors:Yuanyu Huang, Guangwei Chen

MCU: Atmel ATmega325

IDE: Atmel Studio 7 (AVR-C)

Debugger: Atmel ICE (ISP) Note: using JTAG makes ADC channel 4-7 unusable.

IMPORTANT: Please see this code only as a reference, since the code is based on specific PCB and other HW. Any minor change can potentially
cause failure. The code can be simplified by integrating into RTOS which is supported by Atmel Studio(ASF). However the Purdue ECE477 doesn't allow dev board for 
final design so the code is completely written in C.

FUNCTION: The code is for digital harp (video: https://www.youtube.com/watch?v=FjgvhXFUIPo). More descriptions and documents are included in
Purdue ECE477 archive website: https://engineering.purdue.edu/ece477/Archive/archive.php.