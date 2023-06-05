# STM32_Serial_Communication

#Implemented USB serial communication, controlled temperature ADC measurement, and package elements recognition.
#Added full integration of google Protocol Buffers using library github.com/nanopb. The program now serializes(encrypts) all incoming serial communication, so only the exact described messages will be able to control the measuring process. Those message-commands are: (0801100A186400 - measurements on; 0802101418c801 - measurements off; 0803101E18ac02  - diode 12 on; 08041028189003  - diode 12 off). The program decrypts those messages as three integers described in the proto file, then switches depending on those integers. If measurement-command was issued, the ADC measurements start and then are encrypted and sent over the serial connection. Decryption takes more code space because the messages need to be converted from string format into appropriate hexadecimal format, which is accomplished using function.
#-Igor Elche
