First, before starting, the MCU board must have micropython loaded -- see other docs 
such as micropython_compile_G30TH.txt, micropython_coding_with_rshell.txt about that.

To run code on a culture shock board, (with STM32F401 inside), several files are needed.

As this code is written for v0.5 hardware, it needs these files loaded to the flash filesystem of the 
micropython running MCU:

* pulser_main.py
* stm_low_level.py
* nvic.py
* boot.py
* ssd1306.py
  * (__OPTIONAL__) for 0.96" 128x64 OLED support

After loading those files and resetting the MCU board, the program called by boot.py will run.

