Etarget FPGA

The purpose of this FPGA is to move the misc digital logic from the
board into this FPGA to simplify the board design and allow the use of
a processor with less GPIO IE the esp8266.

Since the FPGA will have a significant amount of free IO and
resources, some signal proccessing may be moved into the FPGA in the
future.

Design will include sigma delta ADC's for direct mic sampling

The FPGA will be updateable over the air from the esp2866.  On power
on the esp8266 will load the fpga through the slave spi port.

The fpga is coded in verilog/system verilog;
Synthesized with the open source yosys synthesis tool;
pleace and route with the open source next-pnr tool;
all thanks to the iceaxe project.

Simulation and verification through icarus verilog.


