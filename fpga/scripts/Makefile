# makefile to synthesize and run nextpnr
#
SRC = ../verilog/etarget.sv  ../verilog/jml_i2c.sv  ../verilog/target_counter.sv ../verilog/slope_det.sv ../verilog/data_recorder.sv
VLIB = /usr/local/share/yosys/ice40/cells_sim.v

ALL: compress
synth: etarget.json
pnr: etarget.asc 
compress: etarget.bin
# I could get clever and pass the files into the script but theyar eduplicated in the .ys scrips
etarget.json : etarget.ys $(SRC) $(VLIB)
	yosys -s etarget.ys

etarget.asc : etarget.json etargetu5k.pcf etarget_cons.py
	nextpnr-ice40 --up5k --package sg48 --verbose --top  etarget --json etarget.json --pcf etargetu5k.pcf --asc etarget.asc --ignore-loops --pre-pack etarget_cons.py

etarget.bin: etarget.asc
	icepack etarget.asc etarget.bin
