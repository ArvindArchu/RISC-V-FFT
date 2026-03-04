run:
	iverilog -I rtl -o sim/cpu_tb.vvp rtl/*.v tb/cpu_tb.v
	vvp sim/cpu_tb.vvp
	gtkwave sim/cpu_tb.vcd
