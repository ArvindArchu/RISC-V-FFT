`timescale 1ns / 1ps
module cpu_tb;
    reg clk;
    reg reset;
    integer i;

    // Instantiate the CPU
    cpu uut (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation: 10ns period (100MHz)
    always #5 clk = ~clk;

    initial begin
        // Initialize
        clk = 0;
        reset = 1;
        #50 reset = 0;  // Release reset

        // Run simulation for some cycles
        #500;

        // Dump register values
        $display("=== Register File Contents ===");
        for (i = 0; i < 32; i = i + 1) begin
            $display("x%0d = %h", i, uut.u_regfile.regs[i]);
        end

        // Dump data memory values
        $display("=== Data Memory Contents ===");
        for (i = 0; i < 32; i = i + 1) begin
            $display("MEM[%0d] = %h", i, uut.u_dmem.mem[i]);
        end 
        $finish;
    end
endmodule
