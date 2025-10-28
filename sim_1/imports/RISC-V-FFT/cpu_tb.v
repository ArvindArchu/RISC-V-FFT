
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
        #20 reset = 0;  // Release reset

        // Run simulation for some cycles
        #5000;

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

        $display("\n=== FFT Output (Q1.15) ===");
        for (i = 0; i < 8; i = i + 1) begin
            $display("X[%0d] = %0d + j%0d",
                    i,
                    $signed(uut.fft_out_re_flat[i*16 +: 16]),
                    $signed(uut.fft_out_im_flat[i*16 +: 16]));
        end

        $finish;
    end
endmodule

/*`timescale 1ns / 1ps

module cpu_tb;

    // --- Testbench Parameters ---
    parameter FFT_N = 64;
    parameter FFT_DATA_W = 16;
    parameter CPU_DATA_W = 32;
    parameter ADDR_W = 32;
    parameter RE_BASE_ADDR = 1000; // Byte address for Real input/output
    parameter IM_BASE_ADDR = 2000; // Byte address for Imag input/output
    parameter N_SAMPLES = FFT_N;
    parameter N_WORDS = N_SAMPLES / 2; // Packed 2 samples per word

    // --- Testbench Signals ---
    reg clk;
    reg reset; // Assuming active-high reset for CPU

    // --- Loop Variables (Module Scope) ---
    integer i;
    integer k;

    // --- Temporary Variables for Initialization (Module Scope) ---
    reg signed [FFT_DATA_W-1:0] temp_re0, temp_re1, temp_im0, temp_im1;

    // Instantiate the CPU (which contains internal memories and FFT interface)
    cpu #(
        .FFT_N(FFT_N),
        .FFT_DATA_W(FFT_DATA_W)
        // Pass other CPU parameters if needed
    ) uut (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation: 10ns period (100MHz)
    initial clk = 0;
    always #5 clk = ~clk;

    // Main initial block for reset, loading, initialization, verification
    initial begin
        // Variables declared above are now accessible here

        // --- Apply Reset ---
        reset = 1;
        #50; // Hold reset
        reset = 0;
        #20; // Wait for reset release
        $display("t=%0t Reset released.", $time);

        // --- Load Instruction Memory ---
        // Access internal Inst_MEM using hierarchical path
        // ** Adjust path uut.u_Inst_MEM1.mem if needed **
        // Assuming Inst_MEM uses word addressing for its internal 'mem' array
        $display("Loading Instruction Memory...");
        uut.u_Inst_MEM1.mem[0] = 32'h00000513; // addi x10, x0, RE_BASE_ADDR
        uut.u_Inst_MEM1.mem[1] = 32'h00000593; // addi x11, x0, IM_BASE_ADDR
        uut.u_Inst_MEM1.mem[2] = 32'h00B5000B; // FFT x10, x11 (custom-0, rs1=10, rs2=11, rd=0)
        uut.u_Inst_MEM1.mem[3] = 32'h00000013; // NOP
        uut.u_Inst_MEM1.mem[4] = 32'h00000013; // NOP
        // Corrected lw instructions using hex values
        uut.u_Inst_MEM1.mem[5] = 32'h00052603; // lw x12, 0(x10) ; Load Re[1], Re[0] output
        uut.u_Inst_MEM1.mem[6] = 32'h0005A683; // lw x13, 0(x11) ; Load Im[1], Im[0] output
        uut.u_Inst_MEM1.mem[7] = 32'h0000006F; // jal x0, . (halt loop at PC=28)
        // Initialize rest of instruction memory? (Optional, defaults often 0)
        for (i = 8; i < 256; i=i+1) uut.u_Inst_MEM1.mem[i] = 32'h00000013; // Fill with NOPs
        $display("Instruction Memory Loaded.");


        // --- Initialize Data Memory ---
        // Access internal Data_MEM using hierarchical path
        // ** Adjust path uut.u_dmem.mem if needed **
        $display("Initializing Data Memory for FFT Input...");
        for (k = 0; k < N_WORDS; k = k + 1) begin
            // Generate samples i and i+1 (where i = k*2)
            if (k == 0) begin // Word 0: samples 1 (high), 0 (low)
                temp_re0 = 16'h7FFF; temp_im0 = 16'h0000; // Sample 0 = 1.0 + 0j
                temp_re1 = 16'h0000; temp_im1 = 16'h0000; // Sample 1 = 0.0 + 0j
            end else begin      // Other words are 0
                temp_re0 = 16'h0000; temp_im0 = 16'h0000;
                temp_re1 = 16'h0000; temp_im1 = 16'h0000;
            end

            // Assuming Data_MEM uses word addressing internally (index = byte_addr / 4)
            // ** Ensure this path is correct for your setup **
            uut.u_dmem.mem[ (RE_BASE_ADDR/4) + k ] = {temp_re1, temp_re0}; // Pack Re[i+1], Re[i]
            uut.u_dmem.mem[ (IM_BASE_ADDR/4) + k ] = {temp_im1, temp_im0}; // Pack Im[i+1], Im[i]
        end
        $display("Data Memory Initialized at addresses %0d and %0d.", RE_BASE_ADDR, IM_BASE_ADDR);

        // --- Run Simulation ---
        $display("Starting CPU execution...");
        // *** REDUCED RUN TIME ***
        #5000; // Run for 5 us (500 cycles) - Adjust if needed

        // --- Verification (after simulation run) ---
        $display("\n=== Simulation Complete (at t=%0t) ===", $time);

        // --- Dump register values (Simple loop for x0-x10) ---
        // ** Adjust path uut.u_regfile.regs if needed **
        $display("=== Register File Contents (x0-x10) ===");
        for (i = 0; i < 11; i = i + 1) begin
             $display("x%0d = %h", i, uut.u_regfile.regs[i]);
        end

        // --- Dump data memory values (Simple loop for MEM[0]-MEM[124]) ---
        // ** Adjust path uut.u_dmem.mem if needed **
        $display("=== Data Memory Contents (MEM[0]-MEM[124]) ==="); // Displaying first 32 words
        for (i = 0; i < 32; i = i + 1) begin
             $display("MEM[%0d] = %h", i*4, uut.u_dmem.mem[i]); // Display byte address and word value
        end
        $display("..."); // Indicate that only part of memory is shown

        $finish; // End simulation after dumping
    end

    // Optional: Monitor signals during simulation (Uncomment if needed)
    // initial begin
    //      #70; // Wait until after reset release
    //      $monitor("t=%0t PC=%h FDPC=%h Inst1=%s Inst2=%s Stall=%b FFTBusy=%b FFTState=%d WB1=%h WB2=%h",
    //               $time, uut.pc_reg, uut.fd_pc, uut.inst1_name_reg, uut.inst2_name_reg,
    //               uut.pipeline_stall, uut.cpu_fft_busy, uut.u_fft_interface.fft_state, uut.wb_wdata1, uut.wb_wdata2);
    // end

endmodule




*/