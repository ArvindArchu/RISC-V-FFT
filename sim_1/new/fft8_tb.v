`timescale 1ns / 1ps

module fft8_tb;

    parameter DATA_W = 16;
    reg clk, rst_n, start;
    wire done;

    // Packed I/O
    reg signed [(DATA_W*8)-1:0] in_re_flat;
    reg signed [(DATA_W*8)-1:0] in_im_flat;
    wire signed [(DATA_W*8)-1:0] out_re_flat;
    wire signed [(DATA_W*8)-1:0] out_im_flat;

    // Instantiate FFT core
    fft8_core #(.DATA_W(DATA_W)) uut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .in_re_flat(in_re_flat),
        .in_im_flat(in_im_flat),
        .out_re_flat(out_re_flat),
        .out_im_flat(out_im_flat),
        .done(done)
    );

    always #5 clk = ~clk;

    integer i;
    reg signed [DATA_W-1:0] tmp_in_re [0:7];
    reg signed [DATA_W-1:0] tmp_in_im [0:7];
    reg signed [DATA_W-1:0] tmp_out_re [0:7];
    reg signed [DATA_W-1:0] tmp_out_im [0:7];

    initial begin
        clk = 0;
        rst_n = 0;
        start = 0;
        in_re_flat = 0;
        in_im_flat = 0;
        #20 rst_n = 1;

        // Create a real sinusoid spanning 1 cycle over 8 samples.
        for (i = 0; i < 8; i = i + 1) begin
            tmp_in_re[i] = $rtoi(32767.0 * $cos(2.0 * 3.141592653589793 * i / 8.0));
            tmp_in_im[i] = 0;
        end

        // Pack into flattened signals (order: re7..re0)
        in_re_flat = { tmp_in_re[7], tmp_in_re[6], tmp_in_re[5], tmp_in_re[4],
                       tmp_in_re[3], tmp_in_re[2], tmp_in_re[1], tmp_in_re[0] };
        in_im_flat = { tmp_in_im[7], tmp_in_im[6], tmp_in_im[5], tmp_in_im[4],
                       tmp_in_im[3], tmp_in_im[2], tmp_in_im[1], tmp_in_im[0] };

        $display("Input (Q1.15):");
        for (i = 0; i < 8; i = i + 1) begin
            $display("x[%0d] = %0d", i, tmp_in_re[i]);
        end

        // Start FFT
        #10 start = 1;
        #10 start = 0;

        // Wait for done
        wait(done);

        // Unpack outputs
        for (i = 0; i < 8; i = i + 1) begin
            tmp_out_re[i] = out_re_flat[(i+1)*DATA_W-1 -: DATA_W];
            tmp_out_im[i] = out_im_flat[(i+1)*DATA_W-1 -: DATA_W];
        end

        #10;
        $display("\nOutput (Q1.15):");
        for (i = 0; i < 8; i = i + 1) begin
            $display("k=%0d: Re=%0d  Im=%0d", i, tmp_out_re[i], tmp_out_im[i]);
        end

        $finish;
    end

endmodule
