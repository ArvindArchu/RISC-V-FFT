`timescale 1ns / 1ps

// 8-point FFT core implemented as direct DFT (fixed-point Q1.15 inputs).
// - Inputs: in_re_flat / in_im_flat are packed MSB-first as {x7, x6, ..., x0}
// - Output: out_re_flat / out_im_flat are packed same order {X7, X6, ..., X0}
// - Output is normalized by N (divide by 8) and saturated to 16-bit signed.
module fft8_core #(
    parameter DATA_W = 16
)(
    input  wire                        clk,
    input  wire                        reset,
    input  wire                        start,
    output reg                         done,
    input  wire signed [DATA_W*8-1:0]  in_re_flat,
    input  wire signed [DATA_W*8-1:0]  in_im_flat,
    output reg signed [DATA_W*8-1:0]   out_re_flat,
    output reg signed [DATA_W*8-1:0]   out_im_flat
);

    // internal buffers
    reg signed [DATA_W-1:0] x_re [0:7];
    reg signed [DATA_W-1:0] x_im [0:7];

    // 32-bit accumulators (enough headroom before normalization)
    reg signed [31:0] sum_re, sum_im;

    // Twiddle LUTs (Q1.15)
    reg signed [DATA_W-1:0] cos_lut [0:7];
    reg signed [DATA_W-1:0] sin_lut [0:7];

    initial begin
        cos_lut[0] = 16'sh7FFF; cos_lut[1] = 16'sh5A82; cos_lut[2] = 16'sh0000; cos_lut[3] = 16'shA57E;
        cos_lut[4] = 16'sh8001; cos_lut[5] = 16'shA57E; cos_lut[6] = 16'sh0000; cos_lut[7] = 16'sh5A82;
        sin_lut[0] = 16'sh0000; sin_lut[1] = 16'shA57E; sin_lut[2] = 16'sh8001; sin_lut[3] = 16'shA57E;
        sin_lut[4] = 16'sh0000; sin_lut[5] = 16'sh5A82; sin_lut[6] = 16'sh7FFF; sin_lut[7] = 16'sh5A82;
    end

    reg [3:0] state;
    reg [2:0] k, n;
    reg [5:0] twiddle_idx;
    reg signed [31:0] prod_re, prod_im;
    reg started_q;

    localparam IDLE      = 4'd0;
    localparam LOAD      = 4'd1;
    localparam INIT_BIN  = 4'd2;
    localparam ACCUM     = 4'd3;
    localparam STORE_BIN = 4'd4;
    localparam FINISH    = 4'd5;
    reg signed [31:0] norm_re;
    reg signed [31:0] norm_im;

    integer i;

    // helper: saturate 32-bit value (already normalized) to signed 16-bit
    function signed [DATA_W-1:0] sat16;
        input signed [31:0] val;
        begin
            if (val >  32'sd32767) sat16 = 16'sd32767;
            else if (val < -32'sd32768) sat16 = -16'sd32768;
            else sat16 = val[15:0];
        end
    endfunction

    always @(posedge clk) begin
        if (reset) begin
            done        <= 1'b0;
            state       <= IDLE;
            started_q   <= 1'b0;
            k           <= 0;
            n           <= 0;
            sum_re      <= 0;
            sum_im      <= 0;
            out_re_flat <= 0;
            out_im_flat <= 0;
            for (i = 0; i < 8; i = i + 1) begin
                x_re[i] <= 0;
                x_im[i] <= 0;
            end
        end else begin
            case (state)
                IDLE: begin
                    done      <= 1'b0;
                    started_q <= 1'b0;
                    if (start && !started_q) begin
                        started_q <= 1'b1;
                        state <= LOAD;
                        $display("[%0t] FFT core: START detected", $time);
                    end
                end

                LOAD: begin
                    // UNPACK: assume in_flat packed MSB-first as {x7,x6,...,x0}
                    // so x_re[0] = least-significant chunk -> use (7-0) mapping
                    for (i = 0; i < 8; i = i + 1) begin
                        x_re[i] <= in_re_flat[((7 - i)*DATA_W) +: DATA_W];
                        x_im[i] <= in_im_flat[((7 - i)*DATA_W) +: DATA_W];
                    end
                    k <= 0;
                    state <= INIT_BIN;
                    $display("[%0t] FFT core: Inputs loaded", $time);
                end

                INIT_BIN: begin
                    sum_re <= 32'sd0;
                    sum_im <= 32'sd0;
                    n <= 0;
                    state <= ACCUM;
                end

                ACCUM: begin
                    // twiddle_idx = (k * n) % 8  -- mask low 3 bits
                    twiddle_idx = (k * n) & 3'b111;

                    // multiply (x_re + j x_im) * (cos - j sin)  (cos & sin are Q1.15)
                    prod_re = ($signed(x_re[n]) * $signed(cos_lut[twiddle_idx])) -
                              ($signed(x_im[n]) * $signed(sin_lut[twiddle_idx]));
                    prod_im = ($signed(x_im[n]) * $signed(cos_lut[twiddle_idx])) +
                              ($signed(x_re[n]) * $signed(sin_lut[twiddle_idx]));

                    // scale down product by 15 (because cos/sin are Q1.15)
                    // accumulate in 32-bit accumulator
                    sum_re <= sum_re + (prod_re >>> 15);
                    sum_im <= sum_im + (prod_im >>> 15);

                    if (n == 3'd7) begin
                        state <= STORE_BIN;
                    end else begin
                        n <= n + 1;
                    end
                end

                STORE_BIN: begin
                    // NORMALIZE by N (8) to keep output in Q1.15 range
                    norm_re = sum_re >>> 3; // divide by 8
                    norm_im = sum_im >>> 3;

                    // store back into out_flat. Keep same MSB-first packing.
                    out_re_flat[((7 - k)*DATA_W) +: DATA_W] <= sat16(norm_re);
                    out_im_flat[((7 - k)*DATA_W) +: DATA_W] <= sat16(norm_im);

                    $display("[%0t] FFT: X[%0d] = %d (0x%h) + j %d (0x%h)",
                            $time, k, norm_re, sat16(norm_re), norm_im, sat16(norm_im));

                    if (k == 3'd7) begin
                        state <= FINISH;
                        $display("[%0t] FFT core: All bins computed", $time);
                    end else begin
                        k <= k + 1;
                        state <= INIT_BIN;
                    end
                end

                FINISH: begin
                    done <= 1'b1;
                    state <= IDLE; // next cycle done will be cleared by IDLE
                    $display("[%0t] FFT core: FINISH state - asserting done=1", $time);
                end

                default: begin
                    state <= IDLE;
                    done <= 1'b0;
                end
            endcase
        end
    end

    // debug monitor
    always @(posedge clk) begin
        if (!reset) begin
            $display("[%0t] FFT core: state=%d done=%b", $time, state, done);
        end
    end

endmodule

/* //impulse works here
`timescale 1ns / 1ps

// 8-point FFT core using simple DFT
module fft8_core #(
    parameter DATA_W = 16
)(
    input  wire                   clk,
    input  wire                   reset,
    input  wire                   start,
    output reg                    done,
    input  wire signed [DATA_W*8-1:0] in_re_flat,
    input  wire signed [DATA_W*8-1:0] in_im_flat,
    output reg  signed [DATA_W*8-1:0] out_re_flat,
    output reg  signed [DATA_W*8-1:0] out_im_flat
);

    reg signed [DATA_W-1:0] x_re [0:7];
    reg signed [DATA_W-1:0] x_im [0:7];
    reg signed [31:0] sum_re, sum_im;
    
    reg signed [DATA_W-1:0] cos_lut [0:7];
    reg signed [DATA_W-1:0] sin_lut [0:7];
    
    initial begin
        cos_lut[0] = 16'sh7FFF; cos_lut[1] = 16'sh5A82; cos_lut[2] = 16'sh0000; cos_lut[3] = 16'shA57E;
        cos_lut[4] = 16'sh8001; cos_lut[5] = 16'shA57E; cos_lut[6] = 16'sh0000; cos_lut[7] = 16'sh5A82;
        sin_lut[0] = 16'sh0000; sin_lut[1] = 16'shA57E; sin_lut[2] = 16'sh8001; sin_lut[3] = 16'shA57E;
        sin_lut[4] = 16'sh0000; sin_lut[5] = 16'sh5A82; sin_lut[6] = 16'sh7FFF; sin_lut[7] = 16'sh5A82;
    end
    
    reg [3:0] state;
    reg [2:0] k, n;
    reg [5:0] twiddle_idx;
    reg signed [31:0] prod_re, prod_im;
    reg started_q;
    
    localparam IDLE = 4'd0;
    localparam LOAD = 4'd1;
    localparam INIT_BIN = 4'd2;
    localparam ACCUM = 4'd3;
    localparam STORE_BIN = 4'd4;
    localparam FINISH = 4'd5;  // Dedicated state for asserting done
    
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            done <= 1'b0;
            state <= IDLE;
            started_q <= 1'b0;
            k <= 0;
            n <= 0;
            sum_re <= 0;
            sum_im <= 0;
            out_re_flat <= 0;
            out_im_flat <= 0;
            for (i = 0; i < 8; i = i + 1) begin
                x_re[i] <= 0;
                x_im[i] <= 0;
            end
        end else begin
            // State machine
            case (state)
                IDLE: begin
                    done <= 1'b0;  // Clear done in IDLE
                    started_q <= 1'b0;
                    if (start && !started_q) begin
                        started_q <= 1'b1;
                        state <= LOAD;
                        $display("[%0t] FFT core: START detected", $time);
                    end
                end
                
                LOAD: begin
                    for (i = 0; i < 8; i = i + 1) begin
                        x_re[i] <= in_re_flat[i*DATA_W +: DATA_W];
                        x_im[i] <= in_im_flat[i*DATA_W +: DATA_W];
                    end
                    k <= 0;
                    state <= INIT_BIN;
                    $display("[%0t] FFT core: Inputs loaded", $time);
                end
                
                INIT_BIN: begin
                    sum_re <= 0;
                    sum_im <= 0;
                    n <= 0;
                    state <= ACCUM;
                end
                
                ACCUM: begin
                    twiddle_idx = (k * n) & 6'd7;
                    prod_re = ($signed(x_re[n]) * $signed(cos_lut[twiddle_idx])) + 
                              ($signed(x_im[n]) * $signed(sin_lut[twiddle_idx]));
                    prod_im = ($signed(x_im[n]) * $signed(cos_lut[twiddle_idx])) - 
                              ($signed(x_re[n]) * $signed(sin_lut[twiddle_idx]));
                    
                    sum_re <= sum_re + (prod_re >>> 15);
                    sum_im <= sum_im + (prod_im >>> 15);
                    
                    if (n == 7) begin
                        state <= STORE_BIN;
                    end else begin
                        n <= n + 1;
                    end
                end
                
                STORE_BIN: begin
                    out_re_flat[k*DATA_W +: DATA_W] <= sum_re[DATA_W-1:0];
                    out_im_flat[k*DATA_W +: DATA_W] <= sum_im[DATA_W-1:0];
                    $display("[%0t] FFT: X[%0d] = %h + j%h", $time, k, sum_re[15:0], sum_im[15:0]);
                    
                    if (k == 7) begin
                        state <= FINISH;  // Go to FINISH state
                        $display("[%0t] FFT core: All bins computed", $time);
                    end else begin
                        k <= k + 1;
                        state <= INIT_BIN;
                    end
                end
                
                FINISH: begin
                    // Dedicated state: ONLY assert done, do nothing else
                    done <= 1'b1;
                    state <= IDLE;  // Next cycle will clear done
                    $display("[%0t] FFT core: FINISH state - asserting done=1", $time);
                end
                
                default: begin
                    state <= IDLE;
                    done <= 1'b0;
                end
            endcase
        end
    end
    
    // Debug monitor
    always @(posedge clk) begin
        if (!reset) begin
            $display("[%0t] FFT core: state=%d done=%b", $time, state, done);
        end
    end

endmodule
*/
/*
`timescale 1ns / 1ps

// 8-point FFT core (pipelined, fixed-point Q1.15)
module fft8_core #(
    parameter DATA_W = 16
)(
    input  wire                   clk,
    input  wire                   reset,
    input  wire                   start,
    output reg                    done,
    // Flattened buses
    input  wire signed [DATA_W*8-1:0] in_re_flat,
    input  wire signed [DATA_W*8-1:0] in_im_flat,
    output reg  signed [DATA_W*8-1:0] out_re_flat,
    output reg  signed [DATA_W*8-1:0] out_im_flat
);

    // Local arrays for computation
    reg signed [DATA_W-1:0] x_re [0:7];
    reg signed [DATA_W-1:0] x_im [0:7];
    reg signed [DATA_W-1:0] y_re [0:7];
    reg signed [DATA_W-1:0] y_im [0:7];
    
    reg [2:0] compute_stage;
    reg computing;
    
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            done <= 1'b0;
            computing <= 1'b0;
            compute_stage <= 3'd0;
            out_re_flat <= {DATA_W*8{1'b0}};
            out_im_flat <= {DATA_W*8{1'b0}};
            for (i = 0; i < 8; i = i + 1) begin
                x_re[i] <= 0;
                x_im[i] <= 0;
                y_re[i] <= 0;
                y_im[i] <= 0;
            end
        end else begin
            // Default: keep done high until start is deasserted
            if (start) begin
                done <= 1'b0;  // Clear done when new computation starts
            end
            
            if (start && !computing) begin
                // Latch inputs
                for (i = 0; i < 8; i = i + 1) begin
                    x_re[i] <= in_re_flat[i*DATA_W +: DATA_W];
                    x_im[i] <= in_im_flat[i*DATA_W +: DATA_W];
                end
                computing <= 1'b1;
                compute_stage <= 3'd0;
                done <= 1'b0;
                $display("[%0t] FFT core: Started computation", $time);
                $display("  Inputs: [0]=%h+j%h [1]=%h+j%h [2]=%h+j%h [3]=%h+j%h", 
                         in_re_flat[0*DATA_W +: DATA_W], in_im_flat[0*DATA_W +: DATA_W],
                         in_re_flat[1*DATA_W +: DATA_W], in_im_flat[1*DATA_W +: DATA_W],
                         in_re_flat[2*DATA_W +: DATA_W], in_im_flat[2*DATA_W +: DATA_W],
                         in_re_flat[3*DATA_W +: DATA_W], in_im_flat[3*DATA_W +: DATA_W]);
            end else if (computing) begin
                case (compute_stage)
                    3'd0: begin  // Stage 1: Radix-2 butterflies (4 parallel)
                        // Butterfly 0-4
                        y_re[0] <= x_re[0] + x_re[4];
                        y_im[0] <= x_im[0] + x_im[4];
                        y_re[4] <= x_re[0] - x_re[4];
                        y_im[4] <= x_im[0] - x_im[4];
                        
                        // Butterfly 1-5
                        y_re[1] <= x_re[1] + x_re[5];
                        y_im[1] <= x_im[1] + x_im[5];
                        y_re[5] <= x_re[1] - x_re[5];
                        y_im[5] <= x_im[1] - x_im[5];
                        
                        // Butterfly 2-6
                        y_re[2] <= x_re[2] + x_re[6];
                        y_im[2] <= x_im[2] + x_im[6];
                        y_re[6] <= x_re[2] - x_re[6];
                        y_im[6] <= x_im[2] - x_im[6];
                        
                        // Butterfly 3-7
                        y_re[3] <= x_re[3] + x_re[7];
                        y_im[3] <= x_im[3] + x_im[7];
                        y_re[7] <= x_re[3] - x_re[7];
                        y_im[7] <= x_im[3] - x_im[7];
                        
                        compute_stage <= 3'd1;
                        $display("[%0t] FFT core: Stage 1 complete", $time);
                    end
                    
                    3'd1: begin  // Copy results to x array
                        for (i = 0; i < 8; i = i + 1) begin
                            x_re[i] <= y_re[i];
                            x_im[i] <= y_im[i];
                        end
                        compute_stage <= 3'd2;
                    end
                    
                    3'd2: begin  // Stage 2: Radix-2 butterflies
                        // Butterfly 0-2
                        y_re[0] <= x_re[0] + x_re[2];
                        y_im[0] <= x_im[0] + x_im[2];
                        y_re[2] <= x_re[0] - x_re[2];
                        y_im[2] <= x_im[0] - x_im[2];
                        
                        // Butterfly 1-3
                        y_re[1] <= x_re[1] + x_re[3];
                        y_im[1] <= x_im[1] + x_im[3];
                        y_re[3] <= x_re[1] - x_re[3];
                        y_im[3] <= x_im[1] - x_im[3];
                        
                        // Butterfly 4-6
                        y_re[4] <= x_re[4] + x_re[6];
                        y_im[4] <= x_im[4] + x_im[6];
                        y_re[6] <= x_re[4] - x_re[6];
                        y_im[6] <= x_im[4] - x_im[6];
                        
                        // Butterfly 5-7
                        y_re[5] <= x_re[5] + x_re[7];
                        y_im[5] <= x_im[5] + x_im[7];
                        y_re[7] <= x_re[5] - x_re[7];
                        y_im[7] <= x_im[5] - x_im[7];
                        
                        compute_stage <= 3'd3;
                        $display("[%0t] FFT core: Stage 2 complete", $time);
                    end
                    
                    3'd3: begin  // Copy results to x array
                        for (i = 0; i < 8; i = i + 1) begin
                            x_re[i] <= y_re[i];
                            x_im[i] <= y_im[i];
                        end
                        compute_stage <= 3'd4;
                    end
                    
                    3'd4: begin  // Stage 3: Final butterflies
                        // Butterfly 0-1
                        y_re[0] <= x_re[0] + x_re[1];
                        y_im[0] <= x_im[0] + x_im[1];
                        y_re[1] <= x_re[0] - x_re[1];
                        y_im[1] <= x_im[0] - x_im[1];
                        
                        // Butterfly 2-3
                        y_re[2] <= x_re[2] + x_re[3];
                        y_im[2] <= x_im[2] + x_im[3];
                        y_re[3] <= x_re[2] - x_re[3];
                        y_im[3] <= x_im[2] - x_im[3];
                        
                        // Butterfly 4-5
                        y_re[4] <= x_re[4] + x_re[5];
                        y_im[4] <= x_im[4] + x_im[5];
                        y_re[5] <= x_re[4] - x_re[5];
                        y_im[5] <= x_im[4] - x_im[5];
                        
                        // Butterfly 6-7
                        y_re[6] <= x_re[6] + x_re[7];
                        y_im[6] <= x_im[6] + x_im[7];
                        y_re[7] <= x_re[6] - x_re[7];
                        y_im[7] <= x_im[6] - x_im[7];
                        
                        compute_stage <= 3'd5;
                        $display("[%0t] FFT core: Stage 3 complete", $time);
                    end
                    
                    3'd5: begin  // Pack outputs and finish
                        for (i = 0; i < 8; i = i + 1) begin
                            out_re_flat[i*DATA_W +: DATA_W] <= y_re[i];
                            out_im_flat[i*DATA_W +: DATA_W] <= y_im[i];
                        end
                        
                        done <= 1'b1;  // Assert done
                        computing <= 1'b0;
                        compute_stage <= 3'd0;
                        
                        $display("[%0t] FFT core: Computation DONE", $time);
                        $display("  Outputs: [0]=%h+j%h [1]=%h+j%h [2]=%h+j%h [3]=%h+j%h", 
                                 y_re[0], y_im[0], y_re[1], y_im[1],
                                 y_re[2], y_im[2], y_re[3], y_im[3]);
                    end
                    
                    default: begin
                        compute_stage <= 3'd0;
                        computing <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule

`timescale 1ns / 1ps

// 8-point FFT core (combinational, fixed-point Q1.15)
module fft8_core #(
    parameter DATA_W = 16
)(
    input  wire                   clk,
    input  wire                   reset,
    input  wire                   start,
    output reg                    done,

    // Flattened buses
    input  wire signed [DATA_W*8-1:0] in_re_flat,
    input  wire signed [DATA_W*8-1:0] in_im_flat,
    output reg  signed [DATA_W*8-1:0] out_re_flat,
    output reg  signed [DATA_W*8-1:0] out_im_flat
);

    // Local unpacked copies for readability
    reg signed [DATA_W-1:0] in_re [0:7];
    reg signed [DATA_W-1:0] in_im [0:7];
    reg signed [DATA_W-1:0] out_re [0:7];
    reg signed [DATA_W-1:0] out_im [0:7];

    integer i;

    // Unpack flattened inputs
    always @(*) begin
        for (i = 0; i < 8; i = i + 1) begin
            in_re[i] = in_re_flat[i*DATA_W +: DATA_W];
            in_im[i] = in_im_flat[i*DATA_W +: DATA_W];
        end
    end

    reg started_q;
    // FFT computation (simple blocking version)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            done <= 1'b0;
            started_q <= 1'b0;
            out_re_flat <= {DATA_W*8{1'b0}};
            out_im_flat <= {DATA_W*8{1'b0}};
        end else begin
            done <= 1'b0; // default: done pulse cleared (1-cycle)
            if (start && !started_q) begin
                // latch inputs into local regs (synchronous)
                for (i = 0; i < 8; i = i + 1) begin
                    in_re[i] <= in_re_flat[i*DATA_W +: DATA_W];
                    in_im[i] <= in_im_flat[i*DATA_W +: DATA_W];
                end
                started_q <= 1'b1;
                // you may set an internal compute counter if you simulate latency
                // For now, compute in next cycle(s)
            end else if (started_q) begin
                // perform computation here (single-cycle or multi-cycle)
                // After computation completed, write outputs and pulse done
                // Example: compute immediately (replace with your verified algorithm)
                for (i = 0; i < 8; i = i + 1) begin
                    out_re[i] <=  in_re[i]; // placeholder
                    out_im[i] <= in_im[i];
                end
                // pack outputs
                for (i = 0; i < 8; i = i + 1) begin
                    out_re_flat[i*DATA_W +: DATA_W] <= out_re[i];
                    out_im_flat[i*DATA_W +: DATA_W] <= out_im[i];
                end
                done <= 1'b1;       // 1-cycle pulse
                started_q <= 1'b0;  // ready for next start
            end
        end
    end

    always @(posedge clk) begin
        if (done)
            $display("[%0t] FFT core done, outputs valid", $time);
    end

endmodule
*/