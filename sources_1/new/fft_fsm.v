`timescale 1ns / 1ps

module fft_fsm #(
    parameter DATA_W = 16,
    parameter CPU_DATA_W = 32
)(
    input  wire                  clk,
    input  wire                  reset,
    input  wire                  start,
    input  wire [CPU_DATA_W-1:0] start_addr,
    input  wire [CPU_DATA_W-1:0] end_addr,
    output reg                   busy,
    output reg                   done,
    output reg  [CPU_DATA_W-1:0] mem_addr,
    output reg                   mem_we,
    output reg  [CPU_DATA_W-1:0] mem_wdata,
    input  wire [CPU_DATA_W-1:0] mem_rdata,
    output reg                   fft_start,
    input  wire                  fft_done,
    output reg  signed [DATA_W*8-1:0] in_re_flat,
    output reg  signed [DATA_W*8-1:0] in_im_flat,
    input  wire signed [DATA_W*8-1:0] out_re_flat,
    input  wire signed [DATA_W*8-1:0] out_im_flat
);

    localparam S_IDLE       = 3'd0;
    localparam S_READ_ADDR  = 3'd1;
    localparam S_READ_DATA  = 3'd2;
    localparam S_START_FFT  = 3'd3;
    localparam S_WAIT_FFT   = 3'd4;
    localparam S_WRITE_MEM  = 3'd5;
    localparam S_DONE       = 3'd6;

    reg [2:0] state;
    reg [2:0] read_count, write_count;
    reg [CPU_DATA_W-1:0] base_addr;
    reg start_prev;

    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            state <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            fft_start <= 1'b0;
            read_count <= 3'd0;
            write_count <= 3'd0;
            base_addr <= 32'd0;
            mem_we <= 1'b0;
            mem_addr <= 32'd0;
            mem_wdata <= 32'd0;
            in_re_flat <= 0;
            in_im_flat <= 0;
            start_prev <= 1'b0;
        end else begin
            // Defaults
            done <= 1'b0;
            mem_we <= 1'b0;
            fft_start <= 1'b0;
            start_prev <= start;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start && !start_prev) begin
                        busy <= 1'b1;
                        base_addr <= start_addr;
                        read_count <= 3'd0;
                        state <= S_READ_ADDR;
                        $display("[%0t] FSM: START rising edge - reading from %h", $time, start_addr);
                    end
                end

                S_READ_ADDR: begin
                    mem_addr <= base_addr + (read_count << 2);
                    state <= S_READ_DATA;
                end

                S_READ_DATA: begin
                    in_re_flat[read_count*DATA_W +: DATA_W] <= mem_rdata[DATA_W-1:0];
                    in_im_flat[read_count*DATA_W +: DATA_W] <= mem_rdata[CPU_DATA_W-1:DATA_W];
                    
                    $display("[%0t] FSM: READ[%0d] = %h", $time, read_count, mem_rdata);
                    
                    read_count <= read_count + 1;
                    
                    if (read_count == 3'd7) begin
                        state <= S_START_FFT;
                        $display("[%0t] FSM: All samples read", $time);
                    end else begin
                        state <= S_READ_ADDR;
                    end
                end

                S_START_FFT: begin
                    fft_start <= 1'b1;
                    state <= S_WAIT_FFT;
                    $display("[%0t] FSM: Pulsing fft_start", $time);
                end

                S_WAIT_FFT: begin
                    $display("[%0t] FSM: Waiting... fft_done=%b", $time, fft_done);
                    if (fft_done) begin
                        write_count <= 3'd0;
                        state <= S_WRITE_MEM;
                        $display("[%0t] FSM: fft_done=1 detected! Starting writeback", $time);
                    end
                end

                S_WRITE_MEM: begin
                    mem_addr <= base_addr + (write_count << 2);
                    mem_wdata <= {out_im_flat[write_count*DATA_W +: DATA_W], 
                                  out_re_flat[write_count*DATA_W +: DATA_W]};
                    mem_we <= 1'b1;
                    
                    $display("[%0t] FSM: WRITE[%0d] addr=%h data=%h", 
                             $time, write_count, base_addr + (write_count << 2), mem_wdata);
                    
                    write_count <= write_count + 1;
                    
                    if (write_count == 3'd7) begin
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    mem_we <= 1'b0;
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                    $display("[%0t] FSM: DONE", $time);
                end
            endcase
        end
    end

endmodule


//this is claude
/*
`timescale 1ns / 1ps

// FSM controller for 8-point FFT accelerator
module fft_fsm #(
    parameter DATA_W = 16,
    parameter CPU_DATA_W = 32
)(
    input  wire                  clk,
    input  wire                  reset,

    // Control from CPU
    input  wire                  start,       // from control unit
    input  wire [CPU_DATA_W-1:0] start_addr,  // rs1
    input  wire [CPU_DATA_W-1:0] end_addr,    // rs2
    output reg                   busy,
    output reg                   done,

    // Memory interface
    output reg  [CPU_DATA_W-1:0] mem_addr,
    output reg                   mem_we,
    output reg  [CPU_DATA_W-1:0] mem_wdata,
    input  wire [CPU_DATA_W-1:0] mem_rdata,

    // FFT core handshake
    output reg                   fft_start,
    input  wire                  fft_done,
    output reg  signed [DATA_W*8-1:0] in_re_flat,
    output reg  signed [DATA_W*8-1:0] in_im_flat,
    input  wire signed [DATA_W*8-1:0] out_re_flat,
    input  wire signed [DATA_W*8-1:0] out_im_flat
);

    // FSM states
    localparam S_IDLE       = 3'd0;
    localparam S_READ_ADDR  = 3'd1;  // Issue address
    localparam S_READ_DATA  = 3'd2;  // Capture data (1 cycle later)
    localparam S_START_FFT  = 3'd3;
    localparam S_WAIT_FFT   = 3'd4;
    localparam S_WRITE_MEM  = 3'd5;
    localparam S_DONE       = 3'd6;

    reg [2:0] state;
    reg [2:0] read_count;   // Counts 0-7 for reading
    reg [2:0] write_count;  // Counts 0-7 for writing
    reg [CPU_DATA_W-1:0] base_addr;

    integer i;
    
    // Main FSM
    always @(posedge clk) begin
        if (reset) begin
            state <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            fft_start <= 1'b0;
            read_count <= 3'd0;
            write_count <= 3'd0;
            base_addr <= 32'd0;
            mem_we <= 1'b0;
            mem_addr <= 32'd0;
            mem_wdata <= 32'd0;
            in_re_flat <= {DATA_W*8{1'b0}};
            in_im_flat <= {DATA_W*8{1'b0}};
        end else begin
            // Default outputs
            done <= 1'b0;
            mem_we <= 1'b0;
            fft_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        base_addr <= start_addr;
                        read_count <= 3'd0;
                        state <= S_READ_ADDR;
                        $display("[%0t] FSM: START - Reading from addr %h", $time, start_addr);
                    end
                end

                S_READ_ADDR: begin
                    // Issue read address
                    mem_addr <= base_addr + (read_count << 2);  // read_count * 4
                    state <= S_READ_DATA;
                    $display("[%0t] FSM: READ_ADDR[%0d] = %h", $time, read_count, base_addr + (read_count << 2));
                end

                S_READ_DATA: begin
                    // Capture data that arrived from previous cycle's address
                    in_re_flat[read_count*DATA_W +: DATA_W] <= mem_rdata[DATA_W-1:0];
                    in_im_flat[read_count*DATA_W +: DATA_W] <= mem_rdata[CPU_DATA_W-1:DATA_W];
                    
                    $display("[%0t] FSM: READ_DATA[%0d] = %h (re=%h, im=%h)", 
                             $time, read_count, mem_rdata,
                             mem_rdata[DATA_W-1:0], mem_rdata[CPU_DATA_W-1:DATA_W]);
                    
                    read_count <= read_count + 1;
                    
                    if (read_count == 3'd7) begin
                        // All 8 samples read
                        state <= S_START_FFT;
                        $display("[%0t] FSM: All samples read, starting FFT", $time);
                    end else begin
                        // Read next sample
                        state <= S_READ_ADDR;
                    end
                end

                S_START_FFT: begin
                    fft_start <= 1'b1;  // Pulse for one cycle
                    state <= S_WAIT_FFT;
                    $display("[%0t] FSM: START_FFT pulse issued", $time);
                end

                S_WAIT_FFT: begin
                    // fft_start already deasserted by default assignment
                    if (fft_done) begin
                        write_count <= 3'd0;
                        state <= S_WRITE_MEM;
                        $display("[%0t] FSM: FFT done, starting write-back", $time);
                    end
                end

                S_WRITE_MEM: begin
                    // Write one sample per cycle
                    mem_addr <= base_addr + (write_count << 2);  // write_count * 4
                    mem_wdata <= {out_im_flat[write_count*DATA_W +: DATA_W], 
                                  out_re_flat[write_count*DATA_W +: DATA_W]};
                    mem_we <= 1'b1;
                    
                    $display("[%0t] FSM: WRITE[%0d] to addr %h = %h (re=%h, im=%h)", 
                             $time, write_count, base_addr + (write_count << 2), 
                             {out_im_flat[write_count*DATA_W +: DATA_W], out_re_flat[write_count*DATA_W +: DATA_W]},
                             out_re_flat[write_count*DATA_W +: DATA_W],
                             out_im_flat[write_count*DATA_W +: DATA_W]);
                    
                    write_count <= write_count + 1;
                    
                    if (write_count == 3'd7) begin
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    mem_we <= 1'b0;
                    busy <= 1'b0;
                    done <= 1'b1;  // Pulse done for 1 cycle
                    state <= S_IDLE;
                    $display("[%0t] FSM: DONE - FFT operation complete", $time);
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

//this is gpt
`timescale 1ns / 1ps

// FSM controller for 8-point FFT accelerator
module fft_fsm #(
    parameter DATA_W = 16,
    parameter CPU_DATA_W = 32
)(
    input  wire                  clk,
    input  wire                  reset,

    // Control from CPU
    input  wire                  start,       // from control unit
    input  wire [CPU_DATA_W-1:0] start_addr,  // rs1
    input  wire [CPU_DATA_W-1:0] end_addr,    // rs2
    output reg                   busy,
    output reg                   done,

    // Memory interface
    output reg  [CPU_DATA_W-1:0] mem_addr,
    output reg                   mem_we,
    output reg  [CPU_DATA_W-1:0] mem_wdata,
    input  wire [CPU_DATA_W-1:0] mem_rdata,

    // FFT core handshake
    output reg                   fft_start,
    input  wire                  fft_done,
    output reg  signed [DATA_W*8-1:0] in_re_flat,
    output reg  signed [DATA_W*8-1:0] in_im_flat,
    input  wire signed [DATA_W*8-1:0] out_re_flat,
    input  wire signed [DATA_W*8-1:0] out_im_flat
);

    // FSM states
    localparam S_IDLE      = 3'd0;
    localparam S_READ_MEM  = 3'd1;
    localparam S_START_FFT = 3'd2;
    localparam S_WAIT_FFT  = 3'd3;
    localparam S_WRITE_MEM = 3'd4;
    localparam S_DONE      = 3'd5;

    reg [2:0] state, next_state;
    reg [2:0] index;
    reg [CPU_DATA_W-1:0] addr_ptr;


    // signals used:
    reg fft_start_q;   // local 1-cycle pulse generator
    reg [2:0] load_idx;
    reg [2:0] store_idx;
    reg waiting_done_q;


    // Sequential state register
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // Next state logic
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:      if (start)         next_state = S_READ_MEM;
            S_READ_MEM:  if (index == 3'd7) next_state = S_START_FFT;
            S_START_FFT:                    next_state = S_WAIT_FFT;
            S_WAIT_FFT:  if (fft_done)      next_state = S_WRITE_MEM;
            S_WRITE_MEM: if (index == 3'd7) next_state = S_DONE;
            S_DONE:                         next_state = S_IDLE;
            default:                        next_state = S_IDLE;
        endcase
    end

    // Main FSM
    integer i;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= S_IDLE;
            busy <= 1'b0;
            done <= 1'b0;
            fft_start <= 1'b0;
            fft_start_q <= 1'b0;
            load_idx <= 3'd0;
            store_idx <= 3'd0;
            addr_ptr <= 32'd0;
            mem_we <= 1'b0;
        end else begin
            // default outputs
            done <= 1'b0;
            mem_we <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    fft_start <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        addr_ptr <= start_addr;
                        load_idx <= 3'd0;
                        state <= S_READ_MEM;
                    end
                end

                S_READ_MEM: begin
                    // Issue read for one sample each cycle
                    mem_addr <= addr_ptr;
                    // after memory read (mem_rdata valid next cycle) capture it
                    // So use a pipeline: set mem_addr now, capture mem_rdata next cycle
                    // Use a small two-cycle handshake approach:
                    // NOTE: if Data_MEM gives read data combinationally, you can capture immediately
                    in_re_flat[load_idx*DATA_W +: DATA_W] <= mem_rdata[15:0];
                    in_im_flat[load_idx*DATA_W +: DATA_W] <= mem_rdata[31:16];
                    addr_ptr <= addr_ptr + 4;
                    load_idx <= load_idx + 1;
                    if (load_idx == 3'd7) begin
                        state <= S_START_FFT;
                    end
                end

                S_START_FFT: begin
                    // Pulse fft_start for exactly one cycle
                    fft_start <= 1'b1;
                    waiting_done_q <= 1'b1; // indicate waiting for done next
                    state <= S_WAIT_FFT;
                end

                S_WAIT_FFT: begin
                    // Deassert the start pulse immediately next cycle
                    fft_start <= 1'b0;
                    if (fft_done) begin
                        // latch outputs (they are stable when fft_done asserted per core contract)
                        store_idx <= 3'd0;
                        addr_ptr <= start_addr;
                        state <= S_WRITE_MEM;
                    end
                end

                S_WRITE_MEM: begin
                    // Write back one sample per cycle
                    if (mem_we) $display("[%0t] FSM WRITE mem_addr=%0h mem_wdata=%h idx=%0d", $time, mem_addr, mem_wdata, store_idx);
                    mem_addr <= addr_ptr;
                    mem_wdata <= { out_im_flat[store_idx*DATA_W +: DATA_W],
                                out_re_flat[store_idx*DATA_W +: DATA_W] };
                    mem_we <= 1'b1;
                    addr_ptr <= addr_ptr + 4;
                    store_idx <= store_idx + 1;
                    if (store_idx == 3'd7) begin
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    mem_we <= 1'b0;
                    busy <= 1'b0;
                    done <= 1'b1; // pulse done for 1 cycle
                    state <= S_IDLE;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (!reset) begin
            if (state == S_START_FFT)
                $display("[%0t] FSM: START_FFT pulse issued", $time);
            if (state == S_WAIT_FFT && fft_done)
                $display("[%0t] FSM: got fft_done", $time);
            if (state == S_WRITE_MEM)
                $display("[%0t] FSM: writing mem[%0h] <= %h", $time, mem_addr, mem_wdata);
            if (state == S_DONE)
                $display("[%0t] FSM: FFT finished, clearing busy", $time);
        end
    end
endmodule
*/