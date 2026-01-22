module bfly_radix2_q214 (
    input  signed [15:0] ar, ai,
    input  signed [15:0] br, bi,
    input  signed [15:0] wr, wi,
    output signed [17:0] xr, xi,
    output signed [17:0] yr, yi
);

    //multiply
    wire signed [31:0] p1 = br * wr;
    wire signed [31:0] p2 = bi * wi;
    wire signed [31:0] p3 = br * wi;
    wire signed [31:0] p4 = bi * wr;

    // Back to Q2.14
    wire signed [17:0] bw_r = (p1 - p2) >>> 15;
    wire signed [17:0] bw_i = (p3 + p4) >>> 15;
  
    assign xr = ar + bw_r;
    assign xi = ai + bw_i;
    assign yr = ar - bw_r;
    assign yi = ai - bw_i;

endmodule