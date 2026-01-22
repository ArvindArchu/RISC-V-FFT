module scale_by_2 (
    input  signed [17:0] inr,
    input  signed [17:0] ini,
    output signed [15:0] outr,
    output signed [15:0] outi
);
    assign outr = (inr + 18'sd1) >>> 1;
    assign outi = (ini + 18'sd1) >>> 1;
endmodule