module twiddle_rom_64 (
    input  [4:0] addr,
    output reg signed [15:0] wr,
    output reg signed [15:0] wi
);
    // Twiddle factors W_N^k = e^(-j*2*pi*k/64)
    // Q1.15
    always @(*) begin
        case (addr)
            5'd0:  begin wr = 16'sd32767;  wi = 16'sd0;      end // W0
            5'd1:  begin wr = 16'sd32610;  wi = -16'sd3212;  end // W1
            5'd2:  begin wr = 16'sd32138;  wi = -16'sd6393;  end // W2
            5'd3:  begin wr = 16'sd31357;  wi = -16'sd9512;  end // W3
            5'd4:  begin wr = 16'sd30273;  wi = -16'sd12539; end // W4
            5'd5:  begin wr = 16'sd28898;  wi = -16'sd15447; end // W5
            5'd6:  begin wr = 16'sd27245;  wi = -16'sd18204; end // W6
            5'd7:  begin wr = 16'sd25330;  wi = -16'sd20787; end // W7
            5'd8:  begin wr = 16'sd23170;  wi = -16'sd23170; end // W8
            5'd9:  begin wr = 16'sd20787;  wi = -16'sd25330; end // W9
            5'd10: begin wr = 16'sd18204;  wi = -16'sd27245; end // W10
            5'd11: begin wr = 16'sd15447;  wi = -16'sd28898; end // W11
            5'd12: begin wr = 16'sd12539;  wi = -16'sd30273; end // W12
            5'd13: begin wr = 16'sd9512;   wi = -16'sd31357; end // W13
            5'd14: begin wr = 16'sd6393;   wi = -16'sd32138; end // W14
            5'd15: begin wr = 16'sd3212;   wi = -16'sd32610; end // W15
            5'd16: begin wr = 16'sd0;      wi = -16'sd32767; end // W16
            5'd17: begin wr = -16'sd3212;  wi = -16'sd32610; end // W17
            5'd18: begin wr = -16'sd6393;  wi = -16'sd32138; end // W18
            5'd19: begin wr = -16'sd9512;  wi = -16'sd31357; end // W19
            5'd20: begin wr = -16'sd12539; wi = -16'sd30273; end // W20
            5'd21: begin wr = -16'sd15447; wi = -16'sd28898; end // W21
            5'd22: begin wr = -16'sd18204; wi = -16'sd27245; end // W22
            5'd23: begin wr = -16'sd20787; wi = -16'sd25330; end // W23
            5'd24: begin wr = -16'sd23170; wi = -16'sd23170; end // W24
            5'd25: begin wr = -16'sd25330; wi = -16'sd20787; end // W25
            5'd26: begin wr = -16'sd27245; wi = -16'sd18204; end // W26
            5'd27: begin wr = -16'sd28898; wi = -16'sd15447; end // W27
            5'd28: begin wr = -16'sd30273; wi = -16'sd12539; end // W28
            5'd29: begin wr = -16'sd31357; wi = -16'sd9512;  end // W29
            5'd30: begin wr = -16'sd32138; wi = -16'sd6393;  end // W30
            5'd31: begin wr = -16'sd32610; wi = -16'sd3212;  end // W31
        endcase
    end
endmodule