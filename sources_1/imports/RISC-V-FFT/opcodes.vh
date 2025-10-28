`ifndef OPCODES_VH
`define OPCODES_VH

`define OP_RTYPE  7'b0110011
`define OP_ITYPE  7'b0010011
`define OP_LOAD   7'b0000011
`define OP_STORE  7'b0100011
`define OP_BRANCH 7'b1100011
`define OP_JAL    7'b1101111
`define OP_JALR   7'b1100111
`define OP_LUI    7'b0110111
`define OP_AUIPC  7'b0010111
`define OP_CUSTOM0 7'b0001011 // Opcode for custom instructions (like FFT)

`endif
