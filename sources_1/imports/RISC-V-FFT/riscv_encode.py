# Minimal RISC-V instruction encoder for your Verilog CPU
# Supports: ADDI, JAL, JALR, BEQ, BNE
# Prints binary/hex encodings for immediate use in your instruction memory

def tobin(val, bits):
    """Return val as two's complement binary string of length bits."""
    if val < 0:
        val = (1 << bits) + val
    return format(val & ((1 << bits) - 1), f'0{bits}b')

# -------------------------
# Base opcodes and funct3
# -------------------------
OPCODES = {
    "ADDI":  0b0010011,
    "JAL":   0b1101111,
    "JALR":  0b1100111,
    "BEQ":   0b1100011,
    "BNE":   0b1100011
}

FUNCT3 = {
    "ADDI":  0b000,
    "BEQ":   0b000,
    "BNE":   0b001,
    "JALR":  0b000
}

# -------------------------
# Encoders
# -------------------------

def encode_addi(rd, rs1, imm):
    return (int(tobin(imm, 12) + tobin(rs1, 5) + tobin(FUNCT3["ADDI"], 3)
            + tobin(rd, 5) + tobin(OPCODES["ADDI"], 7), 2))

def encode_jal(rd, offset):
    """offset: PC-relative target - current PC (in bytes)"""
    imm = offset
    bits = tobin(imm, 21)
    # imm[20|10:1|11|19:12]
    return int(bits[0] + bits[10:20] + bits[9] + bits[1:9] +
               tobin(rd, 5) + tobin(OPCODES["JAL"], 7), 2)

def encode_jalr(rd, rs1, imm):
    return int(tobin(imm, 12) + tobin(rs1, 5) + tobin(FUNCT3["JALR"], 3)
               + tobin(rd, 5) + tobin(OPCODES["JALR"], 7), 2)

def encode_branch(op, rs1, rs2, offset):
    """BEQ/BNE: offset is PC-relative (bytes)"""
    imm = offset
    bits = tobin(imm, 13)
    # imm[12|10:5|4:1|11]
    return int(bits[0] + bits[2:8] + tobin(rs2, 5) + tobin(rs1, 5)
               + tobin(FUNCT3[op], 3) + bits[8:12] + bits[1] +
               tobin(OPCODES[op], 7), 2)

# -------------------------
# Example test program
# -------------------------
if __name__ == "__main__":
    print("Example encodings:\n")

    insts = [
        ("ADDI x1, x0, 5", encode_addi(1, 0, 5)),
        ("ADDI x2, x0, 3", encode_addi(2, 0, 3)),
        ("BEQ  x1, x2, +8", encode_branch("BEQ", 1, 2, 8)),
        ("JAL  x5, -12", encode_jal(5, -12)),
        ("JALR x0, x1, 0", encode_jalr(0, 1, 0))
    ]

    for asm, val in insts:
        print(f"{asm:20s} -> bin: {format(val, '032b')}  hex: 0x{val:08x}")
