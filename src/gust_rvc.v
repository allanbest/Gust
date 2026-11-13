// =============================================================================
// Gust v1.0 - compressed instruction expander (RV32C + optional Zcb)
//
// Purely combinational: a 16-bit parcel in, the equivalent 32-bit base
// instruction out.  Everything downstream only ever sees 32-bit encodings, so
// the decoder and datapath stay single-format.
//
// `illegal` is raised for reserved / RV64-only / floating-point encodings.
// The all-zero parcel is illegal by definition (c.addi4spn with nzuimm == 0).
// =============================================================================
module gust_rvc #(
    parameter EXT_ZCB = 1,   // Zcb: c.lbu/c.lhu/c.lh/c.sb/c.sh/c.not/c.zext.b
    parameter EXT_B   = 1    // needed for Zcb's c.sext.b/c.sext.h/c.zext.h
) (
    input  wire [15:0] c,
    output reg  [31:0] instr,
    output reg         illegal
);
    // 3-bit "prime" register fields map to x8..x15
    wire [4:0] rs1p = {2'b01, c[9:7]};
    wire [4:0] rs2p = {2'b01, c[4:2]};
    wire [4:0] rdf  = c[11:7];      // full-width rd / rs1 field
    wire [4:0] rs2f = c[6:2];       // full-width rs2 field

    localparam [6:0] OP_LOAD   = 7'b0000011;
    localparam [6:0] OP_STORE  = 7'b0100011;
    localparam [6:0] OP_IMM    = 7'b0010011;
    localparam [6:0] OP_REG    = 7'b0110011;
    localparam [6:0] OP_LUI    = 7'b0110111;
    localparam [6:0] OP_BRANCH = 7'b1100011;
    localparam [6:0] OP_JALR   = 7'b1100111;
    localparam [6:0] OP_JAL    = 7'b1101111;

    // c.j / c.jal target, already arranged in JAL's scrambled immediate order:
    // imm[20|10:1|11|19:12]
    wire [19:0] cj_imm = {c[12], c[8], c[10:9], c[6], c[7], c[2], c[11], c[5:3],
                          c[12], {8{c[12]}}};

    always @* begin
        illegal = 1'b0;
        instr   = 32'h0000_0013;                 // default: nop (addi x0,x0,0)
        case (c[1:0])
        // ------------------------------------------------------------ Q0
        2'b00: begin
            case (c[15:13])
            3'b000: begin // c.addi4spn -> addi rd', x2, nzuimm
                instr   = {2'b00, c[10:7], c[12:11], c[5], c[6], 2'b00,
                           5'd2, 3'b000, rs2p, OP_IMM};
                illegal = (c[12:5] == 8'h00);
            end
            3'b010: // c.lw -> lw rd', off(rs1')
                instr = {5'b0, c[5], c[12:10], c[6], 2'b00,
                         rs1p, 3'b010, rs2p, OP_LOAD};
            3'b110: // c.sw -> sw rs2', off(rs1')
                instr = {5'b0, c[5], c[12], rs2p, rs1p, 3'b010,
                         c[11:10], c[6], 2'b00, OP_STORE};
            3'b100: begin // Zcb byte / halfword loads and stores
                illegal = (EXT_ZCB == 0);
                case (c[12:10])
                3'b000: // c.lbu
                    instr = {10'b0, c[5], c[6], rs1p, 3'b100, rs2p, OP_LOAD};
                3'b001: // c.lhu (c[6]=0) / c.lh (c[6]=1)
                    instr = {10'b0, c[5], 1'b0, rs1p,
                             (c[6] ? 3'b001 : 3'b101), rs2p, OP_LOAD};
                3'b010: // c.sb
                    instr = {7'b0, rs2p, rs1p, 3'b000, 3'b000, c[5], c[6],
                             OP_STORE};
                3'b011: begin // c.sh
                    instr = {7'b0, rs2p, rs1p, 3'b001, 3'b000, c[5], 1'b0,
                             OP_STORE};
                    if (c[6]) illegal = 1'b1;
                end
                default: illegal = 1'b1;
                endcase
            end
            default: illegal = 1'b1;  // c.fld / c.flw / c.fsd / c.fsw
            endcase
        end
        // ------------------------------------------------------------ Q1
        2'b01: begin
            case (c[15:13])
            3'b000: // c.addi (c.nop) -> addi rd, rd, imm
                instr = {{6{c[12]}}, c[12], c[6:2], rdf, 3'b000, rdf, OP_IMM};
            3'b001: // c.jal -> jal x1, off
                instr = {cj_imm, 5'd1, OP_JAL};
            3'b010: // c.li -> addi rd, x0, imm
                instr = {{6{c[12]}}, c[12], c[6:2], 5'd0, 3'b000, rdf, OP_IMM};
            3'b011: begin
                if (rdf == 5'd2) // c.addi16sp -> addi x2, x2, nzimm
                    instr = {{3{c[12]}}, c[4:3], c[5], c[2], c[6], 4'b0000,
                             5'd2, 3'b000, 5'd2, OP_IMM};
                else             // c.lui -> lui rd, nzimm
                    instr = {{15{c[12]}}, c[6:2], rdf, OP_LUI};
                illegal = ({c[12], c[6:2]} == 6'b0);
            end
            3'b100: begin
                case (c[11:10])
                2'b00: begin // c.srli
                    instr   = {7'b0000000, c[6:2], rs1p, 3'b101, rs1p, OP_IMM};
                    illegal = c[12];           // shamt[5] reserved on RV32
                end
                2'b01: begin // c.srai
                    instr   = {7'b0100000, c[6:2], rs1p, 3'b101, rs1p, OP_IMM};
                    illegal = c[12];
                end
                2'b10: // c.andi
                    instr = {{6{c[12]}}, c[12], c[6:2], rs1p, 3'b111, rs1p,
                             OP_IMM};
                2'b11: begin
                    if (!c[12]) begin
                        case (c[6:5])
                        2'b00: instr = {7'b0100000, rs2p, rs1p, 3'b000, rs1p, OP_REG}; // c.sub
                        2'b01: instr = {7'b0000000, rs2p, rs1p, 3'b100, rs1p, OP_REG}; // c.xor
                        2'b10: instr = {7'b0000000, rs2p, rs1p, 3'b110, rs1p, OP_REG}; // c.or
                        2'b11: instr = {7'b0000000, rs2p, rs1p, 3'b111, rs1p, OP_REG}; // c.and
                        endcase
                    end else begin
                        // c.subw / c.addw are RV64 only; the rest is Zcb
                        illegal = (EXT_ZCB == 0);
                        case (c[6:5])
                        2'b10: // c.mul -> mul rd', rd', rs2'
                            instr = {7'b0000001, rs2p, rs1p, 3'b000, rs1p, OP_REG};
                        2'b11: begin
                            case (c[4:2])
                            3'b000: // c.zext.b -> andi rd', rd', 0xff
                                instr = {12'h0ff, rs1p, 3'b111, rs1p, OP_IMM};
                            3'b001: begin // c.sext.b (Zbb)
                                instr = {12'h604, rs1p, 3'b001, rs1p, OP_IMM};
                                if (EXT_B == 0) illegal = 1'b1;
                            end
                            3'b010: begin // c.zext.h (Zbb)
                                instr = {7'b0000100, 5'b0, rs1p, 3'b100, rs1p, OP_REG};
                                if (EXT_B == 0) illegal = 1'b1;
                            end
                            3'b011: begin // c.sext.h (Zbb)
                                instr = {12'h605, rs1p, 3'b001, rs1p, OP_IMM};
                                if (EXT_B == 0) illegal = 1'b1;
                            end
                            3'b101: // c.not -> xori rd', rd', -1
                                instr = {12'hfff, rs1p, 3'b100, rs1p, OP_IMM};
                            default: illegal = 1'b1;
                            endcase
                        end
                        default: illegal = 1'b1;
                        endcase
                    end
                end
                endcase
            end
            3'b101: // c.j -> jal x0, off
                instr = {cj_imm, 5'd0, OP_JAL};
            3'b110, 3'b111: // c.beqz / c.bnez -> beq/bne rs1', x0, off
                instr = {{4{c[12]}}, c[6:5], c[2], 5'd0, rs1p,
                         {2'b00, c[13]}, c[11:10], c[4:3], c[12], OP_BRANCH};
            endcase
        end
        // ------------------------------------------------------------ Q2
        2'b10: begin
            case (c[15:13])
            3'b000: begin // c.slli
                instr   = {7'b0000000, c[6:2], rdf, 3'b001, rdf, OP_IMM};
                illegal = c[12];
            end
            3'b010: begin // c.lwsp -> lw rd, off(x2)
                instr   = {4'b0, c[3:2], c[12], c[6:4], 2'b00,
                           5'd2, 3'b010, rdf, OP_LOAD};
                illegal = (rdf == 5'd0);
            end
            3'b100: begin
                if (!c[12]) begin
                    if (rs2f == 5'd0) begin // c.jr -> jalr x0, 0(rs1)
                        instr   = {12'b0, rdf, 3'b000, 5'd0, OP_JALR};
                        illegal = (rdf == 5'd0);
                    end else                // c.mv -> add rd, x0, rs2
                        instr = {7'b0, rs2f, 5'd0, 3'b000, rdf, OP_REG};
                end else begin
                    if (rs2f == 5'd0) begin
                        if (rdf == 5'd0)    // c.ebreak
                            instr = 32'h0010_0073;
                        else                // c.jalr -> jalr x1, 0(rs1)
                            instr = {12'b0, rdf, 3'b000, 5'd1, OP_JALR};
                    end else                // c.add -> add rd, rd, rs2
                        instr = {7'b0, rs2f, rdf, 3'b000, rdf, OP_REG};
                end
            end
            3'b110: // c.swsp -> sw rs2, off(x2)
                instr = {4'b0, c[8:7], c[12], rs2f, 5'd2, 3'b010,
                         c[11:9], 2'b00, OP_STORE};
            default: illegal = 1'b1;  // c.fldsp / c.flwsp / c.fsdsp / c.fswsp
            endcase
        end
        // ---------------------------------------------- not compressed (11)
        default: begin
            instr   = 32'h0000_0013;
            illegal = 1'b1;
        end
        endcase
    end
endmodule
