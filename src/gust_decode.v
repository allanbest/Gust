// =============================================================================
// Gust v1.0 - instruction decoder (RV32IM + Zicsr + Zifencei + optional B)
//
// Combinational.  Takes a 32-bit instruction (compressed forms have already
// been expanded by gust_rvc) and produces datapath controls.  Every legal
// encoding is matched fully; anything else raises `illegal`.
// =============================================================================
`include "gust_defs.vh"

module gust_decode #(
    parameter EXT_B    = 1,   // Zba + Zbb + Zbs
    parameter MUL_FAST = 1    // 1: MUL uses the single-cycle array
) (
    input  wire [31:0] instr,

    output reg         illegal,
    output reg         rs1_used,
    output reg         rs2_used,
    output reg         rd_wen,
    output reg  [31:0] imm,        // I / S / U immediate for the ALU
    output wire [31:0] imm_bj,     // B or J immediate for the branch adder
    output reg  [5:0]  alu_op,
    output reg  [1:0]  op_a_sel,
    output reg         op_b_imm,
    output reg  [2:0]  res_sel,

    output reg         is_branch,
    output reg         is_jal,
    output reg         is_jalr,
    output reg         is_load,
    output reg         is_store,
    output reg         is_csr,
    output reg         is_mul,     // single-cycle multiply
    output reg         is_md,      // sequential multiply / divide
    output reg         is_ecall,
    output reg         is_ebreak,
    output reg         is_mret,
    output reg         is_wfi,
    output reg         is_fencei
);
    wire [6:0] opcode = instr[6:0];
    wire [2:0] f3     = instr[14:12];
    wire [6:0] f7     = instr[31:25];
    wire [4:0] rs2f   = instr[24:20];

    wire [31:0] imm_i = {{20{instr[31]}}, instr[31:20]};
    wire [31:0] imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
    wire [31:0] imm_u = {instr[31:12], 12'b0};
    wire [31:0] imm_b = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
    wire [31:0] imm_j = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

    assign imm_bj = (opcode == 7'b1101111) ? imm_j : imm_b;

    wire has_b = (EXT_B != 0);

    always @* begin
        illegal   = 1'b0;
        rs1_used  = 1'b0;
        rs2_used  = 1'b0;
        rd_wen    = 1'b0;
        imm       = imm_i;
        alu_op    = `GUST_ALU_ADD;
        op_a_sel  = `GUST_OPA_RS1;
        op_b_imm  = 1'b0;
        res_sel   = `GUST_RES_ALU;
        is_branch = 1'b0;
        is_jal    = 1'b0;
        is_jalr   = 1'b0;
        is_load   = 1'b0;
        is_store  = 1'b0;
        is_csr    = 1'b0;
        is_mul    = 1'b0;
        is_md     = 1'b0;
        is_ecall  = 1'b0;
        is_ebreak = 1'b0;
        is_mret   = 1'b0;
        is_wfi    = 1'b0;
        is_fencei = 1'b0;

        case (opcode)
        7'b0110111: begin // LUI
            rd_wen   = 1'b1;
            op_a_sel = `GUST_OPA_ZERO;
            op_b_imm = 1'b1;
            imm      = imm_u;
        end
        7'b0010111: begin // AUIPC
            rd_wen   = 1'b1;
            op_a_sel = `GUST_OPA_PC;
            op_b_imm = 1'b1;
            imm      = imm_u;
        end
        7'b1101111: begin // JAL
            rd_wen  = 1'b1;
            is_jal  = 1'b1;
            res_sel = `GUST_RES_LINK;
        end
        7'b1100111: begin // JALR
            rd_wen   = 1'b1;
            rs1_used = 1'b1;
            op_b_imm = 1'b1;
            is_jalr  = 1'b1;
            res_sel  = `GUST_RES_LINK;
            illegal  = (f3 != 3'b000);
        end
        7'b1100011: begin // BRANCH
            rs1_used  = 1'b1;
            rs2_used  = 1'b1;
            is_branch = 1'b1;
            alu_op    = `GUST_ALU_SUB;
            illegal   = (f3 == 3'b010) | (f3 == 3'b011);
        end
        7'b0000011: begin // LOAD
            rs1_used = 1'b1;
            rd_wen   = 1'b1;
            op_b_imm = 1'b1;
            is_load  = 1'b1;
            illegal  = (f3 == 3'b011) | (f3 == 3'b110) | (f3 == 3'b111);
        end
        7'b0100011: begin // STORE
            rs1_used = 1'b1;
            rs2_used = 1'b1;
            op_b_imm = 1'b1;
            imm      = imm_s;
            is_store = 1'b1;
            illegal  = (f3[2] == 1'b1) | (f3 == 3'b011);
        end
        7'b0010011: begin // OP-IMM
            rs1_used = 1'b1;
            rd_wen   = 1'b1;
            op_b_imm = 1'b1;
            case (f3)
            3'b000: alu_op = `GUST_ALU_ADD;
            3'b010: alu_op = `GUST_ALU_SLT;
            3'b011: alu_op = `GUST_ALU_SLTU;
            3'b100: alu_op = `GUST_ALU_XOR;
            3'b110: alu_op = `GUST_ALU_OR;
            3'b111: alu_op = `GUST_ALU_AND;
            3'b001: begin
                case (f7)
                7'b0000000: alu_op = `GUST_ALU_SLL;
                7'b0110000: begin
                    illegal = ~has_b;
                    case (rs2f)
                    5'b00000: alu_op = `GUST_ALU_CLZ;
                    5'b00001: alu_op = `GUST_ALU_CTZ;
                    5'b00010: alu_op = `GUST_ALU_CPOP;
                    5'b00100: alu_op = `GUST_ALU_SEXTB;
                    5'b00101: alu_op = `GUST_ALU_SEXTH;
                    default:  illegal = 1'b1;
                    endcase
                end
                7'b0100100: begin alu_op = `GUST_ALU_BCLR; illegal = ~has_b; end
                7'b0110100: begin alu_op = `GUST_ALU_BINV; illegal = ~has_b; end
                7'b0010100: begin alu_op = `GUST_ALU_BSET; illegal = ~has_b; end
                default:    illegal = 1'b1;
                endcase
            end
            3'b101: begin
                case (f7)
                7'b0000000: alu_op = `GUST_ALU_SRL;
                7'b0100000: alu_op = `GUST_ALU_SRA;
                7'b0110000: begin alu_op = `GUST_ALU_ROR;  illegal = ~has_b; end
                7'b0100100: begin alu_op = `GUST_ALU_BEXT; illegal = ~has_b; end
                7'b0010100: begin // orc.b : imm = 0x287
                    alu_op  = `GUST_ALU_ORCB;
                    illegal = ~has_b | (rs2f != 5'b00111);
                end
                7'b0110100: begin // rev8 (RV32) : imm = 0x698
                    alu_op  = `GUST_ALU_REV8;
                    illegal = ~has_b | (rs2f != 5'b11000);
                end
                default:    illegal = 1'b1;
                endcase
            end
            endcase
        end
        7'b0110011: begin // OP
            rs1_used = 1'b1;
            rs2_used = 1'b1;
            rd_wen   = 1'b1;
            case (f7)
            7'b0000000: begin
                case (f3)
                3'b000: alu_op = `GUST_ALU_ADD;
                3'b001: alu_op = `GUST_ALU_SLL;
                3'b010: alu_op = `GUST_ALU_SLT;
                3'b011: alu_op = `GUST_ALU_SLTU;
                3'b100: alu_op = `GUST_ALU_XOR;
                3'b101: alu_op = `GUST_ALU_SRL;
                3'b110: alu_op = `GUST_ALU_OR;
                3'b111: alu_op = `GUST_ALU_AND;
                endcase
            end
            7'b0100000: begin
                case (f3)
                3'b000: alu_op = `GUST_ALU_SUB;
                3'b101: alu_op = `GUST_ALU_SRA;
                3'b111: begin alu_op = `GUST_ALU_ANDN; illegal = ~has_b; end
                3'b110: begin alu_op = `GUST_ALU_ORN;  illegal = ~has_b; end
                3'b100: begin alu_op = `GUST_ALU_XNOR; illegal = ~has_b; end
                default: illegal = 1'b1;
                endcase
            end
            7'b0000001: begin // M extension
                if ((f3 == 3'b000) && (MUL_FAST != 0)) begin
                    is_mul  = 1'b1;
                    res_sel = `GUST_RES_MUL;
                end else begin
                    is_md   = 1'b1;
                    res_sel = `GUST_RES_MD;
                end
            end
            7'b0010000: begin // Zba
                illegal = ~has_b;
                case (f3)
                3'b010: alu_op = `GUST_ALU_SH1ADD;
                3'b100: alu_op = `GUST_ALU_SH2ADD;
                3'b110: alu_op = `GUST_ALU_SH3ADD;
                default: illegal = 1'b1;
                endcase
            end
            7'b0000101: begin // Zbb min / max
                illegal = ~has_b;
                case (f3)
                3'b100: alu_op = `GUST_ALU_MIN;
                3'b101: alu_op = `GUST_ALU_MINU;
                3'b110: alu_op = `GUST_ALU_MAX;
                3'b111: alu_op = `GUST_ALU_MAXU;
                default: illegal = 1'b1;
                endcase
            end
            7'b0000100: begin // Zbb zext.h (rs2 field must be zero)
                alu_op   = `GUST_ALU_ZEXTH;
                rs2_used = 1'b0;
                illegal  = ~has_b | (f3 != 3'b100) | (rs2f != 5'b00000);
            end
            7'b0110000: begin // Zbb rotates
                illegal = ~has_b;
                case (f3)
                3'b001: alu_op = `GUST_ALU_ROL;
                3'b101: alu_op = `GUST_ALU_ROR;
                default: illegal = 1'b1;
                endcase
            end
            7'b0100100: begin // Zbs bclr / bext
                illegal = ~has_b;
                case (f3)
                3'b001: alu_op = `GUST_ALU_BCLR;
                3'b101: alu_op = `GUST_ALU_BEXT;
                default: illegal = 1'b1;
                endcase
            end
            7'b0110100: begin // Zbs binv
                alu_op  = `GUST_ALU_BINV;
                illegal = ~has_b | (f3 != 3'b001);
            end
            7'b0010100: begin // Zbs bset
                alu_op  = `GUST_ALU_BSET;
                illegal = ~has_b | (f3 != 3'b001);
            end
            default: illegal = 1'b1;
            endcase
        end
        7'b0001111: begin // MISC-MEM
            // FENCE is a no-op (single hart, no caches).  FENCE.I refetches
            // the instructions already sitting in the prefetch buffer.
            is_fencei = (f3 == 3'b001);
            illegal   = (f3 != 3'b000) & (f3 != 3'b001);
        end
        7'b1110011: begin // SYSTEM
            if (f3 == 3'b000) begin
                case (instr[31:7])
                25'h0000000: is_ecall  = 1'b1;   // 0x00000073
                25'h0002000: is_ebreak = 1'b1;   // 0x00100073
                25'h0604000: is_mret   = 1'b1;   // 0x30200073
                25'h020a000: is_wfi    = 1'b1;   // 0x10500073
                default:     illegal   = 1'b1;
                endcase
            end else if (f3 == 3'b100) begin
                illegal = 1'b1;
            end else begin
                is_csr   = 1'b1;
                rd_wen   = 1'b1;
                rs1_used = ~f3[2];
                res_sel  = `GUST_RES_CSR;
            end
        end
        default: illegal = 1'b1;
        endcase

        // compressed parcels never reach this module un-expanded
        if (instr[1:0] != 2'b11) illegal = 1'b1;
    end
endmodule
