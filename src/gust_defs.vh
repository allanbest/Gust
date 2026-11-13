// =============================================================================
// Gust v1.0 - shared constants
// Included by decode / ALU / core.  Plain Verilog-2005 macros, GUST_ prefix.
// =============================================================================
`ifndef GUST_DEFS_VH
`define GUST_DEFS_VH

// ---- ALU operation select (6 bits) -----------------------------------------
`define GUST_ALU_ADD     6'd0
`define GUST_ALU_SUB     6'd1
`define GUST_ALU_SLT     6'd2
`define GUST_ALU_SLTU    6'd3
`define GUST_ALU_XOR     6'd4
`define GUST_ALU_OR      6'd5
`define GUST_ALU_AND     6'd6
`define GUST_ALU_SLL     6'd7
`define GUST_ALU_SRL     6'd8
`define GUST_ALU_SRA     6'd9
// Zba
`define GUST_ALU_SH1ADD  6'd10
`define GUST_ALU_SH2ADD  6'd11
`define GUST_ALU_SH3ADD  6'd12
// Zbb
`define GUST_ALU_ANDN    6'd13
`define GUST_ALU_ORN     6'd14
`define GUST_ALU_XNOR    6'd15
`define GUST_ALU_MIN     6'd16
`define GUST_ALU_MINU    6'd17
`define GUST_ALU_MAX     6'd18
`define GUST_ALU_MAXU    6'd19
`define GUST_ALU_ROL     6'd20
`define GUST_ALU_ROR     6'd21
`define GUST_ALU_CLZ     6'd22
`define GUST_ALU_CTZ     6'd23
`define GUST_ALU_CPOP    6'd24
`define GUST_ALU_SEXTB   6'd25
`define GUST_ALU_SEXTH   6'd26
`define GUST_ALU_ZEXTH   6'd27
`define GUST_ALU_ORCB    6'd28
`define GUST_ALU_REV8    6'd29
// Zbs
`define GUST_ALU_BSET    6'd30
`define GUST_ALU_BCLR    6'd31
`define GUST_ALU_BINV    6'd32
`define GUST_ALU_BEXT    6'd33

// ---- ALU operand A select ---------------------------------------------------
`define GUST_OPA_RS1     2'd0
`define GUST_OPA_PC      2'd1
`define GUST_OPA_ZERO    2'd2

// ---- X-stage result select --------------------------------------------------
`define GUST_RES_ALU     3'd0
`define GUST_RES_LINK    3'd1
`define GUST_RES_CSR     3'd2
`define GUST_RES_MUL     3'd3
`define GUST_RES_MD      3'd4

// ---- Trap causes (mcause exception codes) -----------------------------------
`define GUST_EXC_IFAULT      5'd1
`define GUST_EXC_ILLEGAL     5'd2
`define GUST_EXC_BREAKPOINT  5'd3
`define GUST_EXC_LMISALIGN   5'd4
`define GUST_EXC_LFAULT      5'd5
`define GUST_EXC_SMISALIGN   5'd6
`define GUST_EXC_SFAULT      5'd7
`define GUST_EXC_ECALL_M     5'd11

`endif
