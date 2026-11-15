// =============================================================================
// Gust N1 - memory-management unit (Sv32)
//
// Sits between the core's two buses (virtual addresses) and the system buses
// (physical addresses).  The core is unchanged by it: a page fault comes back
// exactly like a bus error, with one extra wire (pf) saying which kind it is,
// so the core's existing precise bus-fault machinery handles page faults.
//
//   * Translation off (M-mode, or satp.MODE = 0): a pure wire-through, no
//     added delay and no added cycles.
//   * TLB hit: the request goes out in the same cycle with the translated
//     address.  One shared TLB, ENTRIES entries, fully associative, used by
//     both the instruction and the data port.  4 KiB pages and 4 MiB superpages.
//   * TLB miss: the request is held (gnt low) while a hardware walker reads
//     one or two page-table entries through the data bus, then retried.
//   * Permission bits (R W X U A D), SUM and MXR are evaluated on every access
//     from the cached entry, so one entry serves fetches, loads and stores at
//     any privilege.  A and D are never written by hardware: an access that
//     needs A or D set raises a page fault (the "Svade" behaviour), and the
//     entry is dropped so the retry sees the updated page table.
//   * No ASIDs: satp writes and sfence.vma flush the whole TLB (tlb_flush).
//
// Bus rules are those of the Gust bus.  The walker only starts when no data
// response is outstanding, and blocks new data requests while it runs.
// =============================================================================
module gust_mmu #(
    parameter ENTRIES = 8
) (
    input  wire        clk,
    input  wire        rst_n,

    // translation context from the core
    input  wire [1:0]  priv_i,
    input  wire [1:0]  priv_d,
    input  wire        satp_mode,
    input  wire [19:0] satp_ppn,
    input  wire        sum,
    input  wire        mxr,
    input  wire        tlb_flush,

    // core side, instruction (virtual)
    input  wire        c_ibus_req,
    input  wire [31:0] c_ibus_addr,
    output wire        c_ibus_gnt,
    output wire        c_ibus_rvalid,
    output wire [31:0] c_ibus_rdata,
    output wire        c_ibus_err,
    output wire        c_ibus_pf,

    // core side, data (virtual)
    input  wire        c_dbus_req,
    input  wire        c_dbus_we,
    input  wire [3:0]  c_dbus_be,
    input  wire [31:0] c_dbus_addr,
    input  wire [31:0] c_dbus_wdata,
    output wire        c_dbus_gnt,
    output wire        c_dbus_rvalid,
    output wire [31:0] c_dbus_rdata,
    output wire        c_dbus_err,
    output wire        c_dbus_pf,

    // system side (physical)
    output wire        ibus_req,
    output wire [31:0] ibus_addr,
    input  wire        ibus_gnt,
    input  wire        ibus_rvalid,
    input  wire [31:0] ibus_rdata,
    input  wire        ibus_err,

    output wire        dbus_req,
    output wire        dbus_we,
    output wire [3:0]  dbus_be,
    output wire [31:0] dbus_addr,
    output wire [31:0] dbus_wdata,
    input  wire        dbus_gnt,
    input  wire        dbus_rvalid,
    input  wire [31:0] dbus_rdata,
    input  wire        dbus_err
);
    localparam [1:0] PRV_U = 2'b00, PRV_M = 2'b11;
    localparam [2:0] S_IDLE = 3'd0, S_REQ1 = 3'd1, S_WAIT1 = 3'd2,
                     S_REQ0 = 3'd3, S_WAIT0 = 3'd4, S_FAULT = 3'd5;

    wire i_xl = satp_mode & (priv_i != PRV_M);
    wire d_xl = satp_mode & (priv_d != PRV_M);

    // ------------------------------------------------------------------ TLB
    reg               e_valid [0:ENTRIES-1];
    reg               e_super [0:ENTRIES-1];
    reg [19:0]        e_vpn   [0:ENTRIES-1];
    reg [19:0]        e_ppn   [0:ENTRIES-1];
    reg [5:0]         e_perm  [0:ENTRIES-1];      // {D, A, U, X, W, R}
    reg [$clog2(ENTRIES)-1:0] fill_ptr;

    wire [19:0] i_vpn = c_ibus_addr[31:12];
    wire [19:0] d_vpn = c_dbus_addr[31:12];

    reg               i_hit, d_hit;
    reg               i_sup, d_sup;
    reg [19:0]        i_ppn, d_ppn;
    reg [5:0]         i_perm, d_perm;
    reg [ENTRIES-1:0] i_match, d_match;
    integer n;
    always @* begin
        i_hit = 1'b0; d_hit = 1'b0; i_sup = 1'b0; d_sup = 1'b0;
        i_ppn = 20'b0; d_ppn = 20'b0; i_perm = 6'b0; d_perm = 6'b0;
        for (n = 0; n < ENTRIES; n = n + 1) begin
            i_match[n] = e_valid[n] & (e_vpn[n][19:10] == i_vpn[19:10]) &
                         (e_super[n] | (e_vpn[n][9:0] == i_vpn[9:0]));
            d_match[n] = e_valid[n] & (e_vpn[n][19:10] == d_vpn[19:10]) &
                         (e_super[n] | (e_vpn[n][9:0] == d_vpn[9:0]));
            if (i_match[n]) begin
                i_hit = 1'b1; i_sup = e_super[n]; i_ppn = e_ppn[n]; i_perm = e_perm[n];
            end
            if (d_match[n]) begin
                d_hit = 1'b1; d_sup = e_super[n]; d_ppn = e_ppn[n]; d_perm = e_perm[n];
            end
        end
    end

    wire [31:0] i_paddr = {i_ppn[19:10], i_sup ? i_vpn[9:0] : i_ppn[9:0], c_ibus_addr[11:0]};
    wire [31:0] d_paddr = {d_ppn[19:10], d_sup ? d_vpn[9:0] : d_ppn[9:0], c_dbus_addr[11:0]};

    // permission checks: perm = {D, A, U, X, W, R}
    wire i_u_bad = (priv_i == PRV_U) ? ~i_perm[3] : i_perm[3];
    wire d_u_bad = (priv_d == PRV_U) ? ~d_perm[3] : (d_perm[3] & ~sum);
    wire i_fault_perm = ~i_perm[2] | ~i_perm[4] | i_u_bad;
    wire d_fault_perm = (c_dbus_we ? (~d_perm[1] | ~d_perm[5])
                                   : ~(d_perm[0] | (mxr & d_perm[2]))) |
                        ~d_perm[4] | d_u_bad;

    // --------------------------------------------------------------- walker
    reg [2:0]  state;
    reg        w_for_d, w_kill, w_access;
    reg [19:0] w_vpn;
    reg [31:0] w_addr;
    reg        d_out_q;                    // a core data response is outstanding
    wire       walking = (state != S_IDLE);

    wire i_miss = c_ibus_req & i_xl & ~i_hit;
    wire d_miss = c_dbus_req & d_xl & ~d_hit;
    wire d_pass  = c_dbus_req & ~walking & (~d_xl | (d_hit & ~d_fault_perm));
    // Never start while a data request is still waiting for its grant: the
    // walker would have to withdraw it, which the bus protocol forbids.
    wire start  = (state == S_IDLE) & (d_miss | i_miss) & ~d_out_q & ~tlb_flush &
                  ~(d_pass & ~dbus_gnt);

    wire        pte_v    = dbus_rdata[0];
    wire        pte_leaf = dbus_rdata[1] | dbus_rdata[3];
    wire        pte_bad  = ~pte_v | (~dbus_rdata[1] & dbus_rdata[2]);
    wire        pte_wide = |dbus_rdata[31:30];      // physical address above 4 GiB

    // fault delivery
    wire fault_d_now = (state == S_FAULT) & w_for_d & ~w_kill;
    wire fault_i_now = (state == S_FAULT) & ~w_for_d & ~w_kill &
                       c_ibus_req & i_xl & (i_vpn == w_vpn);

    // ------------------------------------------------------------ data port
    wire d_deny  = c_dbus_req & ~walking & d_xl & d_hit & d_fault_perm;
    // the walker's own request also waits for any outstanding data response
    wire w_req   = ((state == S_REQ1) | (state == S_REQ0)) & (~d_out_q | dbus_rvalid);
    wire w_wait  = (state == S_WAIT1) | (state == S_WAIT0);

    assign dbus_req   = w_req | d_pass;
    assign dbus_we    = w_req ? 1'b0 : c_dbus_we;
    assign dbus_be    = w_req ? 4'b1111 : c_dbus_be;
    assign dbus_addr  = w_req ? w_addr : (d_xl ? d_paddr : c_dbus_addr);
    assign dbus_wdata = c_dbus_wdata;

    reg d_fresp_q, d_fresp_pf;
    assign c_dbus_gnt    = (d_pass & dbus_gnt) | d_deny | fault_d_now;
    assign c_dbus_rvalid = (dbus_rvalid & ~w_wait) | d_fresp_q;
    assign c_dbus_rdata  = dbus_rdata;
    assign c_dbus_err    = d_fresp_q | dbus_err;
    assign c_dbus_pf     = d_fresp_q & d_fresp_pf;

    // ----------------------------------------------------- instruction port
    wire i_pass = c_ibus_req & (~i_xl | (i_hit & ~i_fault_perm));
    wire i_deny_now = c_ibus_req & i_xl & i_hit & i_fault_perm;

    assign ibus_req  = i_pass;
    assign ibus_addr = i_xl ? i_paddr : c_ibus_addr;

    reg i_fresp_q, i_fresp_pf;
    assign c_ibus_gnt    = (i_pass & ibus_gnt) | i_deny_now | fault_i_now;
    assign c_ibus_rvalid = ibus_rvalid | i_fresp_q;
    assign c_ibus_rdata  = ibus_rdata;
    assign c_ibus_err    = i_fresp_q | ibus_err;
    assign c_ibus_pf     = i_fresp_q & i_fresp_pf;

    // ---------------------------------------------------------------- state
    integer m;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; w_for_d <= 1'b0; w_kill <= 1'b0; w_access <= 1'b0;
            w_vpn <= 20'b0; w_addr <= 32'b0; d_out_q <= 1'b0;
            d_fresp_q <= 1'b0; d_fresp_pf <= 1'b0; i_fresp_q <= 1'b0; i_fresp_pf <= 1'b0;
            fill_ptr <= 0;
            for (m = 0; m < ENTRIES; m = m + 1) e_valid[m] <= 1'b0;
        end else begin
            // outstanding core data response
            if (d_pass & dbus_gnt) d_out_q <= 1'b1;
            else if (dbus_rvalid & ~w_wait) d_out_q <= 1'b0;

            // fault responses (one cycle after the request was consumed)
            d_fresp_q  <= d_deny | fault_d_now;
            d_fresp_pf <= d_deny | (fault_d_now & ~w_access);
            i_fresp_q  <= i_deny_now | fault_i_now;
            i_fresp_pf <= i_deny_now | (fault_i_now & ~w_access);

            // a permission fault from a cached entry drops that entry, so the
            // retry re-reads the page table (A/D or permissions may have changed)
            for (m = 0; m < ENTRIES; m = m + 1) begin
                if ((d_deny & d_match[m]) | (i_deny_now & i_match[m])) e_valid[m] <= 1'b0;
            end

            if (tlb_flush & walking) w_kill <= 1'b1;

            case (state)
            S_IDLE: begin
                w_kill <= 1'b0;
                if (start) begin
                    w_for_d  <= d_miss;
                    w_vpn    <= d_miss ? d_vpn : i_vpn;
                    w_addr   <= {satp_ppn, (d_miss ? d_vpn[19:10] : i_vpn[19:10]), 2'b00};
                    w_access <= 1'b0;
                    state    <= S_REQ1;
                end
            end
            S_REQ1:  if (w_req & dbus_gnt) state <= S_WAIT1;
            S_REQ0:  if (w_req & dbus_gnt) state <= S_WAIT0;
            S_WAIT1, S_WAIT0: if (dbus_rvalid) begin
                if (dbus_err | pte_wide) begin
                    w_access <= 1'b1;
                    state    <= S_FAULT;
                end else if (pte_bad) begin
                    state <= S_FAULT;
                end else if (pte_leaf) begin
                    if ((state == S_WAIT1) & (|dbus_rdata[19:10])) begin
                        state <= S_FAULT;               // misaligned superpage
                    end else begin
                        if (!(w_kill | tlb_flush)) begin
                            e_valid[fill_ptr] <= 1'b1;
                            e_super[fill_ptr] <= (state == S_WAIT1);
                            e_vpn[fill_ptr]   <= w_vpn;
                            e_ppn[fill_ptr]   <= dbus_rdata[29:10];
                            e_perm[fill_ptr]  <= {dbus_rdata[7:6], dbus_rdata[4:1]};
                            fill_ptr          <= fill_ptr + 1'b1;
                        end
                        state <= S_IDLE;
                    end
                end else if (state == S_WAIT1) begin
                    w_addr <= {dbus_rdata[29:10], w_vpn[9:0], 2'b00};
                    state  <= S_REQ0;
                end else begin
                    state <= S_FAULT;                   // pointer at the last level
                end
            end
            S_FAULT: state <= S_IDLE;
            default: state <= S_IDLE;
            endcase

            if (tlb_flush) begin
                for (m = 0; m < ENTRIES; m = m + 1) e_valid[m] <= 1'b0;
            end
        end
    end
endmodule
