// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.


module fc_subsystem #(
    parameter CORE_TYPE           = 0,
    parameter USE_XPULP           = 1,     // CV32E40P: PULP ISA extensions (hardware loops, post-increment, SIMD)
    parameter USE_FPU             = 1,
    parameter USE_HWPE            = 1,
    parameter N_EXT_PERF_COUNTERS = 1,
    parameter EVENT_ID_WIDTH      = 8,
    parameter PER_ID_WIDTH        = 32,
    parameter NB_HWPE_PORTS       = 4,
    parameter PULP_SECURE         = 1,
    parameter TB_RISCV            = 0,
    parameter CORE_ID             = 32'h0,
    parameter CLUSTER_ID          = 6'h1F,
    parameter USE_ZFINX           = 1
)
(
    input  logic                      clk_i,
    input  logic                      rst_ni,
    input  logic                      test_en_i,

    XBAR_TCDM_BUS.Master              l2_data_master,
    XBAR_TCDM_BUS.Master              l2_instr_master,
    XBAR_TCDM_BUS.Master              l2_hwpe_master [NB_HWPE_PORTS-1:0],
    APB_BUS.Slave                     apb_slave_eu,
    APB_BUS.Slave                     apb_slave_hwpe,

    // camera parallel interface for the accelerator slot (pins shared with udma_camera)
    input  logic                      cam_clk_i,
    input  logic [7:0]                cam_data_i,
    input  logic                      cam_hsync_i,
    input  logic                      cam_vsync_i,
    output logic                      cam_trig_o,
    // power management of the accelerator slot
    output logic                      fulgor_pwr_en_o,
    input  logic                      fulgor_pwr_ack_i,
    output logic                      fulgor_mem_ret_o,

    input  logic                      fetch_en_i,
    input  logic [31:0]               boot_addr_i,
    input  logic                      debug_req_i,

    input  logic                      event_fifo_valid_i,
    output logic                      event_fifo_fulln_o,
    input  logic [EVENT_ID_WIDTH-1:0] event_fifo_data_i, // goes indirectly to core interrupt
    input  logic [31:0]               events_i, // goes directly to core interrupt, should be called irqs
    output logic [1:0]                hwpe_events_o,

    output logic                      supervisor_mode_o
);

    localparam USE_IBEX   = CORE_TYPE == 1 || CORE_TYPE == 2;
    localparam IBEX_RV32M = CORE_TYPE == 1 ? ibex_pkg::RV32MFast : ibex_pkg::RV32MNone;
    localparam IBEX_RV32E = CORE_TYPE == 2;

    // Interrupt signals
    logic        core_irq_req   ;
    logic        core_irq_sec   ;
    logic [4:0]  core_irq_id    ;
    logic [4:0]  core_irq_ack_id;
    logic        core_irq_ack   ;
    logic [31:0] core_irq_x;
    logic [31:0] fc_events;

    // Boot address, core id, cluster id, fethc enable and core_status
    logic [31:0] boot_addr        ;
    logic        fetch_en_int     ;
    logic        core_busy_int    ;
    logic        perf_counters_int;
    logic [31:0] hart_id;

    //EU signals
    logic core_clock_en;
    logic fetch_en_eu  ;

    //Core Instr Bus
    logic [31:0] core_instr_addr, core_instr_rdata;
    logic        core_instr_req, core_instr_gnt, core_instr_rvalid, core_instr_err;

    //Core Data Bus
    logic [31:0] core_data_addr, core_data_rdata, core_data_wdata;
    logic        core_data_req, core_data_gnt, core_data_rvalid, core_data_err;
    logic        core_data_we  ;
    logic [ 3:0]  core_data_be ;
    logic is_scm_instr_req, is_scm_data_req;

    assign perf_counters_int = 1'b0;
    assign fetch_en_int      = fetch_en_eu & fetch_en_i;

    assign hart_id = {21'b0, CLUSTER_ID[5:0], 1'b0, CORE_ID[3:0]};

    XBAR_TCDM_BUS core_data_bus ();
    XBAR_TCDM_BUS core_instr_bus ();

    //********************************************************
    //************ CORE DEMUX (TCDM vs L2) *******************
    //********************************************************
    // l2_data_master.req and l2_instr_master.req are driven next to the core (FC_CORE below)
    assign l2_data_master.add    = core_data_addr;
    assign l2_data_master.wen    = ~core_data_we;
    assign l2_data_master.wdata  = core_data_wdata;
    assign l2_data_master.be     = core_data_be;
    assign core_data_gnt         = l2_data_master.gnt;
    assign core_data_rvalid      = l2_data_master.r_valid;
    assign core_data_rdata       = l2_data_master.r_rdata;
    assign core_data_err         = l2_data_master.r_opc;


    assign l2_instr_master.add   = core_instr_addr;
    assign l2_instr_master.wen   = 1'b1;
    assign l2_instr_master.wdata = '0;
    assign l2_instr_master.be    = 4'b1111;
    assign core_instr_gnt        = l2_instr_master.gnt;
    assign core_instr_rvalid     = l2_instr_master.r_valid;
    assign core_instr_rdata      = l2_instr_master.r_rdata;
    assign core_instr_err        = l2_instr_master.r_opc;

    //********************************************************
    //************ RISCV CORE ********************************
    //********************************************************
    generate
    if ( USE_IBEX == 0) begin: FC_CORE
    // CV32E40P v2 of OpenHW Group (cv32e40p_v1.8.3 line), Chips-IT branch mpaci/rvfi-fixes. cv32e40p_top holds the core and, with FPU = 1, its FPU.
    // Boot as with RI5CY: execution starts at boot_addr_i (0x1A000080 after reset), the trap vector table sits at
    // the 256-byte aligned address below it. The 32 interrupt lines come from the event unit (one-hot, below).
    assign boot_addr = boot_addr_i;

    // CV32E40P (OBI) can issue a second request before the first response; the SoC interconnect cannot take it.
    // obi_pulp_adapter holds the next request until the response arrives (as in pulp_cluster).
    obi_pulp_adapter i_obi_pulp_adapter_data (
        .rst_ni       ( rst_ni                 ),
        .clk_i        ( clk_i                  ),
        .core_req_i   ( core_data_req          ),
        .mem_gnt_i    ( core_data_gnt          ),
        .mem_rvalid_i ( core_data_rvalid       ),
        .mem_req_o    ( l2_data_master.req     )
    );

    obi_pulp_adapter i_obi_pulp_adapter_instr (
        .rst_ni       ( rst_ni                 ),
        .clk_i        ( clk_i                  ),
        .core_req_i   ( core_instr_req         ),
        .mem_gnt_i    ( core_instr_gnt         ),
        .mem_rvalid_i ( core_instr_rvalid      ),
        .mem_req_o    ( l2_instr_master.req    )
    );

    cv32e40p_top #(
        .COREV_PULP       ( USE_XPULP ),
        .COREV_CLUSTER    ( 0         ),
        .FPU              ( USE_FPU   ),
        .FPU_ADDMUL_LAT   ( 0         ),
        .FPU_OTHERS_LAT   ( 0         ),
        .ZFINX            ( USE_ZFINX ),
        .NUM_MHPMCOUNTERS ( 1         )
    ) lFC_CORE (
        .clk_i                 ( clk_i             ),
        .rst_ni                ( rst_ni            ),
        .pulp_clock_en_i       ( 1'b1              ),     // only used with COREV_CLUSTER = 1
        .scan_cg_en_i          ( test_en_i         ),

        .boot_addr_i           ( boot_addr         ),
        .mtvec_addr_i          ( {boot_addr[31:8], 8'h00} ),
        .dm_halt_addr_i        ( 32'h1A110800      ),
        .hart_id_i             ( hart_id           ),
        .dm_exception_addr_i   ( 32'h1A110808      ),

        // Instruction memory interface
        .instr_req_o           ( core_instr_req    ),
        .instr_gnt_i           ( core_instr_gnt    ),
        .instr_rvalid_i        ( core_instr_rvalid ),
        .instr_addr_o          ( core_instr_addr   ),
        .instr_rdata_i         ( core_instr_rdata  ),

        // Data memory interface
        .data_req_o            ( core_data_req     ),
        .data_gnt_i            ( core_data_gnt     ),
        .data_rvalid_i         ( core_data_rvalid  ),
        .data_we_o             ( core_data_we      ),
        .data_be_o             ( core_data_be      ),
        .data_addr_o           ( core_data_addr    ),
        .data_wdata_o          ( core_data_wdata   ),
        .data_rdata_i          ( core_data_rdata   ),

        // Interrupts
        .irq_i                 ( core_irq_x        ),
        .irq_ack_o             ( core_irq_ack      ),
        .irq_id_o              ( core_irq_ack_id   ),

        // Debug
        .debug_req_i           ( debug_req_i       ),
        .debug_havereset_o     (                   ),
        .debug_running_o       (                   ),
        .debug_halted_o        (                   ),

        // CPU control
        .fetch_enable_i        ( fetch_en_int      ),
        .core_sleep_o          (                   )
    );
    end else begin: FC_CORE
    assign boot_addr = boot_addr_i & 32'hFFFFFF00; // RI5CY expects 0x80 offset, Ibex expects 0x00 offset (adds reset offset 0x80 internally)
    assign l2_data_master.req  = core_data_req;
    assign l2_instr_master.req = core_instr_req;
`ifdef VERILATOR
    ibex_core #(
`elsif TRACE_EXECUTION
    ibex_core_tracing #(
`else
    ibex_core #(
`endif
        .PMPEnable        ( 1'b0                ),
        .PMPGranularity   ( 0                   ),
        .PMPNumRegions    ( 4                   ),
        .MHPMCounterNum   ( 10                  ),
        .MHPMCounterWidth ( 40                  ),
        .RV32E            ( IBEX_RV32E          ),
        .RV32M            ( IBEX_RV32M          ),
        .RV32B            ( ibex_pkg::RV32BNone ),
        .RegFile          ( ibex_pkg::RegFileFF ),
        .BranchTargetALU  ( 1'b0                ),
        .WritebackStage   ( 1'b0                ),
        .ICache           ( 1'b0                ),
        .ICacheECC        ( 1'b0                ),
        .BranchPredictor  ( 1'b0                ),
        .DbgTriggerEn     ( 1'b1                ),
        .DbgHwBreakNum    ( 1                   ),
        .SecureIbex       ( 1'b0                ),
        .DmHaltAddr       ( 32'h1A110800        ),
        .DmExceptionAddr  ( 32'h1A110808        )
    ) lFC_CORE (
        .clk_i                 ( clk_i             ),
        .rst_ni                ( rst_ni            ),

        .test_en_i             ( test_en_i         ),

        .hart_id_i             ( hart_id           ),
        .boot_addr_i           ( boot_addr         ),

        // Instruction Memory Interface:  Interface to Instruction Logaritmic interconnect: Req->grant handshake
        .instr_addr_o          ( core_instr_addr   ),
        .instr_req_o           ( core_instr_req    ),
        .instr_rdata_i         ( core_instr_rdata  ),
        .instr_gnt_i           ( core_instr_gnt    ),
        .instr_rvalid_i        ( core_instr_rvalid ),
        .instr_err_i           ( core_instr_err    ),

        // Data memory interface:
        .data_addr_o           ( core_data_addr    ),
        .data_req_o            ( core_data_req     ),
        .data_be_o             ( core_data_be      ),
        .data_rdata_i          ( core_data_rdata   ),
        .data_we_o             ( core_data_we      ),
        .data_gnt_i            ( core_data_gnt     ),
        .data_wdata_o          ( core_data_wdata   ),
        .data_rvalid_i         ( core_data_rvalid  ),
        .data_err_i            ( core_data_err     ),

        .irq_software_i        ( 1'b0              ),
        .irq_timer_i           ( 1'b0              ),
        .irq_external_i        ( 1'b0              ),
        .irq_fast_i            ( 15'b0             ),
        .irq_nm_i              ( 1'b0              ),

        .irq_x_i               ( core_irq_x        ),
        .irq_x_ack_o           ( core_irq_ack      ),
        .irq_x_ack_id_o        ( core_irq_ack_id   ),

        .external_perf_i       ( { {16 - N_EXT_PERF_COUNTERS {'0}}, perf_counters_int } ),

        .debug_req_i           ( debug_req_i       ),

        .fetch_enable_i        ( fetch_en_int      ),
        .alert_minor_o         (                   ),
        .alert_major_o         (                   ),
        .core_sleep_o          (                   )
    );
    end
    endgenerate

    assign supervisor_mode_o = 1'b1;

    // CV32E40P and Ibex both read 32 interrupt lines: convert the id of the event unit back to one line
    always_comb begin : gen_core_irq_x
        core_irq_x = '0;
        if (core_irq_req) begin
            core_irq_x[core_irq_id] = 1'b1;
        end
    end

    // The CV32E40P takes interrupts only on lines 3, 7, 11 and 16 to 31 (IRQ_MASK of cv32e40p_pkg). The sources that
    // soc_peripherals puts on other lines are moved, in front of the interrupt controller, to lines of the upper half
    // that soc_peripherals leaves at 0. Controller line, core line and vector entry keep one and the same number.
    // Software events (status bits set by software): only 3 and 7 of the lines 0 to 7 reach the core.
    // Ibex keeps the numbering of soc_peripherals.
    always_comb begin : gen_fc_events
        fc_events = events_i;
        if (USE_IBEX == 0) begin
            fc_events[10] = 1'b0;  fc_events[16] = events_i[10];  // FC timer, low half
            fc_events[15] = 1'b0;  fc_events[21] = events_i[15];  // GPIO
            fc_events[14] = 1'b0;  fc_events[22] = events_i[14];  // reference clock edges
            fc_events[8]  = 1'b0;  fc_events[23] = events_i[8];   // DMA, event
            fc_events[9]  = 1'b0;  fc_events[24] = events_i[9];   // DMA, interrupt
            fc_events[12] = 1'b0;  fc_events[25] = events_i[12];  // pf_evt
        end
    end

    apb_interrupt_cntrl #(.PER_ID_WIDTH(PER_ID_WIDTH)) fc_eu_i (
        .clk_i              ( clk_i              ),
        .rst_ni             ( rst_ni             ),
        .test_mode_i        ( test_en_i          ),
        .events_i           ( fc_events          ),
        .event_fifo_valid_i ( event_fifo_valid_i ),
        .event_fifo_fulln_o ( event_fifo_fulln_o ),
        .event_fifo_data_i  ( event_fifo_data_i  ),
        .core_secure_mode_i ( 1'b0               ),
        .core_irq_id_o      ( core_irq_id        ),
        .core_irq_req_o     ( core_irq_req       ),
        .core_irq_ack_i     ( core_irq_ack       ),
        .core_irq_id_i      ( core_irq_ack_id    ),
        .core_irq_sec_o     ( /* SECURE IRQ */   ),
        .core_clock_en_o    ( core_clock_en      ),
        .fetch_en_o         ( fetch_en_eu        ),
        .apb_slave          ( apb_slave_eu       )
    );


    generate
    if(USE_HWPE) begin : fc_hwpe_gen
        fc_hwpe #(
            .N_MASTER_PORT ( NB_HWPE_PORTS ),
            .ID_WIDTH      ( 2             )
        ) i_fc_hwpe (
            .clk_i             ( clk_i          ),
            .rst_ni            ( rst_ni         ),
            .test_mode_i       ( test_en_i      ),
            .hwacc_xbar_master ( l2_hwpe_master ),
            .hwacc_cfg_slave   ( apb_slave_hwpe ),
            .cam_clk_i         ( cam_clk_i      ),
            .cam_data_i        ( cam_data_i     ),
            .cam_hsync_i       ( cam_hsync_i    ),
            .cam_vsync_i       ( cam_vsync_i    ),
            .cam_trig_o        ( cam_trig_o     ),
            .pwr_en_o          ( fulgor_pwr_en_o ),
            .pwr_ack_i         ( fulgor_pwr_ack_i ),
            .mem_ret_o         ( fulgor_mem_ret_o ),
            .evt_o             ( hwpe_events_o  ),
            .busy_o            (                )
        );
    end
    else begin : no_fc_hwpe_gen
        assign hwpe_events_o = '0;
        assign cam_trig_o    = 1'b0;
        assign fulgor_pwr_en_o  = 1'b0;
        assign fulgor_mem_ret_o = 1'b0;
        assign apb_slave_hwpe.prdata  = '0;
        assign apb_slave_hwpe.pready  = '0;
        assign apb_slave_hwpe.pslverr = '0;
        for(genvar ii=0; ii<NB_HWPE_PORTS; ii++) begin : no_fc_hwpe_gen_loop
            assign l2_hwpe_master[ii].req   = '0;
            assign l2_hwpe_master[ii].wen   = '0;
            assign l2_hwpe_master[ii].wdata = '0;
            assign l2_hwpe_master[ii].be    = '0;
            assign l2_hwpe_master[ii].add   = '0;
        end
    end
    endgenerate

endmodule
