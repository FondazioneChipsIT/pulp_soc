// ============================================================================
// File: fc_hwpe.sv
// Description: Peripheral in the accelerator slot of the SoC domain, in place
//              of the hwpe-mac-engine example. Same module name as the
//              original, with additional ports.
//
//   The APB port is used directly (the original goes through apb2per towards
//   an HWPE peripheral port). The four memory masters are tied off.
// ============================================================================

module fc_hwpe
#(
    parameter N_MASTER_PORT = 4,
    parameter ID_WIDTH = 8,
    parameter APB_ADDR_WIDTH = 32
)
(
    input  logic         clk_i,
    input  logic         rst_ni,
    input  logic         test_mode_i,

    XBAR_TCDM_BUS.Master hwacc_xbar_master[N_MASTER_PORT-1:0],
    APB_BUS.Slave        hwacc_cfg_slave,

    // camera parallel interface, shared with udma_camera (new ports)
    input  logic         cam_clk_i,
    input  logic [7:0]   cam_data_i,
    input  logic         cam_hsync_i,
    input  logic         cam_vsync_i,
    output logic         cam_trig_o,

    // power management (new ports)
    output logic         pwr_en_o,
    input  logic         pwr_ack_i,
    output logic         mem_ret_o,

    output logic [1:0]   evt_o,
    output logic         busy_o
);

    fulgor_pulp_slot i_fulgor (
        .clk_i         ( clk_i                             ),
        .rst_ni        ( rst_ni                            ),
        .test_mode_i   ( test_mode_i                       ),
        .psel_i        ( hwacc_cfg_slave.psel              ),
        .penable_i     ( hwacc_cfg_slave.penable           ),
        .pwrite_i      ( hwacc_cfg_slave.pwrite            ),
        .paddr_i       ( 32'(hwacc_cfg_slave.paddr)        ),
        .pwdata_i      ( hwacc_cfg_slave.pwdata            ),
        .prdata_o      ( hwacc_cfg_slave.prdata            ),
        .pready_o      ( hwacc_cfg_slave.pready            ),
        .pslverr_o     ( hwacc_cfg_slave.pslverr           ),
        .cam_clk_i     ( cam_clk_i                         ),
        .cam_data_i    ( cam_data_i                        ),
        .cam_hsync_i   ( cam_hsync_i                       ),
        .cam_vsync_i   ( cam_vsync_i                       ),
        .hm0360_trig_o ( cam_trig_o                        ),
        .evt_o         ( evt_o                             ),
        .irq_o         (                                   ),
        .busy_o        ( busy_o                            ),
        .pwr_en_o      ( pwr_en_o                          ),
        .pwr_ack_i     ( pwr_ack_i                         ),
        .mem_ret_o     ( mem_ret_o                         )
    );

    for (genvar i = 0; i < N_MASTER_PORT; i++) begin : gen_no_master
        assign hwacc_xbar_master[i].req   = 1'b0;
        assign hwacc_xbar_master[i].add   = '0;
        assign hwacc_xbar_master[i].wen   = 1'b1;
        assign hwacc_xbar_master[i].wdata = '0;
        assign hwacc_xbar_master[i].be    = '0;
    end

endmodule
