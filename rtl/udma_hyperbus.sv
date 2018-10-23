// Copyright 2016 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Pullini Antonio - pullinia@iis.ee.ethz.ch                  //
//                                                                            //
// Additional contributions by:                                               //
//                                                                            //
//                                                                            //
// Design Name:    hyper Master Top Level file                                  //
// Project Name:   hyper Master                                                 //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    hyper Master with full QPI support                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

`define log2(VALUE) ((VALUE) < ( 1 ) ? 0 : (VALUE) < ( 2 ) ? 1 : (VALUE) < ( 4 ) ? 2 : (VALUE) < ( 8 ) ? 3 : (VALUE) < ( 16 )  ? 4 : (VALUE) < ( 32 )  ? 5 : (VALUE) < ( 64 )  ? 6 : (VALUE) < ( 128 ) ? 7 : (VALUE) < ( 256 ) ? 8 : (VALUE) < ( 512 ) ? 9 : (VALUE) < ( 1024 ) ? 10 : (VALUE) < ( 2048 ) ? 11 : (VALUE) < ( 4096 ) ? 12 : (VALUE) < ( 8192 ) ? 13 : (VALUE) < ( 16384 ) ? 14 : (VALUE) < ( 32768 ) ? 15 : (VALUE) < ( 65536 ) ? 16 : (VALUE) < ( 131072 ) ? 17 : (VALUE) < ( 262144 ) ? 18 : (VALUE) < ( 524288 ) ? 19 : (VALUE) < ( 1048576 ) ? 20 : (VALUE) < ( 1048576 * 2 ) ? 21 : (VALUE) < ( 1048576 * 4 ) ? 22 : (VALUE) < ( 1048576 * 8 ) ? 23 : (VALUE) < ( 1048576 * 16 ) ? 24 : 25)

module udma_hyperbus
#(
    parameter L2_AWIDTH_NOAL = 12,
    parameter TRANS_SIZE     = 16
)
(
    input  logic                      sys_clk_i,
    input  logic                      periph_clk_i,
    input  logic                      rstn_i,

    input  logic                      dft_test_mode_i,
    input  logic                      dft_cg_enable_i,

    input  logic               [31:0] cfg_data_i,
    input  logic                [4:0] cfg_addr_i,
    input  logic                      cfg_valid_i,
    input  logic                      cfg_rwn_i,
    output logic               [31:0] cfg_data_o,
    output logic                      cfg_ready_o,

    output logic [L2_AWIDTH_NOAL-1:0] cfg_rx_startaddr_o,
    output logic     [TRANS_SIZE-1:0] cfg_rx_size_o,
    output logic                      cfg_rx_continuous_o,
    output logic                      cfg_rx_en_o,
    output logic                      cfg_rx_clr_o,
    input  logic                      cfg_rx_en_i,
    input  logic                      cfg_rx_pending_i,
    input  logic [L2_AWIDTH_NOAL-1:0] cfg_rx_curr_addr_i,
    input  logic     [TRANS_SIZE-1:0] cfg_rx_bytes_left_i,

    output logic [L2_AWIDTH_NOAL-1:0] cfg_tx_startaddr_o,
    output logic     [TRANS_SIZE-1:0] cfg_tx_size_o,
    output logic                      cfg_tx_continuous_o,
    output logic                      cfg_tx_en_o,
    output logic                      cfg_tx_clr_o,
    input  logic                      cfg_tx_en_i,
    input  logic                      cfg_tx_pending_i,
    input  logic [L2_AWIDTH_NOAL-1:0] cfg_tx_curr_addr_i,
    input  logic     [TRANS_SIZE-1:0] cfg_tx_bytes_left_i,

    output logic                      data_tx_req_o,
    input  logic                      data_tx_gnt_i,
    output logic                [1:0] data_tx_datasize_o,
    input  logic               [31:0] data_tx_i,
    input  logic                      data_tx_valid_i,
    output logic                      data_tx_ready_o,
             
    output logic                [1:0] data_rx_datasize_o,
    output logic               [31:0] data_rx_o,
    output logic                      data_rx_valid_o,
    input  logic                      data_rx_ready_i,

    output logic                      hyper_clk_o,
    output logic                      hyper_clkn_o,
    output logic                      hyper_csn0_o,
    output logic                      hyper_csn1_o,
    output logic                      hyper_rwds_o,
    output logic                      hyper_rwds_oen_o,
    input  logic                      hyper_rwds_i,
    output logic                      hyper_dq_oen_o,
    output logic                [7:0] hyper_dq_o,
    input  logic                [7:0] hyper_dq_i

);

    localparam BUFFER_WIDTH=8;
    localparam MODE_BITS = 3;
    localparam TRANS_FIFO_SIZE = 32 + TRANS_SIZE + MODE_BITS + 1;  

    logic                [31:0] s_udma_rx_data;
    logic                       s_udma_rx_data_valid;
    logic                       s_udma_rx_data_ready;

    logic                [31:0] s_udma_tx_data;
    logic                       s_udma_tx_data_valid;
    logic                       s_udma_tx_data_ready;

    logic                [31:0] s_hyper_tx_data;
    logic                       s_hyper_tx_data_valid;
    logic                       s_hyper_tx_data_ready;

    logic                [31:0] s_hyper_rx_data;
    logic                       s_hyper_rx_data_valid;
    logic                       s_hyper_rx_data_ready;

    logic                [15:0] s_phy_tx_data;
    logic                       s_phy_tx_data_valid;
    logic                       s_phy_tx_data_ready;

    logic                [15:0] s_phy_rx_data;
    logic                       s_phy_rx_data_valid;
    logic                       s_phy_rx_data_ready;

    logic                       s_clk_hyper;


    logic   [TRANS_FIFO_SIZE-1] s_cfg_trans_data;
    logic                       s_cfg_trans_valid;
    logic                       s_cfg_trans_ready;

    logic    [TRANS_ARG_SIZE-1] s_cfg_arg_data;
    logic                       s_cfg_arg_valid;
    logic                       s_cfg_arg_ready;


    logic                       s_trans_hyper_valid;
    logic                       s_trans_hyper_ready;
    logic      [TRANS_SIZE-1:0] s_trans_hyper_len;
    logic                 [1:0] s_trans_hyper_burst;
    logic                [31:0] s_trans_hyper_addr;
    logic [TRANS_FIFO_SIZE-1:0] s_trans_hyper_data;

    logic                       s_phy_trans_valid;
    logic                       s_phy_trans_ready;
    logic                [31:0] s_phy_trans_address;
    logic                 [1:0] s_phy_trans_cs;
    logic                       s_phy_trans_write;
    logic      [TRANS_SIZE-1:0] s_phy_trans_burst;
    logic                 [1:0] s_phy_trans_burst_type;
    logic                       s_phy_trans_address_space;
 
    logic                       s_cfg_en_latency_additional;
    logic                 [3:0] s_cfg_latency_access;
    logic                [15:0] s_cfg_cs_max;
    logic                 [3:0] s_cfg_read_write_recovery;
    logic                 [2:0] s_cfg_rwds_delay_line;
    logic                 [1:0] s_cfg_variable_latency_check;


    logic [1:0] s_csn;
    assign hyper_csn0_o = s_csn[0];
    assign hyper_csn1_o = s_csn[1];

    hyper_reg_if #(
        .L2_AWIDTH_NOAL(L2_AWIDTH_NOAL),
        .TRANS_SIZE(TRANS_SIZE)
    ) u_reg_if (
        .clk_i              ( sys_clk_i           ),
        .rstn_i             ( rstn_i              ),

        .cfg_data_i         ( cfg_data_i          ),
        .cfg_addr_i         ( cfg_addr_i          ),
        .cfg_valid_i        ( cfg_valid_i         ),
        .cfg_rwn_i          ( cfg_rwn_i           ),
        .cfg_ready_o        ( cfg_ready_o         ),
        .cfg_data_o         ( cfg_data_o          ),

        .cfg_rx_startaddr_o ( cfg_rx_startaddr_o  ),
        .cfg_rx_size_o      ( cfg_rx_size_o       ),
        .cfg_rx_continuous_o( cfg_rx_continuous_o ),
        .cfg_rx_en_o        ( cfg_rx_en_o         ),
        .cfg_rx_clr_o       ( cfg_rx_clr_o        ),
        .cfg_rx_en_i        ( cfg_rx_en_i         ),
        .cfg_rx_pending_i   ( cfg_rx_pending_i    ),
        .cfg_rx_curr_addr_i ( cfg_rx_curr_addr_i  ),
        .cfg_rx_bytes_left_i( cfg_rx_bytes_left_i ),

        .cfg_tx_startaddr_o ( cfg_tx_startaddr_o  ),
        .cfg_tx_size_o      ( cfg_tx_size_o       ),
        .cfg_tx_continuous_o( cfg_tx_continuous_o ),
        .cfg_tx_en_o        ( cfg_tx_en_o         ),
        .cfg_tx_clr_o       ( cfg_tx_clr_o        ),
        .cfg_tx_en_i        ( cfg_tx_en_i         ),
        .cfg_tx_pending_i   ( cfg_tx_pending_i    ),
        .cfg_tx_curr_addr_i ( cfg_tx_curr_addr_i  ),
        .cfg_tx_bytes_left_i( cfg_tx_bytes_left_i ),

        .cfg_latency_access_o         ( s_cfg_latency_access            ),
        .cfg_en_latency_additional_o  ( s_cfg_en_latency_additional     ),
        .cfg_cs_max_o                 ( s_cfg_cs_max                    ),
        .cfg_read_write_recovery_o    ( s_cfg_read_write_recovery       ),
        .cfg_rwds_delay_line_o        ( s_cfg_rwds_delay_line           ),
        .cfg_variable_latency_check_o ( s_cfg_variable_latency_check    ),

        .clk_div_enable_i ( s_clkdiv_en     ),
        .clk_div_data_i   ( s_clkdiv_data   ),
        .clk_div_valid_i  ( s_clkdiv_valid  ),
        .clk_div_ack_o    ( s_clkdiv_ack    ),

        .cfg_trans_data_o  ( s_cfg_trans_data  ),
        .cfg_trans_valid_o ( s_cfg_trans_valid ),
        .cfg_trans_ready_i ( s_cfg_trans_ready ),
        .cfg_arg_data_o    ( s_cfg_arg_data  ),
        .cfg_arg_valid_o   ( s_cfg_arg_valid ),
        .cfg_arg_ready_i   ( s_cfg_arg_ready )
    );

    udma_clkgen u_clockgen
    (
        .clk_i           ( periph_clk_i    ),
        .rstn_i          ( rstn_i          ),
        .dft_test_mode_i ( 1'b0 ),
        .dft_cg_enable_i ( 1'b0 ),
        .clock_enable_i  ( s_clkdiv_en     ),
        .clk_div_data_i  ( s_clkdiv_data   ),
        .clk_div_valid_i ( s_clkdiv_valid  ),
        .clk_div_ack_o   ( s_clkdiv_ack    ),
        .clk_o           ( s_clk_hyper     )
    );

    hyperbus_clk_gen i_hyper_clk_gen (
        .clk_i           ( s_clk_hyper     ),
        .rst_ni          ( rstn_i          ),
        .dft_test_mode_i ( dft_test_mode_i ),
        .clk0_o          ( s_clk_hyper_0   ),
        .clk90_o         ( s_clk_hyper_90  ),
        .clk180_o        ( s_clk_hyper_180 ),
        .clk270_o        ( s_clk_hyper_270 )
    );

    hyperbus_ctrl #(
        .TRANS_SIZE(TRANS_SIZE)
    ) i_hyperbus_ctrl (
        .clk_i           ( s_clk_hyper_0         ),
        .rst_ni          ( rst_ni                ),
        .arg_data_i      ( s_arg_data            ),
        .arg_valid_i     ( s_arg_valid           ),
        .arg_ready_o     ( s_arg_ready           ),
        .trans_data_i    ( s_trans_data          ),
        .trans_valid_i   ( s_trans_valid         ),
        .trans_ready_o   ( s_trans_ready         ),
        .tx_fifo_data_i  ( s_hyper_tx_data       ),
        .tx_fifo_valid_i ( s_hyper_tx_data_valid ),
        .tx_fifo_ready_o ( s_hyper_tx_data_ready ),
        .tx_phy_data_o   ( s_phy_tx_data         ),
        .tx_phy_valid_o  ( s_phy_tx_data_valid   ),
        .tx_phy_ready_i  ( s_phy_tx_data_ready   ),
        .rx_fifo_data_o  ( s_hyper_rx_data       ),
        .rx_fifo_valid_o ( s_hyper_rx_data_valid ),
        .rx_fifo_ready_i ( s_hyper_rx_data_ready ),
        .rx_phy_data_i   ( s_phy_rx_data         ),
        .rx_phy_valid_i  ( s_phy_rx_data_valid   ),
        .rx_phy_ready_o  ( s_phy_rx_data_ready   )
    );

    hyperbus_phy #(
        .NR_CS(N_CS),
        .BURST_WIDTH(TRANS_SIZE)
    ) i_hyperbus_phy (
        .clk0                              ( s_clk_hyper_0                   ),
        .clk90                             ( s_clk_hyper_90                  ),

        .rst_ni                            ( rstn_i                          ),
        .test_en_ti                        ( dft_test_mode_i                 ),

        .config_t_latency_access_i         ( s_cfg_latency_access            ),
        .config_en_latency_additional_i    ( s_cfg_en_latency_additional     ),
        .config_t_cs_max_i                 ( s_cfg_cs_max                    ),
        .config_t_read_write_recovery_i    ( s_cfg_read_write_recovery       ),
        .config_t_rwds_delay_line_i        ( s_cfg_rwds_delay_line           ),
        .config_t_variable_latency_check_i ( s_cfg_variable_latency_check    ),

        .trans_valid_i                     ( s_phy_trans_valid               ),
        .trans_ready_o                     ( s_phy_trans_ready               ),
        .trans_address_i                   ( s_phy_trans_address             ),
        .trans_cs_i                        ( s_phy_trans_cs                  ),
        .trans_write_i                     ( s_phy_trans_write               ),
        .trans_burst_i                     ( s_phy_trans_burst               ),
        .trans_burst_type_i                ( s_phy_trans_burst_type          ),
        .trans_address_space_i             ( s_phy_trans_address_space       ),

        .tx_data_i                         ( s_phy_tx_data                   ),
        .tx_strb_i                         ( s_phy_tx_data_strb              ),
        .tx_valid_i                        ( s_phy_tx_data_valid             ),
        .tx_ready_o                        ( s_phy_tx_data_ready             ),

        .rx_valid_o                        ( s_phy_rx_data_valid             ),
        .rx_ready_i                        ( s_phy_rx_data_ready             ),
        .rx_data_o                         ( s_phy_rx_data                   ),
        .rx_last_o                         ( ),
        .rx_error_o                        ( ),
        .b_valid_o                         ( ),
        .b_last_o                          ( ),
        .b_error_o                         ( ),

        .hyper_cs_no                       ( s_csn                           ),
        .hyper_ck_o                        ( hyper_clk_o                     ),
        .hyper_ck_no                       ( hyper_clkn_o                    ),
        .hyper_rwds_o                      ( hyper_rwds_o                    ),
        .hyper_rwds_i                      ( hyper_rwds_i                    ),
        .hyper_rwds_oe_o                   ( hyper_rwds_oen_o                ),
        .hyper_dq_i                        ( hyper_dq_i                      ),
        .hyper_dq_o                        ( hyper_dq_o                      ),
        .hyper_dq_oe_o                     ( hyper_dq_oen_o                  ),
        .hyper_reset_no                    ( hyper_reset_no                  ),
        .debug_hyper_rwds_oe_o             ( ),
        .debug_hyper_dq_oe_o               ( ),
        .debug_hyper_phy_state_o           ( )
    );

    io_tx_fifo #(
      .DATA_WIDTH(32),
      .BUFFER_DEPTH(4)
    ) u_fifo_tx (
        .clk_i      ( sys_clk_i             ),
        .rstn_i     ( rstn_i                ),
        .clr_i      ( 1'b0                  ),
        .data_o     ( s_udma_data_tx        ),
        .valid_o    ( s_udma_data_tx_valid  ),
        .ready_i    ( s_udma_data_tx_ready  ),
        .req_o      ( data_tx_req_o         ),
        .gnt_i      ( data_tx_gnt_i         ),
        .valid_i    ( data_tx_valid_i       ),
        .data_i     ( data_tx_i             ),
        .ready_o    ( data_tx_ready_o       )
    );

    udma_dc_fifo #(32,BUFFER_WIDTH) u_dc_tx
    (
        .dst_clk_i          ( s_clk_hyper_0         ),   
        .dst_rstn_i         ( rstn_i                ),  
        .dst_data_o         ( s_hyper_tx_data       ),
        .dst_valid_o        ( s_hyper_tx_data_valid ),
        .dst_ready_i        ( s_hyper_tx_data_ready ),
        .src_clk_i          ( sys_clk_i             ),
        .src_rstn_i         ( rstn_i                ),
        .src_data_i         ( s_udma_data_tx        ),
        .src_valid_i        ( s_udma_data_tx_valid  ),
        .src_ready_o        ( s_udma_data_tx_ready  )
    );

    udma_dc_fifo #(32,BUFFER_WIDTH) u_dc_rx
    (
        .dst_clk_i          ( sys_clk_i             ),
        .dst_rstn_i         ( rstn_i                ),
        .dst_data_o         ( data_rx_o             ),
        .dst_valid_o        ( data_rx_valid_o       ),
        .dst_ready_i        ( data_rx_ready_i       ),
        .src_clk_i          ( s_clk_hyper_0         ),  
        .src_rstn_i         ( rstn_i                ),  
        .src_data_i         ( s_hyper_data_rx       ),
        .src_valid_i        ( s_hyper_data_rx_valid ),
        .src_ready_o        ( s_hyper_data_rx_ready )
    );

    udma_dc_fifo #(TRANS_FIFO_SIZE,6) i_dc_transaction_fifo
    (
        .src_clk_i          ( sys_clk_i           ),  
        .src_rstn_i         ( rstn_i              ),  
        .src_data_i         ( s_cfg_trans_data    ),
        .src_valid_i        ( s_cfg_trans_valid   ),
        .src_ready_o        ( s_cfg_trans_ready   ),
        .dst_clk_i          ( s_clk_hyper_0       ),
        .dst_rstn_i         ( rstn_i              ),
        .dst_data_o         ( s_trans_data        ),
        .dst_valid_o        ( s_trans_valid       ),
        .dst_ready_i        ( s_trans_ready       )
    );

    udma_dc_fifo #(TRANS_ARG_SIZE,6) i_dc_argument_fifo
    (
        .src_clk_i          ( sys_clk_i       ),  
        .src_rstn_i         ( rstn_i          ),  
        .src_data_i         ( s_cfg_arg_data  ),
        .src_valid_i        ( s_cfg_arg_valid ),
        .src_ready_o        ( s_cfg_arg_ready ),
        .dst_clk_i          ( s_clk_hyper_0   ),
        .dst_rstn_i         ( rstn_i          ),
        .dst_data_o         ( s_arg_data      ),
        .dst_valid_o        ( s_arg_valid     ),
        .dst_ready_i        ( s_arg_ready     )
    );

endmodule
