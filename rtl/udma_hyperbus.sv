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

module udma_hyper_top 
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

    logic [31:0] s_udma_rx_data;
    logic        s_udma_rx_data_valid;
    logic        s_udma_rx_data_ready;

    logic [31:0] s_udma_mux_rx_data;
    logic        s_udma_mux_rx_data_valid;
    logic        s_udma_mux_rx_data_valid_fix;
    logic        s_udma_mux_rx_data_ready;
    logic        s_udma_rx_two_bytes;

    logic [31:0] s_udma_tx_data;
    logic        s_udma_tx_data_valid;
    logic        s_udma_tx_data_ready;
    logic        s_hyper_tx_two_bytes;

    logic [31:0] s_hyper_data_tx;
    logic        s_hyper_data_tx_valid;
    logic        s_hyper_data_tx_ready;

    logic        s_clk_hyper;

    logic        s_csn0_en; 
    logic        s_csn1_en; 
    logic        s_csn; 
    logic        s_ck_en;
    logic        s_ck_en_fix; 
    logic        s_ck_no_en; 

    logic        s_dq_oen; 
    logic        s_dq_en; 
    logic  [7:0] s_dq_out0; 
    logic  [7:0] s_dq_out1; 
    logic        s_rwds_oen; 
    logic        s_rwds_en; 
    logic        s_rwds_out0; 
    logic        s_rwds_out1; 
    logic        s_rds_clk; 
    logic  [7:0] s_dq_in0; 
    logic  [7:0] s_dq_in1; 
    logic        s_rwds_in; 

    logic [15:0] s_hyper_mux_data_tx;
    logic        s_hyper_mux_data_tx_valid;
    logic        s_hyper_mux_data_tx_valid_fix;
    logic        s_hyper_mux_data_tx_ready;

    logic [15:0] r_lsb_data_rx;

    logic  [1:0] s_wrap_size0;
    logic  [1:0] s_wrap_size1;
    logic  [7:0] s_mbr0;
    logic  [7:0] s_mbr1;
    logic  [3:0] s_latency0;
    logic  [3:0] s_latency1;
    logic  [3:0] s_rd_cshi0;
    logic  [3:0] s_rd_cshi1;
    logic  [3:0] s_rd_css0;
    logic  [3:0] s_rd_css1;
    logic  [3:0] s_rd_csh0;
    logic  [3:0] s_rd_csh1;
    logic  [3:0] s_wr_cshi0;
    logic  [3:0] s_wr_cshi1;
    logic  [3:0] s_wr_css0;
    logic  [3:0] s_wr_css1;
    logic  [3:0] s_wr_csh0;
    logic  [3:0] s_wr_csh1;
    logic  [8:0] s_rd_max_length0;
    logic  [8:0] s_rd_max_length1;
    logic  [8:0] s_wr_max_length0;
    logic  [8:0] s_wr_max_length1;

    logic        s_acs0;
    logic        s_tco0;
    logic        s_dt0;
    logic        s_crt0;
    logic        s_rd_max_len_en0;
    logic        s_wr_max_len_en0;

    logic        s_acs1;
    logic        s_tco1;
    logic        s_dt1;
    logic        s_crt1;
    logic        s_rd_max_len_en1;
    logic        s_wr_max_len_en1;

    logic        s_clkdiv_en;
    logic        s_trans_rwn;
    logic        s_cfg_hyper_valid;
    logic        s_cfg_hyper_ready;

    assign s_clkdiv_en = 1'b1;

    assign data_tx_datasize_o = 2'b10;
    assign data_rx_datasize_o = 2'b10;

    logic [31:0] s_cfg_hyper_addr;
    logic        s_cfg_hyper_rwn;
    logic  [8:0] s_cfg_hyper_size;

    logic        s_trans_valid;
    logic        s_trans_ready;
    logic  [8:0] s_trans_len;
    logic  [1:0] s_trans_burst;
    logic [31:0] s_trans_addr;
    logic [41:0] s_trans_data;

    logic [41:0] s_trans_sys_data;

    logic        r_msb_tx;
    logic        r_msb_rx;

    logic [2:0] s_rds_delay_adj;

    logic s_trans_valid_fifo;
    logic s_trans_ready_fifo;
    logic s_trans_stall;

    //do NOT enque a write transfer if data is not available
    assign s_trans_stall      = s_trans_valid_fifo & ~s_trans_rwn & ~s_udma_tx_data_valid;
    assign s_trans_ready_fifo = s_trans_stall ? 1'b0 : s_trans_ready;
    assign s_trans_valid      = s_trans_stall ? 1'b0 : s_trans_valid_fifo;

    assign s_trans_sys_data[31:0]  = s_cfg_hyper_addr;
    assign s_trans_sys_data[32]    = s_cfg_hyper_rwn;
    assign s_trans_sys_data[41:33] = s_cfg_hyper_size - 1;

    assign s_trans_rwn   = s_trans_data[32];
    assign s_trans_addr  = s_trans_data[31:0];
    assign s_trans_burst = 2'b01;
    assign s_trans_len   = s_trans_data[41:33];

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

        .cfg_wrap_size0_o    ( s_wrap_size0   ),
        .cfg_wrap_size1_o    ( s_wrap_size1   ),
        .cfg_mbr0_o          ( s_mbr0         ),
        .cfg_mbr1_o          ( s_mbr1         ),
        .cfg_latency0_o      ( s_latency0     ),
        .cfg_latency1_o      ( s_latency1     ),
        .cfg_rd_cshi0_o      ( s_rd_cshi0     ),
        .cfg_rd_cshi1_o      ( s_rd_cshi1     ),
        .cfg_rd_css0_o       ( s_rd_css0      ),
        .cfg_rd_css1_o       ( s_rd_css1      ),
        .cfg_rd_csh0_o       ( s_rd_csh0      ),
        .cfg_rd_csh1_o       ( s_rd_csh1      ),
        .cfg_wr_cshi0_o      ( s_wr_cshi0     ),
        .cfg_wr_cshi1_o      ( s_wr_cshi1     ),
        .cfg_wr_css0_o       ( s_wr_css0      ),
        .cfg_wr_css1_o       ( s_wr_css1      ),
        .cfg_wr_csh0_o       ( s_wr_csh0      ),
        .cfg_wr_csh1_o       ( s_wr_csh1      ),
        .cfg_rd_max_length0_o( s_rd_max_length0 ),
        .cfg_rd_max_length1_o( s_rd_max_length1 ),
        .cfg_wr_max_length0_o( s_wr_max_length0 ),
        .cfg_wr_max_length1_o( s_wr_max_length1 ),
        .cfg_acs0_o           ( s_acs0           ),
        .cfg_tco0_o           ( s_tco0           ),
        .cfg_dt0_o            ( s_dt0            ),
        .cfg_crt0_o           ( s_crt0           ),
        .cfg_rd_max_len_en0_o ( s_rd_max_len_en0 ),
        .cfg_wr_max_len_en0_o ( s_wr_max_len_en0 ),
        .cfg_acs1_o           ( s_acs1           ),
        .cfg_tco1_o           ( s_tco1           ),
        .cfg_dt1_o            ( s_dt1            ),
        .cfg_crt1_o           ( s_crt1           ),
        .cfg_rd_max_len_en1_o ( s_rd_max_len_en1 ),
        .cfg_wr_max_len_en1_o ( s_wr_max_len_en1 ),
        .cfg_rds_delay_adj_o  ( s_rds_delay_adj  ),

        .cfg_hyper_addr_o   ( s_cfg_hyper_addr    ),
        .cfg_hyper_size_o   ( s_cfg_hyper_size    ),
        .cfg_hyper_rwn_o    ( s_cfg_hyper_rwn     ),
        .cfg_hyper_valid_o  ( s_cfg_hyper_valid   ),
        .cfg_hyper_ready_i  ( s_cfg_hyper_ready   )
    );

    hyper_clkgen_sync u_clockgen_sync
    (
        .clk_i           ( periph_clk_i    ),
        .rstn_i          ( rstn_i          ),

        .dft_test_mode_i ( dft_test_mode_i ),
        .dft_cg_enable_i ( dft_cg_enable_i ),

        .cfg_rds_delay_adj_i ( s_rds_delay_adj ),

        .csn0_en_i       ( s_csn0_en       ),
        .csn1_en_i       ( s_csn1_en       ),
        .csn_i           ( s_csn           ),
        .ck_en_i         ( s_ck_en         ),
        .dq_oen_i        ( s_dq_oen        ),
        .dq_en_i         ( s_dq_en         ),
        .dq_out0_i       ( s_dq_out0       ),
        .dq_out1_i       ( s_dq_out1       ),
        .rwds_oen_i      ( s_rwds_oen      ),
        .rwds_en_i       ( s_rwds_en       ),
        .rwds_out0_i     ( s_rwds_out0     ),
        .rwds_out1_i     ( s_rwds_out1     ),
        .rds_clk_o       ( s_rds_clk       ),
        .dq_in0_o        ( s_dq_in0        ),
        .dq_in1_o        ( s_dq_in1        ),
        .rwds_in_o       ( s_rwds_in       ),

        .clk_o           ( s_clk_hyper      ),

        .hyper_clk_o     ( hyper_clk_o      ),
        .hyper_clkn_o    ( hyper_clkn_o     ),
        .hyper_csn0_o    ( hyper_csn0_o     ),
        .hyper_csn1_o    ( hyper_csn1_o     ),
        .hyper_rwds_o    ( hyper_rwds_o     ),
        .hyper_rwds_oen_o( hyper_rwds_oen_o ),
        .hyper_rwds_i    ( hyper_rwds_i     ),
        .hyper_dq_oen_o  ( hyper_dq_oen_o   ),
        .hyper_dq_o      ( hyper_dq_o       ),
        .hyper_dq_i      ( hyper_dq_i       )

    );

    assign s_udma_tx_data[31:16] = 'h0;

    udma_dc_fifo #(16,BUFFER_WIDTH) u_dc_tx
    (
        .dst_clk_i          ( s_clk_hyper           ),   
        .dst_rstn_i         ( rstn_i                ),  
        .dst_data_o         ( s_udma_tx_data[15:0]  ),
        .dst_valid_o        ( s_udma_tx_data_valid  ),
        .dst_ready_i        ( s_udma_tx_data_ready  ),
        .src_clk_i          ( sys_clk_i                 ),
        .src_rstn_i         ( rstn_i                ),
        .src_data_i         ( s_hyper_mux_data_tx       ),
        .src_valid_i        ( s_hyper_mux_data_tx_valid_fix ),
        .src_ready_o        ( s_hyper_mux_data_tx_ready )
    );



    assign s_hyper_mux_data_tx_valid = s_hyper_data_tx_valid;
    assign s_hyper_mux_data_tx_valid_fix = s_hyper_tx_two_bytes ?  s_hyper_data_tx_valid & ~r_msb_tx : s_hyper_data_tx_valid;
    assign s_hyper_tx_two_bytes          = cfg_tx_size_o[0] ^ cfg_tx_size_o[1];
    assign s_hyper_data_tx_ready     = r_msb_tx & s_hyper_mux_data_tx_ready;
    assign s_hyper_mux_data_tx       = r_msb_tx ? s_hyper_data_tx[31:16] : s_hyper_data_tx[15:0];

    always_ff @(posedge sys_clk_i or negedge rstn_i) begin
        if(~rstn_i) begin
            r_msb_tx <= 0;
        end else begin
            if (s_hyper_data_tx_valid && s_hyper_mux_data_tx_ready)
                r_msb_tx <= ~r_msb_tx;
        end
    end

    io_tx_fifo #(
      .DATA_WIDTH(32),
      .BUFFER_DEPTH(4)
    ) u_fifo_tx (
        .clk_i      (sys_clk_i                 ),
        .rstn_i     (rstn_i                ),
        .clr_i      (1'b0                  ),
        .data_o     (s_hyper_data_tx       ),
        .valid_o    (s_hyper_data_tx_valid ),
        .ready_i    (s_hyper_data_tx_ready ),
        .req_o      (data_tx_req_o         ),
        .gnt_i      (data_tx_gnt_i         ),
        .valid_i    (data_tx_valid_i       ),
        .data_i     (data_tx_i             ),
        .ready_o    (data_tx_ready_o       )
    );


    udma_dc_fifo #(32,BUFFER_WIDTH) u_dc_rx
    (
        .dst_clk_i          ( sys_clk_i                ),
        .dst_rstn_i         ( rstn_i               ),
        .dst_data_o         ( data_rx_o            ),
        .dst_valid_o        ( data_rx_valid_o      ),
        .dst_ready_i        ( data_rx_ready_i      ),
        .src_clk_i          ( s_clk_hyper          ),  
        .src_rstn_i         ( rstn_i               ),  
        .src_data_i         ( s_udma_mux_rx_data       ),
        .src_valid_i        ( s_udma_mux_rx_data_valid_fix ),
        .src_ready_o        ( s_udma_mux_rx_data_ready )
    );

    udma_dc_fifo #(42,6) u_dc_trans
    (
        .src_clk_i          ( sys_clk_i                ),  
        .src_rstn_i         ( rstn_i               ),  
        .src_data_i         ( s_trans_sys_data     ),
        .src_valid_i        ( s_cfg_hyper_valid    ),
        .src_ready_o        ( s_cfg_hyper_ready    ),
        .dst_clk_i          ( s_clk_hyper          ),
        .dst_rstn_i         ( rstn_i               ),
        .dst_data_o         ( s_trans_data        ),
        .dst_valid_o        ( s_trans_valid_fifo  ),
        .dst_ready_i        ( s_trans_ready_fifo  )
    );

    hyper_ctrl_wrapper i_hyper_ctrl (
      .clk_i  ( s_clk_hyper ),
      .rstn_i ( rstn_i ),

        .trans_valid_i   ( s_trans_valid ),
        .trans_ready_o   ( s_trans_ready ),
        .trans_rwn_i     ( s_trans_rwn   ),
        .trans_address_i ( s_trans_addr  ),
        .trans_burst_i   ( s_trans_burst ),
        .trans_len_i     ( s_trans_len   ),

        .datatx_valid_i  ( s_udma_tx_data_valid ),
        .datatx_ready_o  ( s_udma_tx_data_ready ),
        .datatx_data_i   ( s_udma_tx_data ),
        .datatx_strb_i   ( 4'b0011 ),

        .datarx_valid_o  ( s_udma_rx_data_valid ),
        .datarx_ready_i  ( s_udma_rx_data_ready ),
        .datarx_data_o   ( s_udma_rx_data ),
        .datarx_strb_o   (  ),

        .cfg_wrap_size0_i    ( s_wrap_size0   ),
        .cfg_wrap_size1_i    ( s_wrap_size1   ),
        .cfg_mbr0_i          ( s_mbr0         ),
        .cfg_mbr1_i          ( s_mbr1         ),
        .cfg_latency0_i      ( s_latency0     ),
        .cfg_latency1_i      ( s_latency1     ),
        .cfg_rd_cshi0_i      ( s_rd_cshi0     ),
        .cfg_rd_cshi1_i      ( s_rd_cshi1     ),
        .cfg_rd_css0_i       ( s_rd_css0      ),
        .cfg_rd_css1_i       ( s_rd_css1      ),
        .cfg_rd_csh0_i       ( s_rd_csh0      ),
        .cfg_rd_csh1_i       ( s_rd_csh1      ),
        .cfg_wr_cshi0_i      ( s_wr_cshi0     ),
        .cfg_wr_cshi1_i      ( s_wr_cshi1     ),
        .cfg_wr_css0_i       ( s_wr_css0      ),
        .cfg_wr_css1_i       ( s_wr_css1      ),
        .cfg_wr_csh0_i       ( s_wr_csh0      ),
        .cfg_wr_csh1_i       ( s_wr_csh1      ),
        .cfg_rd_max_length0_i( s_rd_max_length0 ),
        .cfg_rd_max_length1_i( s_rd_max_length1 ),
        .cfg_wr_max_length0_i( s_wr_max_length0 ),
        .cfg_wr_max_length1_i( s_wr_max_length1 ),

        .cfg_acs0_i           ( s_acs0           ),
        .cfg_tco0_i           ( s_tco0           ),
        .cfg_dt0_i            ( s_dt0            ),
        .cfg_crt0_i           ( s_crt0           ),
        .cfg_rd_max_len_en0_i ( s_rd_max_len_en0 ),
        .cfg_wr_max_len_en0_i ( s_wr_max_len_en0 ),
        .cfg_acs1_i           ( s_acs1           ),
        .cfg_tco1_i           ( s_tco1           ),
        .cfg_dt1_i            ( s_dt1            ),
        .cfg_crt1_i           ( s_crt1           ),
        .cfg_rd_max_len_en1_i ( s_rd_max_len_en1 ),
        .cfg_wr_max_len_en1_i ( s_wr_max_len_en1 ),

        .csn0_en_o       ( s_csn0_en   ),
        .csn1_en_o       ( s_csn1_en   ),
        .csn_o           ( s_csn       ),
        .ck_en_o         ( s_ck_en     ),
        .dq_oen_o        ( s_dq_oen    ),
        .dq_en_o         ( s_dq_en     ),
        .dq_out0_o       ( s_dq_out0   ),
        .dq_out1_o       ( s_dq_out1   ),
        .rwds_oen_o      ( s_rwds_oen  ),
        .rwds_en_o       ( s_rwds_en   ),
        .rwds_out0_o     ( s_rwds_out0 ),
        .rwds_out1_o     ( s_rwds_out1 ),
        .rds_clk_i       ( s_rds_clk   ),
        .dq_in0_i        ( s_dq_in0    ),
        .dq_in1_i        ( s_dq_in1    ),
        .rwds_in_i       ( s_rwds_in   )
    );

    assign s_udma_rx_data_ready     = s_udma_mux_rx_data_ready;
    assign s_udma_mux_rx_data_valid = r_msb_rx & s_udma_rx_data_valid;
    assign s_udma_mux_rx_data_valid_fix =  s_udma_rx_two_bytes ? s_udma_rx_data_valid : s_udma_mux_rx_data_valid;
    assign s_udma_rx_two_bytes          = cfg_rx_size_o[0] ^ cfg_rx_size_o[1];
    assign s_udma_mux_rx_data       = {s_udma_rx_data[15:0],r_lsb_data_rx};

    always_ff @(posedge s_clk_hyper or negedge rstn_i) begin
        if(~rstn_i) begin
            r_msb_rx <= 0;
            r_lsb_data_rx <= 0;
        end else begin
            if (s_udma_rx_data_valid && s_udma_mux_rx_data_ready & ~s_udma_rx_two_bytes)
            begin
                r_lsb_data_rx <= s_udma_rx_data[15:0];
                r_msb_rx      <= ~r_msb_rx;
            end
        end
    end

endmodule
