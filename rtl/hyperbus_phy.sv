// Copyright (C) 2017 ETH Zurich, University of Bologna
// All rights reserved.
//
// This code is under development and not yet released to the public.
// Until it is released, the code is under the copyright of ETH Zurich and
// the University of Bologna, and may contain confidential and/or unpublished
// work. Any reuse/redistribution is strictly forbidden without written
// permission from ETH Zurich.

// Author:
// Date:
// Description: Connect the AXI interface with the actual HyperBus
`timescale 1ps/1ps

module hyperbus_phy #(
    parameter BURST_WIDTH = 12,
    parameter NR_CS = 2,
    parameter WAIT_CYCLES = 6
)(
    input  logic                   clk0,    // Clock
    input  logic                   clk90,    // Clock

    input  logic                   rst_ni,   // Asynchronous reset active low

    input  logic                   clk_test,
    input  logic                   test_en_ti,

    // configuration
    input  logic                   config_en_latency_additional_i,
    input  logic  [3:0]            config_t_latency_access_i,
    input  logic [15:0]            config_t_cs_max_i,
    input  logic  [3:0]            config_t_read_write_recovery_i,
    input  logic  [2:0]            config_t_rwds_delay_line_i,
    input  logic  [1:0]            config_t_variable_latency_check_i,

    // transactions
    input  logic                   trans_valid_i,
    output logic                   trans_ready_o,
    input  logic [31:0]            trans_address_i,
    input  logic [NR_CS-1:0]       trans_cs_i,        // chipselect
    input  logic                   trans_write_i,     // transaction is a write
    input  logic [BURST_WIDTH-1:0] trans_burst_i,
    input  logic                   trans_burst_type_i,
    input  logic                   trans_address_space_i,

    // transmitting
    input  logic                   tx_valid_i,
    output logic                   tx_ready_o,
    input  logic [15:0]            tx_data_i,
    input  logic [1:0]             tx_strb_i,   // mask data
    // receiving channel
    output logic                   rx_valid_o,
    input  logic                   rx_ready_i,
    output logic [15:0]            rx_data_o,
    output logic                   rx_last_o, //signals the last transfer in a read burst 
    output logic                   rx_error_o,

    output logic                   b_valid_o,
    output logic                   b_last_o,
    output logic                   b_error_o,

    // physical interface
    output logic [NR_CS-1:0]       hyper_cs_no,
    output logic                   hyper_ck_o,
    output logic                   hyper_ck_no,
    output logic                   hyper_rwds_o,
    input  logic                   hyper_rwds_i,
    output logic                   hyper_rwds_oe_o,
    input  logic [7:0]             hyper_dq_i,
    output logic [7:0]             hyper_dq_o,
    output logic                   hyper_dq_oe_o,
    output logic                   hyper_reset_no,

    //debug
    output logic                   debug_hyper_rwds_oe_o,
    output logic                   debug_hyper_dq_oe_o,
    output logic [3:0]             debug_hyper_phy_state_o
);

    logic [47:0] s_cmd_addr;
    logic [15:0] s_data_out;
    logic [1:0]  s_data_rwds_out;
    logic [15:0] s_CA_out;
    logic [1:0]  r_cmd_addr_sel;
    logic [15:0] s_write_data;
    logic [1:0]  s_write_strb;
    logic [15:0] r_cs_max;
    logic        s_write_valid;


    //local copy of transaction
    logic [31:0]            r_local_address;
    logic [NR_CS-1:0]       r_local_cs;
    logic                   r_local_write;
    logic [BURST_WIDTH-1:0] r_local_burst;
    logic                   r_local_burst_type;
    logic                   r_local_address_space;

    logic r_clock_enable;
    logic r_en_cs;
    logic s_en_ddr_in;
    logic s_en_read_transaction;
    logic s_hyper_rwds_oe_n;
    logic s_hyper_dq_oe_n;
    logic s_mode_write;
    logic r_read_clk_en;
    logic s_read_clk_en_n;
    logic s_read_fifo_rst;

    logic             [3:0] r_wait_cnt;
    logic [BURST_WIDTH-1:0] r_burst_cnt;

    logic s_read_fifo_valid;
    logic r_hyper_rwds_i_syn;
    logic s_en_rwds;

    typedef enum logic[3:0] {STANDBY,SET_CMD_ADDR, CMD_ADDR, REG_WRITE, WAIT2, WAIT, DATA_W, DATA_R, WAIT_R, WAIT_W, ERROR, END_R, END} hyper_trans_t;

    hyper_trans_t r_hyper_trans_state;

    hyperbus_clock_diff_out clock_diff_out_i (
        .in_i   ( clk90        ),
        .en_i   ( r_clock_enable ),
        .out_o  ( hyper_ck_o   ),
        .out_no ( hyper_ck_no  )
    );

    assign hyper_reset_no = rst_ni;

    //selecting ram must be in sync with future hyper_ck_o
    always_ff @(posedge clk90 or negedge rst_ni) begin : proc_hyper_cs_no
        if(~rst_ni) begin
            hyper_cs_no <= {NR_CS{1'b1}};
        end else begin
            hyper_cs_no[0] <= ~ (r_en_cs && r_local_cs[0]);
            hyper_cs_no[1] <= ~ (r_en_cs && r_local_cs[1]); //ToDo Use NR_CS
        end
    end

    always_ff @(posedge clk0 or negedge rst_ni) begin : proc_hyper_rwds_oe
        if(~rst_ni) begin
            hyper_rwds_oe_o <= 0;
            hyper_dq_oe_o <= 0;
            r_read_clk_en <= 0;
        end else begin
            hyper_rwds_oe_o <= s_hyper_rwds_oe_n;
            hyper_dq_oe_o <= s_hyper_dq_oe_n;
            r_read_clk_en <= s_read_clk_en_n;
        end
    end

    genvar i;
    generate
      for(i=0; i<=7; i++)
      begin: ddr_out_bus
        hyperbus_ddr_out ddr_data (
          .rst_ni (rst_ni),
          .clk_i (clk0),
          .d0_i (s_data_out[i+8]),
          .d1_i (s_data_out[i]),
          .q_o (hyper_dq_o[i])
        );
      end
    endgenerate

    assign s_write_data = tx_data_i;
    assign s_write_strb = tx_strb_i;
    assign s_write_valid = tx_valid_i && tx_ready_o;

    assign s_data_out = s_mode_write ? s_write_data : s_CA_out;
    assign s_data_rwds_out = s_mode_write ? s_write_strb : 2'b00; //RWDS low before end of initial latency

    hyperbus_ddr_out ddr_data_strb (
      .rst_ni (rst_ni),
      .clk_i (clk0),
      .d0_i (s_data_rwds_out[1]),
      .d1_i (s_data_rwds_out[0]),
      .q_o (hyper_rwds_o)
    );

    hyperbus_cmd_addr_gen cmd_addr_gen (
        .rw_i            ( ~r_local_write        ),
        .address_space_i ( r_local_address_space ),
        .burst_type_i    ( r_local_burst_type    ),
        .address_i       ( r_local_address       ),
        .cmd_addr_o      ( s_cmd_addr            )
    );

    //Takes output from hyperram, includes CDC FIFO
    hyperbus_read_clk_rwds i_read_clk_rwds (
        .clk0                     ( clk0                          ),
        .rst_ni                   ( rst_ni                        ),
        .clk_test                 ( clk_test                      ),
        .test_en_ti               ( test_en_ti                    ),
        .config_t_rwds_delay_line_i ( config_t_rwds_delay_line_i  ),
        .hyper_rwds_i             ( hyper_rwds_i                  ),
        .hyper_dq_i               ( hyper_dq_i                    ),
        .read_clk_en_i            ( r_read_clk_en                 ),
        .en_ddr_in_i              ( s_en_ddr_in                   ),
        .ready_i                  ( rx_ready_i || s_read_fifo_rst ),
        .data_o                   ( rx_data_o                     ),
        .valid_o                  ( s_read_fifo_valid             )
    );

    assign rx_valid_o = (s_read_fifo_valid && !s_read_fifo_rst) || rx_error_o;
    assign rx_last_o =  (r_burst_cnt == {BURST_WIDTH{1'b0}});

    always_ff @(posedge clk0 or negedge rst_ni) begin : proc_hyper_rwds_i
        if(~rst_ni) begin
            r_hyper_rwds_i_syn <= 0;
        end else if (s_en_rwds) begin
            r_hyper_rwds_i_syn <= hyper_rwds_i;
        end
    end

    always @* begin
        case(r_cmd_addr_sel)
            0: s_CA_out = s_cmd_addr[47:32];
            1: s_CA_out = s_cmd_addr[31:16];
            2: s_CA_out = s_cmd_addr[15:0];
            default: s_CA_out = 16'b0;
        endcase // r_cmd_addr_sel
    end

    always_ff @(posedge clk0 or negedge rst_ni) begin : proc_r_hyper_trans_state
        if(~rst_ni) begin
            r_hyper_trans_state <= STANDBY;
            r_wait_cnt          <= WAIT_CYCLES;
            r_burst_cnt         <= {BURST_WIDTH{1'b0}};
            r_cmd_addr_sel      <= 2'b11;
            r_en_cs             <= 1'b0;
            r_clock_enable      <= 1'b0;
        end else begin
            r_clock_enable <= 1'b0;

            case(r_hyper_trans_state)
                STANDBY: begin
                    if(trans_valid_i) begin
                        r_hyper_trans_state <= SET_CMD_ADDR;
                        r_cmd_addr_sel <= 1'b0;
                        r_en_cs <= 1'b1;
                    end
                end
                SET_CMD_ADDR: begin
                    r_cmd_addr_sel <= r_cmd_addr_sel + 1;
                    r_hyper_trans_state <= CMD_ADDR;
                    r_clock_enable <= 1'b1;
                end
                CMD_ADDR: begin
                     r_clock_enable <= 1'b1;
                    if(r_cmd_addr_sel == 3) begin
                        r_wait_cnt <= config_t_latency_access_i - 2;
                        r_hyper_trans_state <= WAIT2;
                    end else begin
                        r_cmd_addr_sel <= r_cmd_addr_sel + 1;
                    end
                    if(r_cmd_addr_sel == 2) begin
                        if (r_local_address_space && r_local_write) begin //Write to memory config register
                            r_wait_cnt <= 1;
                            r_hyper_trans_state <= REG_WRITE;
                        end
                    end
                end 
                REG_WRITE: begin
                    r_clock_enable <= 1'b1;
                    r_wait_cnt <= r_wait_cnt - 1;
                    if(r_wait_cnt == 4'h0) begin
                        r_clock_enable <= 1'b0;
                        r_wait_cnt <= config_t_read_write_recovery_i - 1;
                        r_hyper_trans_state <= END;
                    end
                end
                WAIT2: begin  //Additional latency (If RWDS HIGH)
                    r_wait_cnt <= r_wait_cnt - 1;
                    r_clock_enable <= 1'b1;
                    if(r_wait_cnt == 4'h0) begin
                        r_wait_cnt <= config_t_latency_access_i - 1;
                        r_hyper_trans_state <= WAIT;
                    end
                    if(r_wait_cnt == config_t_latency_access_i - 2) begin
                        if(r_hyper_rwds_i_syn || config_en_latency_additional_i) begin //Check if additinal latency is nesessary (RWDS high or config)
                            r_hyper_trans_state <= WAIT2;
                        end else begin
                            r_hyper_trans_state <= WAIT;
                        end
                    end
                end
                WAIT: begin  //t_ACC
                    r_wait_cnt <= r_wait_cnt - 1;
                    r_clock_enable <= 1'b1;
                    if(r_wait_cnt == 4'h0) begin
                        if (r_local_write) begin
                            r_hyper_trans_state <= DATA_W;
                            if(s_write_valid) begin
                                r_burst_cnt <= r_local_burst - 1;
                            end else begin //Data to write not ready yet
                                r_burst_cnt <= r_local_burst;
                                r_clock_enable <= 1'b0;
                            end
                        end else begin
                            r_burst_cnt <= r_local_burst - 1;
                            r_hyper_trans_state <= DATA_R;
                        end
                    end
                end
                DATA_R: begin
                    r_clock_enable <= 1'b1;
                    if(rx_valid_o && rx_ready_i) begin
                        if(r_burst_cnt == {BURST_WIDTH{1'b0}}) begin
                            r_clock_enable <= 1'b0;
                            r_hyper_trans_state <= END_R;
                        end else begin
                            r_burst_cnt <= r_burst_cnt - 1;
                        end
                    end else if(~rx_ready_i) begin
                        r_hyper_trans_state <= WAIT_R;
                    end
                end
                DATA_W: begin
                    if(tx_valid_i && tx_ready_o) begin
                        r_clock_enable <= 1'b1;
                        r_burst_cnt <= r_burst_cnt - 1;
                    end else begin
                        r_clock_enable <= 1'b0;
                    end
                    if(r_burst_cnt == 0) begin
                        r_wait_cnt <= config_t_read_write_recovery_i - 1;
                        r_hyper_trans_state <= END;
                    end
                end
                WAIT_R: begin
                    if(rx_valid_o && rx_ready_i) begin
                        r_burst_cnt <= r_burst_cnt - 1;
                    end
                    if(rx_ready_i) begin
                        r_hyper_trans_state <= DATA_R;
                    end
                end
                WAIT_W: begin
                    if(tx_valid_i) begin
                        r_hyper_trans_state <= DATA_W;
                    end
                end
                ERROR: begin
                    r_en_cs <= 1'b0;
                    if (~r_local_write) begin //read
                        if (rx_ready_i) begin
                            r_burst_cnt <= r_burst_cnt - 1;
                            if(r_burst_cnt == {BURST_WIDTH{1'b0}}) begin
                                r_wait_cnt <= config_t_read_write_recovery_i - 2;
                                r_hyper_trans_state <= END;
                            end
                        end
                    end else begin  //write
                        if (~tx_valid_i) begin
                            r_wait_cnt <= config_t_read_write_recovery_i - 2;
                            r_hyper_trans_state <= END;
                        end
                    end
                end
                END_R: begin
                    r_wait_cnt <= config_t_read_write_recovery_i - 2;
                    r_hyper_trans_state <= END;
                end
                END: begin
                    r_en_cs <= 1'b0;
                    if(r_wait_cnt == 4'h0) begin //t_RWR
                        r_hyper_trans_state <= STANDBY;
                    end else begin
                        r_wait_cnt <= r_wait_cnt - 1;
                    end
                end
                default: begin
                    r_hyper_trans_state <= STANDBY;
                end
            endcase

            if(r_cs_max == 1) begin
                r_hyper_trans_state <= ERROR;
            end
        end
    end

    always @* begin
        //defaults
        s_en_ddr_in = 1'b0;
        trans_ready_o = 1'b0;
        tx_ready_o = 1'b0;
        s_hyper_dq_oe_n = 1'b0;
        s_hyper_rwds_oe_n = 1'b0;
        s_en_read_transaction = 1'b0; //Read the transaction
        s_read_clk_en_n = 1'b0;
        s_read_fifo_rst = 1'b0;
        s_mode_write = 1'b0;
        s_en_rwds = 1'b0;
        rx_error_o = 1'b0;
        b_valid_o = 1'b0;
        b_last_o = 1'b0;
        b_error_o = 1'b0;

        case(r_hyper_trans_state)
            STANDBY: begin
                s_en_read_transaction = 1'b1;
                s_hyper_dq_oe_n = 1'b1;
            end
            SET_CMD_ADDR: begin
                trans_ready_o = 1'b1;
                s_hyper_dq_oe_n = 1'b1;
            end
            CMD_ADDR: begin
                s_hyper_dq_oe_n = 1'b1;
                if (r_cmd_addr_sel == config_t_variable_latency_check_i) begin
                    s_en_rwds = 1'b1;
                end
            end
            REG_WRITE: begin
                s_hyper_dq_oe_n = 1'b1;
                s_mode_write = 1'b1;
                b_valid_o = 1'b1;
                b_last_o = 1'b1;
                if(r_wait_cnt == 4'h1) begin
                    tx_ready_o = 1'b1;
                end
            end
            WAIT: begin  //t_ACC
                if(r_local_write == 1'b1) begin
                    if(r_wait_cnt == 4'b0001) begin
                        s_hyper_rwds_oe_n = 1'b1;
                        s_hyper_dq_oe_n = 1'b1;
                    end
                    if (r_wait_cnt == 4'b0000) begin 
                        s_hyper_rwds_oe_n = 1'b1;
                        s_hyper_dq_oe_n = 1'b1;
                        tx_ready_o = 1'b1; 
                        s_mode_write = 1'b1;
                    end
                end
                else begin
                    s_read_clk_en_n = 1'b1;
                end
            end
            DATA_R: begin
                s_en_ddr_in = 1'b1;
                s_read_clk_en_n = 1'b1;
            end
            WAIT_R: begin
                s_en_ddr_in = 1'b1;
                s_read_clk_en_n = 1'b1;
            end
            DATA_W: begin
                s_hyper_dq_oe_n = 1'b1;
                s_hyper_rwds_oe_n = 1'b1;
                tx_ready_o = 1'b1;
                s_mode_write = 1'b1;
                if(r_burst_cnt == 0) begin
                    b_valid_o = 1'b1;
                    b_last_o = 1'b1;
                end
            end
            WAIT_W: begin
                s_hyper_dq_oe_n = 1'b1;
                s_hyper_rwds_oe_n = 1'b1;
                tx_ready_o = 1'b1;
                s_mode_write = 1'b1;
            end
            ERROR: begin //Recover state after timeout for t_CSM 
                s_read_fifo_rst = 1'b1;
                if(~r_local_write) begin
                    rx_error_o = 1'b1;
                end else begin
                    tx_ready_o = 1'b1;
                    b_valid_o = 1'b1;
                    b_error_o = 1'b1;   
                end
            end
            END_R: begin
                s_read_clk_en_n = 1'b1;
                s_read_fifo_rst = 1'b1;
            end
            END: begin
                s_read_fifo_rst = 1'b1;
                s_en_read_transaction = 1'b1;
            end
        endcase
    end

    always_ff @(posedge clk0 or negedge rst_ni) begin : proc_r_cs_max
        if(~rst_ni) begin
            r_cs_max <= 'b0;
        end else begin 
            if (r_en_cs) begin
                r_cs_max <= r_cs_max - 1;
            end else begin
                r_cs_max <= config_t_cs_max_i - 1; //30
            end
        end
    end

    always_ff @(posedge clk0 or negedge rst_ni) begin : proc_local_transaction
        if(~rst_ni) begin
            r_local_address <= 32'h0;
            r_local_cs <= {NR_CS{1'b0}};
            r_local_write <= 1'b0;
            r_local_burst <= {BURST_WIDTH{1'b0}};
            r_local_address_space <= 1'b0;
            r_local_burst_type <= 1'b1;
        end else if(s_en_read_transaction) begin
            r_local_address <= trans_address_i;
            r_local_cs <= trans_cs_i;
            r_local_write <= trans_write_i;
            r_local_burst <= trans_burst_i;
            r_local_burst_type <= trans_burst_type_i;
            r_local_address_space <= trans_address_space_i;
        end
    end

    assign debug_hyper_rwds_oe_o = hyper_rwds_oe_o;
    assign debug_hyper_dq_oe_o = hyper_dq_oe_o;
    assign debug_hyper_phy_state_o = r_hyper_trans_state;

endmodule
