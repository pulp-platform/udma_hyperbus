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
    input  logic [31:0]            config_t_latency_access,
    input  logic [31:0]            config_en_latency_additional,
    input  logic [31:0]            config_t_cs_max,
    input  logic [31:0]            config_t_read_write_recovery,
    input  logic [31:0]            config_t_rwds_delay_line,
    input  logic [31:0]            config_t_variable_latency_check,

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

    logic [47:0] cmd_addr;
    logic [15:0] data_out;
    logic [1:0]  data_rwds_out;
    logic [15:0] CA_out;
    logic [1:0]  cmd_addr_sel;
    logic [15:0] write_data;
    logic [1:0]  write_strb;
    logic [15:0] cs_max;
    logic        write_valid;


    //local copy of transaction
    (* dont_touch = "true" *) logic [31:0]            local_address;
    logic [NR_CS-1:0]       local_cs;
    logic                   local_write;
    logic [BURST_WIDTH-1:0] local_burst;
    logic                   local_burst_type;
    logic                   local_address_space;

    (* keep = "true" *) logic clock_enable;
    logic en_cs;
    logic en_ddr_in;
    logic en_read_transaction;
    logic hyper_rwds_oe_n;
    logic hyper_dq_oe_n;
    logic mode_write;
    logic read_clk_en;
    logic read_clk_en_n;
    (* keep = "true" *) logic read_fifo_rst;

    (* keep = "true" *) logic [3:0] wait_cnt;
    logic [BURST_WIDTH-1:0] burst_cnt;

    typedef enum logic[3:0] {STANDBY,SET_CMD_ADDR, CMD_ADDR, REG_WRITE, WAIT2, WAIT, DATA_W, DATA_R, WAIT_R, WAIT_W, ERROR, END_R, END} hyper_trans_t;

    (* keep = "true" *) hyper_trans_t hyper_trans_state;

    clock_diff_out clock_diff_out_i (
        .in_i   ( clk90        ),
        .en_i   ( clock_enable ),
        .out_o  ( hyper_ck_o   ),
        .out_no ( hyper_ck_no  )
    );

    assign hyper_reset_no = rst_ni;

    //selecting ram must be in sync with future hyper_ck_o
    always_ff @(posedge clk90 or negedge rst_ni) begin : proc_hyper_cs_no
        if(~rst_ni) begin
            hyper_cs_no <= {NR_CS{1'b1}};
        end else begin
            hyper_cs_no[0] <= ~ (en_cs && local_cs[0]);
            hyper_cs_no[1] <= ~ (en_cs && local_cs[1]); //ToDo Use NR_CS
        end
    end

    always_ff @(posedge clk0 or negedge rst_ni) begin : proc_hyper_rwds_oe
        if(~rst_ni) begin
            hyper_rwds_oe_o <= 0;
            hyper_dq_oe_o <= 0;
            read_clk_en <= 0;
        end else begin
            hyper_rwds_oe_o <= hyper_rwds_oe_n;
            hyper_dq_oe_o <= hyper_dq_oe_n;
            read_clk_en <= read_clk_en_n;
        end
    end

    genvar i;
    generate
      for(i=0; i<=7; i++)
      begin: ddr_out_bus
        ddr_out ddr_data (
          .rst_ni (rst_ni),
          .clk_i (clk0),
          .d0_i (data_out[i+8]),
          .d1_i (data_out[i]),
          .q_o (hyper_dq_o[i])
        );
      end
    endgenerate

    assign write_data = tx_data_i;
    assign write_strb = tx_strb_i;
    assign write_valid = tx_valid_i && tx_ready_o;

    assign data_out = mode_write ? write_data : CA_out;
    assign data_rwds_out = mode_write ? write_strb : 2'b00; //RWDS low before end of initial latency

    ddr_out ddr_data_strb (
      .rst_ni (rst_ni),
      .clk_i (clk0),
      .d0_i (data_rwds_out[1]),
      .d1_i (data_rwds_out[0]),
      .q_o (hyper_rwds_o)
    );

    cmd_addr_gen cmd_addr_gen (
        .rw_i            ( ~local_write        ),
        .address_space_i ( local_address_space ),
        .burst_type_i    ( local_burst_type    ),
        .address_i       ( local_address       ),
        .cmd_addr_o      ( cmd_addr            )
    );

    

    logic read_fifo_valid;

    //Takes output from hyperram, includes CDC FIFO
    read_clk_rwds i_read_clk_rwds (
        .clk0                     ( clk0                        ),
        .rst_ni                   ( rst_ni                      ),
        .clk_test                 ( clk_test                    ),
        .test_en_ti               ( test_en_ti                  ),
        .config_t_rwds_delay_line ( config_t_rwds_delay_line    ),
        .hyper_rwds_i             ( hyper_rwds_i                ),
        .hyper_dq_i               ( hyper_dq_i                  ),
        .read_clk_en_i            ( read_clk_en                 ),
        .en_ddr_in_i              ( en_ddr_in                   ),
        .ready_i                  ( rx_ready_i || read_fifo_rst ),
        .data_o                   ( rx_data_o                   ),
        .valid_o                  ( read_fifo_valid             )
    );

    assign rx_valid_o = (read_fifo_valid && !read_fifo_rst) || rx_error_o;
    assign rx_last_o =  (burst_cnt == {BURST_WIDTH{1'b0}});


    logic hyper_rwds_i_syn;
    (* keep = "true" *) logic en_rwds;

    always_ff @(posedge clk0 or negedge rst_ni) begin : proc_hyper_rwds_i
        if(~rst_ni) begin
            hyper_rwds_i_syn <= 0;
        end else if (en_rwds) begin
            hyper_rwds_i_syn <= hyper_rwds_i;
        end
    end

    always @* begin
        case(cmd_addr_sel)
            0: CA_out = cmd_addr[47:32];
            1: CA_out = cmd_addr[31:16];
            2: CA_out = cmd_addr[15:0];
            default: CA_out = 16'b0;
        endcase // cmd_addr_sel
    end

    always_ff @(posedge clk0 or negedge rst_ni) begin : proc_hyper_trans_state
        if(~rst_ni) begin
            hyper_trans_state <= STANDBY;
            wait_cnt <= WAIT_CYCLES;
            burst_cnt <= {BURST_WIDTH{1'b0}};
            cmd_addr_sel <= 2'b11;
            en_cs <= 1'b0;
            clock_enable <= 1'b0;
        end else begin
            clock_enable <= 1'b0;

            case(hyper_trans_state)
                STANDBY: begin
                    if(trans_valid_i) begin
                        hyper_trans_state <= SET_CMD_ADDR;
                        cmd_addr_sel <= 1'b0;
                        en_cs <= 1'b1;
                    end
                end
                SET_CMD_ADDR: begin
                    cmd_addr_sel <= cmd_addr_sel + 1;
                    hyper_trans_state <= CMD_ADDR;
                    clock_enable <= 1'b1;
                end
                CMD_ADDR: begin
                     clock_enable <= 1'b1;
                    if(cmd_addr_sel == 3) begin
                        wait_cnt <= config_t_latency_access - 2;
                        hyper_trans_state <= WAIT2;
                    end else begin
                        cmd_addr_sel <= cmd_addr_sel + 1;
                    end
                    if(cmd_addr_sel == 2) begin
                        if (local_address_space && local_write) begin //Write to memory config register
                            wait_cnt <= 1;
                            hyper_trans_state <= REG_WRITE;
                        end
                    end
                end 
                REG_WRITE: begin
                    clock_enable <= 1'b1;
                    wait_cnt <= wait_cnt - 1;
                    if(wait_cnt == 4'h0) begin
                        clock_enable <= 1'b0;
                        wait_cnt <= config_t_read_write_recovery - 1;
                        hyper_trans_state <= END;
                    end
                end
                WAIT2: begin  //Additional latency (If RWDS HIGH)
                    wait_cnt <= wait_cnt - 1;
                    clock_enable <= 1'b1;
                    if(wait_cnt == 4'h0) begin
                        wait_cnt <= config_t_latency_access - 1;
                        hyper_trans_state <= WAIT;
                    end
                    if(wait_cnt == config_t_latency_access - 2) begin
                        if(hyper_rwds_i_syn || config_en_latency_additional[0]) begin //Check if additinal latency is nesessary (RWDS high or config)
                            hyper_trans_state <= WAIT2;
                        end else begin
                            hyper_trans_state <= WAIT;
                        end
                    end
                end
                WAIT: begin  //t_ACC
                    wait_cnt <= wait_cnt - 1;
                    clock_enable <= 1'b1;
                    if(wait_cnt == 4'h0) begin
                        if (local_write) begin
                            hyper_trans_state <= DATA_W;
                            if(write_valid) begin
                                burst_cnt <= local_burst - 1;
                            end else begin //Data to write not ready yet
                                burst_cnt <= local_burst;
                                clock_enable <= 1'b0;
                            end
                        end else begin
                            burst_cnt <= local_burst - 1;
                            hyper_trans_state <= DATA_R;
                        end
                    end
                end
                DATA_R: begin
                    clock_enable <= 1'b1;
                    if(rx_valid_o && rx_ready_i) begin
                        if(burst_cnt == {BURST_WIDTH{1'b0}}) begin
                            clock_enable <= 1'b0;
                            hyper_trans_state <= END_R;
                        end else begin
                            burst_cnt <= burst_cnt - 1;
                        end
                    end else if(~rx_ready_i) begin
                        hyper_trans_state <= WAIT_R;
                    end
                end
                DATA_W: begin
                    if(tx_valid_i && tx_ready_o) begin
                        clock_enable <= 1'b1;
                        burst_cnt <= burst_cnt - 1;
                    end else begin
                        clock_enable <= 1'b0;
                    end
                    if(burst_cnt == 0) begin
                        wait_cnt <= config_t_read_write_recovery - 1;
                        hyper_trans_state <= END;
                    end
                end
                WAIT_R: begin
                    if(rx_valid_o && rx_ready_i) begin
                        burst_cnt <= burst_cnt - 1;
                    end
                    if(rx_ready_i) begin
                        hyper_trans_state <= DATA_R;
                    end
                end
                WAIT_W: begin
                    if(tx_valid_i) begin
                        hyper_trans_state <= DATA_W;
                    end
                end
                ERROR: begin
                    en_cs <= 1'b0;
                    if (~local_write) begin //read
                        if (rx_ready_i) begin
                            burst_cnt <= burst_cnt - 1;
                            if(burst_cnt == {BURST_WIDTH{1'b0}}) begin
                                wait_cnt <= config_t_read_write_recovery - 2;
                                hyper_trans_state <= END;
                            end
                        end
                    end else begin  //write
                        if (~tx_valid_i) begin
                            wait_cnt <= config_t_read_write_recovery - 2;
                            hyper_trans_state <= END;
                        end
                    end
                end
                END_R: begin
                    wait_cnt <= config_t_read_write_recovery - 2;
                    hyper_trans_state <= END;
                end
                END: begin
                    en_cs <= 1'b0;
                    if(wait_cnt == 4'h0) begin //t_RWR
                        hyper_trans_state <= STANDBY;
                    end else begin
                        wait_cnt <= wait_cnt - 1;
                    end
                end
                default: begin
                    hyper_trans_state <= STANDBY;
                end
            endcase

            if(cs_max == 1) begin
                hyper_trans_state <= ERROR;
            end
        end
    end

    always @* begin
        //defaults
        en_ddr_in = 1'b0;
        trans_ready_o = 1'b0;
        tx_ready_o = 1'b0;
        hyper_dq_oe_n = 1'b0;
        hyper_rwds_oe_n = 1'b0;
        en_read_transaction = 1'b0; //Read the transaction
        read_clk_en_n = 1'b0;
        read_fifo_rst = 1'b0;
        mode_write = 1'b0;
        en_rwds = 1'b0;
        rx_error_o = 1'b0;
        b_valid_o = 1'b0;
        b_last_o = 1'b0;
        b_error_o = 1'b0;

        case(hyper_trans_state)
            STANDBY: begin
                en_read_transaction = 1'b1;
                hyper_dq_oe_n = 1'b1;
            end
            SET_CMD_ADDR: begin
                trans_ready_o = 1'b1;
                hyper_dq_oe_n = 1'b1;
            end
            CMD_ADDR: begin
                hyper_dq_oe_n = 1'b1;
                if (cmd_addr_sel == config_t_variable_latency_check) begin
                    en_rwds = 1'b1;
                end
            end
            REG_WRITE: begin
                hyper_dq_oe_n = 1'b1;
                mode_write = 1'b1;
                b_valid_o = 1'b1;
                b_last_o = 1'b1;
                if(wait_cnt == 4'h1) begin
                    tx_ready_o = 1'b1;
                end
            end
            WAIT: begin  //t_ACC
                if(local_write == 1'b1) begin
                    if(wait_cnt == 4'b0001) begin
                        hyper_rwds_oe_n = 1'b1;
                        hyper_dq_oe_n = 1'b1;
                    end
                    if (wait_cnt == 4'b0000) begin 
                        hyper_rwds_oe_n = 1'b1;
                        hyper_dq_oe_n = 1'b1;
                        tx_ready_o = 1'b1; 
                        mode_write = 1'b1;
                    end
                end
                else begin
                    read_clk_en_n = 1'b1;
                end
            end
            DATA_R: begin
                en_ddr_in = 1'b1;
                read_clk_en_n = 1'b1;
            end
            WAIT_R: begin
                en_ddr_in = 1'b1;
                read_clk_en_n = 1'b1;
            end
            DATA_W: begin
                hyper_dq_oe_n = 1'b1;
                hyper_rwds_oe_n = 1'b1;
                tx_ready_o = 1'b1;
                mode_write = 1'b1;
                if(burst_cnt == 0) begin
                    b_valid_o = 1'b1;
                    b_last_o = 1'b1;
                end
            end
            WAIT_W: begin
                hyper_dq_oe_n = 1'b1;
                hyper_rwds_oe_n = 1'b1;
                tx_ready_o = 1'b1;
                mode_write = 1'b1;
            end
            ERROR: begin //Recover state after timeout for t_CSM 
                read_fifo_rst = 1'b1;
                if(~local_write) begin
                    rx_error_o = 1'b1;
                end else begin
                    tx_ready_o = 1'b1;
                    b_valid_o = 1'b1;
                    b_error_o = 1'b1;   
                end
            end
            END_R: begin
                read_clk_en_n = 1'b1;
                read_fifo_rst = 1'b1;
            end
            END: begin
                read_fifo_rst = 1'b1;
                en_read_transaction = 1'b1;
            end
        endcase
    end

    always_ff @(posedge clk0 or negedge rst_ni) begin : proc_cs_max
        if(~rst_ni) begin
            cs_max <= 'b0;
        end else begin 
            if (en_cs) begin
                cs_max <= cs_max - 1;
            end else begin
                cs_max <= config_t_cs_max - 1; //30
            end
        end
    end

    always_ff @(posedge clk0 or negedge rst_ni) begin : proc_local_transaction
        if(~rst_ni) begin
            local_address <= 32'h0;
            local_cs <= {NR_CS{1'b0}};
            local_write <= 1'b0;
            local_burst <= {BURST_WIDTH{1'b0}};
            local_address_space <= 1'b0;
            local_burst_type <= 1'b1;
        end else if(en_read_transaction) begin
            local_address <= trans_address_i;
            local_cs <= trans_cs_i;
            local_write <= trans_write_i;
            local_burst <= trans_burst_i;
            local_burst_type <= trans_burst_type_i;
            local_address_space <= trans_address_space_i;
        end
    end

    assign debug_hyper_rwds_oe_o = hyper_rwds_oe_o;
    assign debug_hyper_dq_oe_o = hyper_dq_oe_o;
    assign debug_hyper_phy_state_o = hyper_trans_state;

endmodule
