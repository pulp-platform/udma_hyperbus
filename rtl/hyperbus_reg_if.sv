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
// Design Name:    HyperRAM/FLASH Programming Register Interface              //
// Project Name:   HyperRAM/FLASH                                             //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    HyperRAM/FLASH memory controller                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// SPI Master Registers
`define REG_RX_SADDR     5'b00000 //BASEADDR+0x00 
`define REG_RX_SIZE      5'b00001 //BASEADDR+0x04
`define REG_RX_CFG       5'b00010 //BASEADDR+0x08  
`define REG_RX_INTCFG    5'b00011 //BASEADDR+0x0C  

`define REG_TX_SADDR     5'b00100 //BASEADDR+0x10
`define REG_TX_SIZE      5'b00101 //BASEADDR+0x14
`define REG_TX_CFG       5'b00110 //BASEADDR+0x18
`define REG_TX_INTCFG    5'b00111 //BASEADDR+0x1C

`define REG_EXT_ADDR     5'b01000 //BASEADDR+0x20
`define REG_EXT_CFG      5'b01001 //BASEADDR+0x24

`define REG_MEM_CFG0     5'b01010 //BASEADDR+0x28
`define REG_HYP_REG      5'b01011 //BASEADDR+0x2C
`define REG_HYP_STRIDE   5'b01100 //BASEADDR+0x30
`define REG_HYP_LINE     5'b01101 //BASEADDR+0x34
`define REG_MEM_CFG4     5'b01110 //BASEADDR+0x38
`define REG_MEM_CFG5     5'b01111 //BASEADDR+0x3C
`define REG_MEM_CFG6     5'b10000 //BASEADDR+0x40
`define REG_MEM_CFG7     5'b10001 //BASEADDR+0x44

`define MODE_NORMAL      3'h0
`define MODE_REG         3'h1
`define MODE_2D          3'h2

module hyper_reg_if #(
    parameter L2_AWIDTH_NOAL = 12,
    parameter TRANS_SIZE     = 16,
    parameter TRANS_DATA_SIZE = 32 + TRANS_SIZE + 4 + 1
) (
	input  logic 	                  clk_i,
	input  logic   	                  rstn_i,

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

    output logic                      cfg_en_latency_additional_o,
    output logic                [3:0] cfg_latency_access_o,
    output logic               [15:0] cfg_cs_max_o,
    output logic                [3:0] cfg_read_write_recovery_o,
    output logic                [2:0] cfg_rwds_delay_line_o,
    output logic                [1:0] cfg_variable_latency_check_o,

    output logic                      clk_div_enable_o,
    output logic                      clk_div_data_o,
    output logic                      clk_div_valid_o,
    input  logic                      clk_div_ack_i,

    output logic [TRANS_DATA_SIZE-1:0] cfg_trans_data_o,
    output logic                       cfg_trans_valid_o,
    input  logic                       cfg_trans_ready_i,

    output logic                [31:0] cfg_arg_data_o,
    output logic                       cfg_arg_valid_o,
    input  logic                       cfg_arg_ready_i

);

    logic [L2_AWIDTH_NOAL-1:0] r_rx_startaddr;
    logic   [TRANS_SIZE-1 : 0] r_rx_size;
    logic                      r_rx_continuous;
    logic                      r_rx_en;
    logic                      r_rx_clr;

    logic [L2_AWIDTH_NOAL-1:0] r_tx_startaddr;
    logic   [TRANS_SIZE-1 : 0] r_tx_size;
    logic                      r_tx_continuous;
    logic                      r_tx_en;
    logic                      r_tx_clr;

    logic                      r_tx_reg;
    logic                      r_rx_reg;

    logic   [TRANS_SIZE-1 : 0] r_stride;
    logic   [TRANS_SIZE-1 : 0] r_line;

    logic   [TRANS_SIZE-1 : 0] s_trans_size;

    logic                [4:0] s_wr_addr;
    logic                [4:0] s_rd_addr;

    logic               [31:0] r_startaddr;
    logic                [3:0] r_mode;

    logic                      r_en_latency_additional;
    logic                [3:0] r_latency_access;
    logic               [15:0] r_cs_max;
    logic                [3:0] r_read_write_recovery;
    logic                [2:0] r_rwds_delay_line;
    logic                [1:0] r_variable_latency_check;

    logic               [15:0] r_reg_val;


    assign s_wr_addr = (cfg_valid_i & ~cfg_rwn_i) ? cfg_addr_i : 5'h0;
    assign s_rd_addr = (cfg_valid_i &  cfg_rwn_i) ? cfg_addr_i : 5'h0;

    assign cfg_rx_startaddr_o  = r_rx_startaddr;
    assign cfg_rx_size_o       = r_rx_size;
    assign cfg_rx_continuous_o = r_rx_continuous;
    assign cfg_rx_en_o         = r_rx_en;
    assign cfg_rx_clr_o        = r_rx_clr;

    assign cfg_tx_startaddr_o  = r_tx_startaddr;
    assign cfg_tx_size_o       = r_tx_size;
    assign cfg_tx_continuous_o = r_tx_continuous;
    assign cfg_tx_en_o         = r_tx_en;
    assign cfg_tx_clr_o        = r_tx_clr;

    assign cfg_trans_data_o    = {r_startaddr,s_trans_size,r_rx_en,r_mode};
    assign cfg_trans_valid_o   = r_tx_en | r_rx_en;

    assign cfg_arg_valid_o     = r_tx_reg | r_rx_reg;

    assign s_trans_size        = r_rx_en ? r_rx_size : r_tx_size;

    assign cfg_en_latency_additional_o  = r_en_latency_additional;
    assign cfg_latency_access_o         = r_latency_access;
    assign cfg_cs_max_o                 = r_cs_max;
    assign cfg_read_write_recovery_o    = r_read_write_recovery;
    assign cfg_rwds_delay_line_o        = r_rwds_delay_line;
    assign cfg_variable_latency_check_o = r_variable_latency_check;

    edge_propagator_tx i_edgeprop_soc 
    (
      .clk_i(clk_i),
      .rstn_i(rstn_i),
      .valid_i(r_clk_div_valid),
      .ack_i(clk_div_ack_i),
      .valid_o(clk_div_data_o)
    );

    always_comb 
    begin
        cfg_arg_data_o = 'h0;
        case(r_mode[2:0])
            `MODE_2D:
                cfg_arg_data_o = r_stride;
            `MODE_REG:
                cfg_arg_data_o = {16'h0,r_reg_val};
        endcase
    end

    always_ff @(posedge clk_i, negedge rstn_i) 
    begin
        if(~rstn_i) 
        begin
            // SPI REGS
            r_rx_startaddr  <=  'h0;
            r_rx_size       <=  'h0;
            r_rx_continuous <=  'h0;
            r_rx_en          =  'h0;
            r_rx_clr         =  'h0;
            r_tx_startaddr  <=  'h0;
            r_tx_size       <=  'h0;
            r_tx_continuous <=  'h0;
            r_tx_en          =  'h0;
            r_tx_clr         =  'h0;
            r_startaddr     <=  'h0;
            r_mode          <=  'h0;
            r_rx_reg         =  'h0;
            r_tx_reg         =  'h0;
            r_reg_val       <=  'h0;
            r_en_latency_additional  <= 'h0;
            r_latency_access         <= 'h0;
            r_cs_max                 <= 'h0;
            r_read_write_recovery    <= 'h0;
            r_rwds_delay_line        <= 'h0;
            r_variable_latency_check <= 'h0;
            r_stride        <= 'h0;
            r_line          <= 'h0;
        end
        else
        begin
            r_rx_en          =  'h0;
            r_rx_clr         =  'h0;
            r_tx_en          =  'h0;
            r_tx_clr         =  'h0;
            r_rx_reg         =  'h0;
            r_tx_reg         =  'h0;

            if (cfg_valid_i & ~cfg_rwn_i)
            begin
                case (s_wr_addr)
                `REG_MEM_CFG0:
                begin
                    r_latency_access         <= cfg_data_i[3:0];
                    r_read_write_recovery    <= cfg_data_i[7:4];
                    r_rwds_delay_line        <= cfg_data_i[10:8];
                    r_variable_latency_check <= cfg_data_i[12:11];
                    r_en_latency_additional  <= cfg_data_i[13];
                    r_cs_max                 <= cfg_data_i[31:16];
                end
                `REG_RX_SADDR:
                    r_rx_startaddr   <= cfg_data_i[L2_AWIDTH_NOAL-1:0];
                `REG_RX_SIZE:
                    r_rx_size        <= cfg_data_i[TRANS_SIZE-1:0];
                `REG_RX_CFG:
                begin
                    r_rx_clr          = cfg_data_i[5];
                    r_rx_en           = cfg_data_i[4];
                    r_rx_continuous  <= cfg_data_i[0];
                end
                `REG_TX_SADDR:
                    r_tx_startaddr   <= cfg_data_i[L2_AWIDTH_NOAL-1:0];
                `REG_TX_SIZE:
                    r_tx_size        <= cfg_data_i[TRANS_SIZE-1:0];
                `REG_TX_CFG:
                begin
                    r_tx_clr          = cfg_data_i[5];
                    r_tx_en           = cfg_data_i[4];
                    r_tx_continuous  <= cfg_data_i[0];
                end
                `REG_EXT_ADDR:
                    r_startaddr      <= cfg_data_i;
                `REG_EXT_CFG:
                begin
                    r_mode           <= cfg_data_i[3:0];
                end
                `REG_HYP_REG:
                begin
                    r_rx_reg          =  cfg_data_i[16];
                    r_tx_reg          = ~cfg_data_i[16];
                    r_reg_val        <=  cfg_data_i[15:0];
                end
                `REG_HYP_STRIDE:
                begin
                    r_stride         <=  cfg_data_i[TRANS_SIZE-1:0];
                end
                `REG_HYP_LINE:
                begin
                    r_line           <=  cfg_data_i[TRANS_SIZE-1:0];
                end
                endcase
            end
        end
    end //always

    always_comb
    begin
        cfg_data_o = 32'h0;
        case (s_rd_addr)
        `REG_RX_SADDR:
            cfg_data_o =  {{(32- L2_AWIDTH_NOAL){1'b0}} , cfg_rx_curr_addr_i };
        `REG_RX_SIZE:
            cfg_data_o[TRANS_SIZE-1:0] = cfg_rx_bytes_left_i;
        `REG_RX_CFG:
            cfg_data_o = {26'h0,cfg_rx_pending_i,cfg_rx_en_i,3'h0,r_rx_continuous};
        `REG_TX_SADDR:
            cfg_data_o = {{(32- L2_AWIDTH_NOAL){1'b0}} , cfg_tx_curr_addr_i};
        `REG_TX_SIZE:
            cfg_data_o[TRANS_SIZE-1:0] = cfg_tx_bytes_left_i;
        `REG_TX_CFG:
            cfg_data_o = {26'h0,cfg_tx_pending_i,cfg_tx_en_i,3'h0,r_tx_continuous};
        `REG_EXT_ADDR:
            cfg_data_o = r_startaddr;
        `REG_MEM_CFG0:
            cfg_data_o = { 
                            r_cs_max,                   //bits[31:16]
                            2'h0,                       //bits[15:14]
                            r_en_latency_additional,    //bits[13]
                            r_variable_latency_check,   //bits[12:11]
                            r_rwds_delay_line,          //bits[10:8]
                            r_read_write_recovery,      //bits[7:4]
                            r_latency_access            //bits[3:0]
                        };
        default:
            cfg_data_o = 'h0;
        endcase
    end

    assign cfg_ready_o  = 1'b1;


endmodule 
