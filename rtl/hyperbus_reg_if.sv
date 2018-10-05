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
`define REG_MEM_CFG1     5'b01011 //BASEADDR+0x2C
`define REG_MEM_CFG2     5'b01100 //BASEADDR+0x30
`define REG_MEM_CFG3     5'b01101 //BASEADDR+0x34
`define REG_MEM_CFG4     5'b01110 //BASEADDR+0x38
`define REG_MEM_CFG5     5'b01111 //BASEADDR+0x3C
`define REG_MEM_CFG6     5'b10000 //BASEADDR+0x40
`define REG_MEM_CFG7     5'b10001 //BASEADDR+0x44

module hyper_reg_if #(
    parameter L2_AWIDTH_NOAL = 12,
    parameter FC_AWIDTH_NOAL = 12,
    parameter TRANS_SIZE     = 16
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

    output logic                [1:0] cfg_wrap_size0_o,
    output logic                [1:0] cfg_wrap_size1_o,
    output logic                [7:0] cfg_mbr0_o,
    output logic                [7:0] cfg_mbr1_o,
    output logic                [3:0] cfg_latency0_o,
    output logic                [3:0] cfg_latency1_o,
    output logic                [3:0] cfg_rd_cshi0_o,
    output logic                [3:0] cfg_rd_cshi1_o,
    output logic                [3:0] cfg_rd_css0_o,
    output logic                [3:0] cfg_rd_css1_o,
    output logic                [3:0] cfg_rd_csh0_o,
    output logic                [3:0] cfg_rd_csh1_o,
    output logic                [3:0] cfg_wr_cshi0_o,
    output logic                [3:0] cfg_wr_cshi1_o,
    output logic                [3:0] cfg_wr_css0_o,
    output logic                [3:0] cfg_wr_css1_o,
    output logic                [3:0] cfg_wr_csh0_o,
    output logic                [3:0] cfg_wr_csh1_o,
    output logic                [8:0] cfg_rd_max_length0_o,
    output logic                [8:0] cfg_rd_max_length1_o,
    output logic                [8:0] cfg_wr_max_length0_o,
    output logic                [8:0] cfg_wr_max_length1_o,
    output logic                [2:0] cfg_rds_delay_adj_o,

    output logic                      cfg_acs0_o,
    output logic                      cfg_tco0_o,
    output logic                      cfg_dt0_o,
    output logic                      cfg_crt0_o,
    output logic                      cfg_rd_max_len_en0_o,
    output logic                      cfg_wr_max_len_en0_o,

    output logic                      cfg_acs1_o,
    output logic                      cfg_tco1_o,
    output logic                      cfg_dt1_o,
    output logic                      cfg_crt1_o,
    output logic                      cfg_rd_max_len_en1_o,
    output logic                      cfg_wr_max_len_en1_o,

    output logic               [31:0] cfg_hyper_addr_o,
    output logic                [8:0] cfg_hyper_size_o,
    output logic                      cfg_hyper_rwn_o,
    output logic                      cfg_hyper_valid_o,
    input  logic                      cfg_hyper_ready_i

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

    logic                [4:0] s_wr_addr;
    logic                [4:0] s_rd_addr;

    logic               [31:0] r_startaddr;

    logic                [1:0] r_wrap_size0;
    logic                [1:0] r_wrap_size1;
    logic                [7:0] r_mbr0;
    logic                [7:0] r_mbr1;
    logic                [3:0] r_latency0;
    logic                [3:0] r_latency1;
    logic                [3:0] r_rd_cshi0;
    logic                [3:0] r_rd_cshi1;
    logic                [3:0] r_rd_css0;
    logic                [3:0] r_rd_css1;
    logic                [3:0] r_rd_csh0;
    logic                [3:0] r_rd_csh1;
    logic                [3:0] r_wr_cshi0;
    logic                [3:0] r_wr_cshi1;
    logic                [3:0] r_wr_css0;
    logic                [3:0] r_wr_css1;
    logic                [3:0] r_wr_csh0;
    logic                [3:0] r_wr_csh1;
    logic                [8:0] r_rd_max_length0;
    logic                [8:0] r_rd_max_length1;
    logic                [8:0] r_wr_max_length0;
    logic                [8:0] r_wr_max_length1;

    logic                [2:0] r_rds_delay_adj;

    logic                      r_acs0;
    logic                      r_acs1;
    logic                      r_tco0;
    logic                      r_tco1;
    logic                      r_dt0;
    logic                      r_dt1;
    logic                      r_crt0;
    logic                      r_crt1;
    logic                      r_rd_max_len_en0;
    logic                      r_rd_max_len_en1;
    logic                      r_wr_max_len_en0;
    logic                      r_wr_max_len_en1;

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

    assign cfg_hyper_addr_o    = r_startaddr;
    assign cfg_hyper_valid_o   = r_tx_en | r_rx_en;
    assign cfg_hyper_rwn_o     = r_rx_en;
    assign cfg_hyper_size_o    = r_rx_en ? r_rx_size[9:1] : r_tx_size[9:1];

    assign cfg_mbr0_o           = r_mbr0;                
    assign cfg_latency0_o       = r_latency0;        
    assign cfg_wrap_size0_o     = r_wrap_size0;    
    assign cfg_rd_cshi0_o       = r_rd_cshi0;        
    assign cfg_rd_css0_o        = r_rd_css0;          
    assign cfg_rd_csh0_o        = r_rd_csh0;          
    assign cfg_wr_cshi0_o       = r_wr_cshi0;        
    assign cfg_wr_css0_o        = r_wr_css0;          
    assign cfg_wr_csh0_o        = r_wr_csh0;          
    assign cfg_rd_max_length0_o = r_rd_max_length0;
    assign cfg_wr_max_length0_o = r_wr_max_length0;
    assign cfg_acs0_o           = r_acs0;                    
    assign cfg_tco0_o           = r_tco0;                    
    assign cfg_dt0_o            = r_dt0;                      
    assign cfg_crt0_o           = r_crt0;                    
    assign cfg_rd_max_len_en0_o = r_rd_max_len_en0;
    assign cfg_wr_max_len_en0_o = r_wr_max_len_en0;
    assign cfg_mbr1_o           = r_mbr1;                
    assign cfg_latency1_o       = r_latency1;        
    assign cfg_wrap_size1_o     = r_wrap_size1;    
    assign cfg_rd_cshi1_o       = r_rd_cshi1;        
    assign cfg_rd_css1_o        = r_rd_css1;          
    assign cfg_rd_csh1_o        = r_rd_csh1;          
    assign cfg_wr_cshi1_o       = r_wr_cshi1;        
    assign cfg_wr_css1_o        = r_wr_css1;          
    assign cfg_wr_csh1_o        = r_wr_csh1;          
    assign cfg_rd_max_length1_o = r_rd_max_length1;
    assign cfg_wr_max_length1_o = r_wr_max_length1;
    assign cfg_acs1_o           = r_acs1;                    
    assign cfg_tco1_o           = r_tco1;                    
    assign cfg_dt1_o            = r_dt1;                      
    assign cfg_crt1_o           = r_crt1;                    
    assign cfg_rd_max_len_en1_o = r_rd_max_len_en1;
    assign cfg_wr_max_len_en1_o = r_wr_max_len_en1;
    assign cfg_rds_delay_adj_o  = r_rds_delay_adj;

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
            r_wrap_size0     <= 'h3;
            r_wrap_size1     <= 'h3;
            r_mbr0           <= 'h0;
            r_mbr1           <= 'h1;
            r_latency0       <= 'h7;
            r_latency1       <= 'h0;
            r_rd_cshi0       <= 'h8;
            r_rd_cshi1       <= 'h8;
            r_rd_css0        <= 'h8;
            r_rd_css1        <= 'h8;
            r_rd_csh0        <= 'h8;
            r_rd_csh1        <= 'h8;
            r_wr_cshi0       <= 'h8;
            r_wr_cshi1       <= 'h8;
            r_wr_css0        <= 'h8;
            r_wr_css1        <= 'h8;
            r_wr_csh0        <= 'h8;
            r_wr_csh1        <= 'h8;
            r_rd_max_length0 <= 'h0;
            r_rd_max_length1 <= 'h0;
            r_wr_max_length0 <= 'h0;
            r_wr_max_length1 <= 'h0;
            r_acs0           <= 'h0;
            r_acs1           <= 'h0;
            r_tco0           <= 'h0;
            r_tco1           <= 'h0;
            r_dt0            <= 'h1;
            r_dt1            <= 'h1;
            r_crt0           <= 'h0;
            r_crt1           <= 'h0;
            r_rd_max_len_en0 <= 'h0;
            r_rd_max_len_en1 <= 'h0;
            r_wr_max_len_en0 <= 'h0;
            r_wr_max_len_en1 <= 'h0;
            r_rds_delay_adj  <= 'h0;
        end
        else
        begin
            r_rx_en          =  'h0;
            r_rx_clr         =  'h0;
            r_tx_en          =  'h0;
            r_tx_clr         =  'h0;

            if (cfg_valid_i & ~cfg_rwn_i)
            begin
                case (s_wr_addr)
                `REG_MEM_CFG0:
                begin
                    r_mbr0       <= cfg_data_i[7:0];
                    r_latency0   <= cfg_data_i[11:8];
                    r_wrap_size0 <= cfg_data_i[13:12];
                end
                `REG_MEM_CFG1:
                begin
                    r_rd_cshi0   <= cfg_data_i[3:0];
                    r_rd_css0    <= cfg_data_i[7:4];
                    r_rd_csh0    <= cfg_data_i[11:8];
                    r_wr_cshi0   <= cfg_data_i[15:12];
                    r_wr_css0    <= cfg_data_i[19:16];
                    r_wr_csh0    <= cfg_data_i[23:20];
                end
                `REG_MEM_CFG2:
                begin
                    r_rd_max_length0 <= cfg_data_i[8:0];
                    r_wr_max_length0 <= cfg_data_i[24:16];
                end
                `REG_MEM_CFG3:
                begin
                    r_acs0           <= cfg_data_i[0];
                    r_tco0           <= cfg_data_i[1];
                    r_dt0            <= cfg_data_i[2];
                    r_crt0           <= cfg_data_i[3];
                    r_rd_max_len_en0 <= cfg_data_i[4];
                    r_wr_max_len_en0 <= cfg_data_i[5];
                    r_rds_delay_adj  <= cfg_data_i[10:8];
                end
                `REG_MEM_CFG4:
                begin
                    r_mbr1       <= cfg_data_i[7:0];
                    r_latency1   <= cfg_data_i[11:8];
                    r_wrap_size1 <= cfg_data_i[13:12];
                end
                `REG_MEM_CFG5:
                begin
                    r_rd_cshi1   <= cfg_data_i[3:0];
                    r_rd_css1    <= cfg_data_i[7:4];
                    r_rd_csh1    <= cfg_data_i[11:8];
                    r_wr_cshi1   <= cfg_data_i[15:12];
                    r_wr_css1    <= cfg_data_i[19:16];
                    r_wr_csh1    <= cfg_data_i[23:20];
                end
                `REG_MEM_CFG6:
                begin
                    r_rd_max_length1 <= cfg_data_i[8:0];
                    r_wr_max_length1 <= cfg_data_i[24:16];
                end
                `REG_MEM_CFG7:
                begin
                    r_acs1           <= cfg_data_i[0];
                    r_tco1           <= cfg_data_i[1];
                    r_dt1            <= cfg_data_i[2];
                    r_crt1           <= cfg_data_i[3];
                    r_rd_max_len_en1 <= cfg_data_i[4];
                    r_wr_max_len_en1 <= cfg_data_i[5];
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
            cfg_data_o = {19'h0,r_wrap_size0,r_latency0,r_mbr0};
        `REG_MEM_CFG1:
            cfg_data_o = {8'h0,r_wr_csh0,r_wr_css0,r_wr_cshi0,r_rd_csh0,r_rd_css0,r_rd_cshi0};
        `REG_MEM_CFG2:
            cfg_data_o = {7'h0,r_wr_max_length0,7'h0,r_rd_max_length0};
        `REG_MEM_CFG3:
            cfg_data_o = {26'h0,r_wr_max_len_en0,r_rd_max_len_en0,r_crt0,r_dt0,r_tco0,r_acs0};
        `REG_MEM_CFG4:
            cfg_data_o = {19'h0,r_wrap_size1,r_latency1,r_mbr1};
        `REG_MEM_CFG5:
            cfg_data_o = {8'h0,r_wr_csh1,r_wr_css1,r_wr_cshi1,r_rd_csh1,r_rd_css1,r_rd_cshi1};
        `REG_MEM_CFG6:
            cfg_data_o = {7'h0,r_wr_max_length1,7'h0,r_rd_max_length1};
        `REG_MEM_CFG7:
            cfg_data_o = {26'h0,r_wr_max_len_en1,r_rd_max_len_en1,r_crt1,r_dt1,r_tco1,r_acs1};
        default:
            cfg_data_o = 'h0;
        endcase
    end

    assign cfg_ready_o  = 1'b1;


endmodule 
