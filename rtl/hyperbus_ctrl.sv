`define MODE_NORMAL      3'h0
`define MODE_REG         3'h1
`define MODE_2D          3'h2

module hyperbus_ctrl #(
    parameter TRANS_DATA_SIZE = 16,
    parameter TRANS_SIZE      = 16
)
(
    input  logic                       clk_i,
    input  logic                       rst_ni,

    input  logic                [15:0] arg_data_i,
    input  logic                       arg_valid_i,
    output logic                       arg_ready_o,

    input  logic [TRANS_DATA_SIZE-1:0] trans_data_i,
    input  logic                       trans_valid_i,
    output logic                       trans_ready_o,

    input  logic                [31:0] tx_fifo_data_i,
    input  logic                       tx_fifo_valid_i,
    input  logic                       tx_fifo_ready_o,

    output logic                [16:0] tx_phy_data_o,
    output logic                       tx_phy_valid_o,
    output logic                 [1:0] tx_phy_strb_o,
    input  logic                       tx_phy_ready_i,

    output logic                [31:0] rx_fifo_data_o,
    output logic                       rx_fifo_valid_o,
    input  logic                       rx_fifo_ready_i,

    input  logic                [16:0] rx_phy_data_i,
    input  logic                       rx_phy_valid_i,
    output logic                       rx_phy_ready_o,

    output logic                       trans_phy_valid_o,
    input  logic                       trans_phy_ready_i,
    output logic                [31:0] trans_phy_address_o,
    output logic                 [1:0] trans_phy_cs_o,
    output logic                       trans_phy_write_o,
    output logic      [TRANS_SIZE-1:0] trans_phy_burst_o,
    output logic                 [1:0] trans_phy_burst_type_o,
    output logic                       trans_phy_address_space_o
 
);
    logic                  s_trans_read;
    logic [TRANS_SIZE-1:0] s_trans_size;
    logic           [31:0] s_trans_addr;
    logic            [3:0] s_trans_mode;

    logic                  s_trans_phy_write;
    logic                  s_trans_phy_addr_space;
    logic                  s_trans_phy_burst_type;
    logic [TRANS_SIZE-1:0] s_trans_phy_size;
    logic           [31:0] s_trans_phy_addr;

    logic           [15:0] r_trans_data;
    logic            [1:0] r_byte_cnt;
    logic            [1:0] s_byte_cnt;

    logic                  s_sample_data;
    logic                  s_sample_cnt;

    enum logic [2:0] { ST_IDLE, ST_RX_LSB, ST_RX_SEND, ST_TX_SEND} r_state,s_state_next;

    assign s_trans_mode = trans_data_i[3:0];
    assign s_trans_read = trans_data_i[4];
    assign s_trans_size = trans_data_i[5+TRANS_SIZE-1:5];
    assign s_trans_addr = trans_data_i[TRANS_DATA_SIZE-1:TRANS_DATA_SIZE-32];
    assign s_trans_na   = s_trans_addr[0]; //if first bit of address is not 0 then it is non aligned

    assign trans_phy_address_o       = {1'b0,s_trans_phy_addr[31:1]}; //shift by 1 since addr in hyper are aligned to 16bits
    assign trans_phy_burst_o         = s_trans_phy_size;
    assign trans_phy_address_space_o = s_trans_phy_addr_space;
    assign trans_phy_burst_type_o    = s_trans_phy_burst_type;
    assign trans_phy_write_o         = s_trans_phy_write;

  always_comb
  begin
    s_state_next           = r_state;
    trans_ready_o          = 1'b0;
    trans_phy_valid_o      = 1'b0;
    s_trans_phy_addr       = s_trans_addr;
    s_trans_phy_size       = s_trans_size;
    s_trans_phy_addr_space = 1'b0;
    s_trans_phy_burst_type = 2'b01;
    s_trans_phy_write      = ~s_trans_read;
    s_sample_data          = 1'b0;
    s_sample_cnt           = 1'b0;
    s_byte_cnt             = r_byte_cnt;
    s_trans_data           = r_trans_data;
    case(r_state)
        ST_IDLE:
        begin
            if(trans_valid_i)
            begin
                if(trans_phy_ready_i)
                begin
                    if(s_trans_na)
                        s_byte_cnt = 2'h1;
                    else
                        s_byte_cnt = 2'h0;
                    s_sample_cnt      = 1'b1;
                    trans_phy_valid_o = 1'b1;
                    s_state_next = ST_TX_SEND;
                end
            end
        end
        ST_TX_SEND:
        begin
        end
    endcase // r_state
  end

endmodule // hyperbus_ctrl
