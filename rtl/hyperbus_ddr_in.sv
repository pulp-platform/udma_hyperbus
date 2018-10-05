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
// Description: Generate the Command-Address to start a transaction

module ddr_in #(
)(
    input logic        clk_i,
    input logic        data_i,
    input logic        rst_ni,
    input logic        enable,
    
    output logic [1:0] data_o
);
    logic ddr_neg;
    logic ddr_pos;

    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_ddr_pos
        if(~rst_ni) begin
            ddr_pos <= 1'b0;
        end else if (enable) begin
            ddr_pos <= data_i;
        end
    end
    
    always_ff @(negedge clk_i or negedge rst_ni) begin : proc_ddr_neg
        if(~rst_ni) begin
            ddr_neg <= 1'b0;
        end else if (enable) begin
            ddr_neg <= data_i;
        end
    end

    assign data_o[0] = ddr_neg;
    assign data_o[1] = ddr_pos;
endmodule

