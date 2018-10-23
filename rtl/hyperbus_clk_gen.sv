// Copyright (C) 2017 ETH Zurich, University of Bologna
// All rights reserved.
//
// This code is under development and not yet released to the public.
// Until it is released, the code is under the copyright of ETH Zurich and
// the University of Bologna, and may contain confidential and/or unpublished
// work. Any reuse/redistribution is strictly forbidden without written
// permission from ETH Zurich.

// Description: Generates 4 phase shifted clocks out of one faster clock
module hyperbus_clk_gen (
    input  logic clk_i,     // input clock
    input  logic rst_ni,
    input  logic dft_test_mode_i,
    output logic clk0_o,    // have the input clock - 0deg phase shift
    output logic clk90_o,   // have the input clock - 90deg phase shift
    output logic clk180_o,  // have the input clock - 180deg phase shift
    output logic clk270_o   // have the input clock - 270deg phase shift
);

`ifndef PULP_FPGA_EMUL
    logic r_clk0_o;
    logic r_clk90_o;
    logic r_clk180_o;
    logic r_clk270_o;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if(~rst_ni) begin
            r_clk0_o   <= 0;
            r_clk180_o <= 1;
        end else begin
            r_clk0_o <= ~r_clk0_o;
            r_clk180_o <= r_clk90_o;
        end
    end

    always_ff @(negedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
            r_clk90_o  <= 0;
            r_clk270_o <= 1;
        end else begin
            r_clk90_o  <= r_clk0_o;
            r_clk270_o <= r_clk180_o;
        end
    end

    pulp_clock_mux2 i_clk0_dft_mux
    (
        .clk_o(clk0_o),
        .clk0_i(r_clk0_o),
        .clk1_i(clk_i),
        .clk_sel_i(dft_test_mode_i)
    );

    pulp_clock_mux2 i_clk90_dft_mux
    (
        .clk_o(clk90_o),
        .clk0_i(r_clk90_o),
        .clk1_i(clk_i),
        .clk_sel_i(dft_test_mode_i)
    );

    pulp_clock_mux2 i_clk180_dft_mux
    (
        .clk_o(clk180_o),
        .clk0_i(r_clk180_o),
        .clk1_i(clk_i),
        .clk_sel_i(dft_test_mode_i)
    );

    pulp_clock_mux2 i_clk270_dft_mux
    (
        .clk_o(clk270_o),
        .clk0_i(r_clk270_o),
        .clk1_i(clk_i),
        .clk_sel_i(dft_test_mode_i)
    );

`else
   assign clk0_o = clk_i;
   assign clk90_o = clk_i;
   assign clk180_o = clk_i;
   assign clk270_o = clk_i;
`endif
endmodule

