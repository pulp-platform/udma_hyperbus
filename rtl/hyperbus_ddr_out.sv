// Copyright (C) 2017-2018 ETH Zurich, University of Bologna
// All rights reserved.
//
// This code is under development and not yet released to the public.
// Until it is released, the code is under the copyright of ETH Zurich and
// the University of Bologna, and may contain confidential and/or unpublished
// work. Any reuse/redistribution is strictly forbidden without written
// permission from ETH Zurich.

/// A single to double data rate converter.

module ddr_out #(
    parameter logic INIT = 1'b0
)(
    input  logic clk_i,
    input  logic rst_ni,
    input  logic d0_i,
    input  logic d1_i,
    output logic q_o
);
    logic q0;
    logic q1;

    pulp_clock_mux2 ddrmux (
        .clk_o     ( q_o   ),
        .clk0_i    ( q1    ),
        .clk1_i    ( q0    ),
        .clk_sel_i ( clk_i )
    );

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
            q0 <= INIT;
            q1 <= INIT;
        end else begin
            q0 <= d0_i;
            q1 <= d1_i;
        end
    end

endmodule
