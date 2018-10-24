module hyperbus_delay_line (
    input        in,
    output       out,
    input [2:0] delay
);

    // assign out = in;

    //assign #(1ns) out = in; 

    `ifndef SYNTHESIS

        assign #(1.5ns) out = in; 

    `else
    

        logic l1_left;
        logic l1_right;

        logic l2_0;
        logic l2_1;
        logic l2_2;
        logic l2_3;

    // Level 0
        pulp_clock_mux2 i_clk_mux_top
        (
            .clk_o(out),
            .clk0_i(l1_left),
            .clk1_i(l1_right),
            .clk_sel_i(delay[2])
        );

    // Level 1
        pulp_clock_mux2 i_clk_mux_l1_left 
        (
            .clk0_i( l2_0     ),
            .clk1_i( l2_1     ),
            .clk_sel_i( delay[1] ),
            .clk_o( l1_left  )
        );

        pulp_clock_mux2 i_clk_mux_l1_right 
        (
            .clk0_i( l2_2     ),
            .clk1_i( l2_3     ),
            .clk_sel_i( delay[1] ),
            .clk_o( l1_right )
        );

    //Level 2
        pulp_clock_mux2 i_clk_mux_l2_0
        (
            .clk0_i( in       ),
            .clk1_i( in       ),
            .clk_sel_i( delay[0] ),
            .clk_o( l2_0     )
        );

        pulp_clock_mux2 i_clk_mux_l2_1
        (
            .clk0_i( in       ),
            .clk1_i( in       ),
            .clk_sel_i( delay[0] ),
            .clk_o( l2_1     )
        );

        pulp_clock_mux2 i_clk_mux_l2_2
        (
            .clk0_i( in       ),
            .clk1_i( in       ),
            .clk_sel_i( delay[0] ),
            .clk_o( l2_2     )
        );

        pulp_clock_mux2 i_clk_mux_l2_3
        (
            .clk0_i( in       ),
            .clk1_i( in       ),
            .clk_sel_i( delay[0] ),
            .clk_o( l2_3     )
        );

    `endif

endmodule
