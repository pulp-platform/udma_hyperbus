module hyperbus_clock_diff_out
(
    input  logic in_i,
    input  logic en_i, //high enable
    output logic out_o,
    output logic out_no
);

    pulp_clock_gating hyper_ck_gating (
        .clk_i     ( in_i  ),
        .en_i      ( en_i  ),
        .test_en_i ( 1'b0  ),
        .clk_o     ( out_o )
    ); 

    pulp_clock_inverter hyper_ck_no_inv (
        .clk_i ( out_o  ),
        .clk_o ( out_no )
    );
    
endmodule
