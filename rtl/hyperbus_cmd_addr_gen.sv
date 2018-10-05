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

module cmd_addr_gen #(
)(
	input logic              rw_i,              //1-read, 0-write
	input logic              address_space_i,   //0-memory, 1-register
	input logic              burst_type_i,      //0-wrapped, 1-linear
	input logic  [31:0]      address_i,
	
	output logic [47:0]      cmd_addr_o
);

	assign cmd_addr_o[47] = rw_i;
	assign cmd_addr_o[46] = address_space_i;
	assign cmd_addr_o[45] = burst_type_i;
	assign cmd_addr_o[44:16] = address_i[31:3];
	assign cmd_addr_o[15:3] = '0;
	assign cmd_addr_o[2:0] = address_i[2:0];

endmodule
