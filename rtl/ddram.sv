//
// ddram.v
//
// DE10-nano DDR3 memory interface
//
// Copyright (c) 2017 Sorgelig
//
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version. 
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License 
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// ------------------------------------------
//

// 8-bit version

module ddram
(
	input         reset,
	input         DDRAM_CLK,

	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	input  [27:0] addr,        // 256MB at the end of 1GB
	output  [7:0] dout,        // data output to cpu
	input   [7:0] din,         // data input from cpu
	input         we,          // cpu requests write
	input         rd,          // cpu requests read
	output        ready,       // dout is valid. Ready to accept new read/write.

	// ch1 - 64-bit interface for SaveStates
	input  [27:1] ch1_addr,
	output [63:0] ch1_dout,
	input  [63:0] ch1_din,
	input         ch1_req,
	input         ch1_rnw,
	input   [7:0] ch1_be,
	output        ch1_ready
);

assign DDRAM_BURSTCNT = 1;

reg [63:0] ram_q[2];
reg [63:0] ram_data;
reg [27:0] ram_address;
reg        ram_read = 0;
reg        ram_write = 0;
reg  [7:0] ram_be;
reg  [1:0] ch_ready;

assign DDRAM_BE   = ram_read ? 8'hFF : ram_be;
assign DDRAM_ADDR = {4'b0011, ram_address[27:3]}; // Base 0x30000000
assign DDRAM_RD   = ram_read;
assign DDRAM_DIN  = ram_data;
assign DDRAM_WE   = ram_write;

assign dout      = ram_q[0][(addr[2:0]*8) +: 8];
assign ready     = ch_ready[0];

assign ch1_dout  = ram_q[1];
assign ch1_ready = ch_ready[1];

reg       state = 0;
reg       ch = 0;
reg [1:0] ch_rq;

always @(posedge DDRAM_CLK) begin
	reg old_rd, old_we;
	
	ch_rq[1] <= ch_rq[1] | ch1_req;
	
	old_rd <= rd;
	old_we <= we;
	if (~old_rd && rd) ch_rq[0] <= 1;
	if (~old_we && we) ch_rq[0] <= 1;

	ch_ready <= 0;

	if (!DDRAM_BUSY) begin
		ram_write <= 0;
		ram_read  <= 0;

		case(state)
			0: begin
				if (ch_rq[1] || ch1_req) begin
					ch_rq[1]    <= 0;
					ch          <= 1;
					ram_data    <= ch1_din;
					ram_be      <= ch1_be;
					ram_address <= {ch1_addr, 1'b0};
					
					if (!ch1_rnw) begin
						ram_write   <= 1;
						ch_ready[1] <= 1;
					end else begin
						ram_read    <= 1;
						state       <= 1;
					end
				end
				else if (ch_rq[0]) begin
					ch_rq[0]    <= 0;
					ch          <= 0;
					ram_address <= addr;
					
					if (we) begin
						ram_data    <= {8{din}};
						ram_be      <= (1'b1 << addr[2:0]);
						ram_write   <= 1;
						ch_ready[0] <= 1;
					end else begin
						ram_read    <= 1;
						state       <= 1;
					end
				end
			end

			1: begin
				if (DDRAM_DOUT_READY) begin
					ram_q[ch] <= DDRAM_DOUT;
					ch_ready[ch] <= 1;
					state <= 0;
				end
			end
		endcase
	end
end

endmodule
