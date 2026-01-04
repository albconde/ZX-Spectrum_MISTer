

module savestates_z80
 (
	input  wire        clk_sys,
	input  wire        reset,

	input  wire  [1:0] ss_slot,
	input  wire        ss_save,
	input  wire        ss_load,
	input  wire        core_paused,
	input  wire        cpu_m1,
	input  wire        is_48k,
	input  wire  [7:0]  z80_hw_mode,
	input  wire [23:0]  z80_tstates,

	input  wire [211:0] cpu_reg,
	input  wire  [2:0] border_color,
	input  wire  [7:0] page_reg,
	input  wire  [7:0] page_reg_plus3,
	input  wire  [7:0] psg_reg_addr,
	input  wire  [7:0] psg_reg_shadow [16],

	input  wire        ram_ready,
	input  wire  [7:0] ram_dout,

	input  wire [63:0] ss_ddr_dout,
	input  wire        ss_ddr_ready,

	output wire [27:1] ss_ddr_addr,
	output wire [63:0] ss_ddr_din,
	output wire        ss_ddr_req,
	output wire        ss_ddr_rnw,
	output wire  [7:0] ss_ddr_be,

	output wire        ss_active,
	output wire [24:0] ss_ram_addr,
	output reg         ss_ram_rd,
	output reg         ss_ram_we,
	output reg   [3:0] ss_state,

	output reg   [7:0] ss_data_save,
	output reg   [7:0] ss_data_load,
	output reg   [7:0] ss_data_latch,

	output reg [211:0] ss_reg_accum,
	output reg         ss_reg_set,
	output reg   [2:0] ss_border,
	output reg   [7:0] ss_7ffd,
	output reg   [7:0] ss_1ffd,

	output reg   [7:0] ss_info,
	output reg         ss_info_req,
	output wire        ss_cpu_reset,
	output reg   [7:0] ss_psg_sel,
	output reg   [7:0] ss_psg_regs [16],
	output reg         ss_psg_restore,
	output reg         ss_is_48k,
	output reg         ss_is_plus3,
	output reg   [1:0] led_user_mode,
	output reg   [1:0] led_disk_mode
);

	// CPU-only reset pulse during savestate LOAD.
	localparam bit SS_CPU_RESET_ON_LOAD = 1'b1;
	reg  [2:0] ss_cpu_reset_cnt;
	assign ss_cpu_reset = (ss_cpu_reset_cnt != 3'd0);

	// Savestate state encoding
	localparam [3:0] SS_IDLE              = 4'd0;
	localparam [3:0] SS_SAVE              = 4'd1;
	localparam [3:0] SS_LOAD              = 4'd2;
	localparam [3:0] SS_WAIT_ACK          = 4'd3;
	localparam [3:0] SS_SAVE_HEADER       = 4'd4;
	localparam [3:0] SS_SAVE_EXT_HEADER   = 4'd5;
	localparam [3:0] SS_SAVE_BLOCK_HEADER = 4'd6;
	localparam [3:0] SS_WRITE_RAM         = 4'd7;
	localparam [3:0] SS_PRE_READ          = 4'd8;
	localparam [3:0] SS_SAVE_WAIT         = 4'd9;
	localparam [3:0] SS_LOAD_FETCH        = 4'd10;
	localparam [3:0] SS_LOAD_DONE         = 4'd11;
	localparam [3:0] SS_WAIT_PAUSE        = 4'd12;

	// LED indicator FSM
	localparam [1:0] LED_OFF   = 2'd0;
	localparam [1:0] LED_BLINK = 2'd1;
	localparam [1:0] LED_ON    = 2'd2;
	localparam [1:0] LED_PULSE = 2'd3;

	// Savestate FSM variables
	reg [17:0] ss_addr_counter;
	reg [17:0] ss_write_addr;
	reg [31:0] ss_total_size;
	reg [63:0] ss_ddr_data_buffer;
	reg [2:0]  ss_byte_count;

	// Optional stability: arm SAVE/LOAD in SS_IDLE and only leave SS_IDLE
	// (asserting ss_active) on a cpu_m1 rising edge.
	reg        ss_pending_save;
	reg        ss_pending_load;
	reg        cpu_m1_d;
	reg [25:0] ss_m1_wait_cnt;
	wire       cpu_m1_rise = cpu_m1 & ~cpu_m1_d;
	localparam [25:0] SS_M1_WAIT_MAX = 26'd1000000; // ~20ms at 50MHz (rough)

	// SDRAM handshake tracking
	reg        ss_wr_wait;
	reg        ss_wr_seen_busy;
	reg        ss_rd_seen_busy;
	reg  [1:0] ss_rd_hi_cnt;

	reg        ss_saving;
	reg        ss_header_mode;
	localparam [7:0] SS_FORMAT_ID = 8'd1; // 1 = Z80
	reg  [23:0] ss_seq_hi = 24'd1;
	wire [31:0] ss_seq_num = {ss_seq_hi, SS_FORMAT_ID};
	reg  [7:0] ss_wait_timer;
	reg [19:0] ss_post_load_timer;
	reg [25:0] ss_stall_cnt;
	localparam [25:0] SS_STALL_MAX = 26'd50000000;
	reg [15:0] ss_freeze_timeout;
	localparam [15:0] SS_FREEZE_TIMEOUT_MAX = 16'd10000;
	reg        ss_save_old, ss_load_old;

	reg  [3:0] ss_block_idx;
	reg [14:0] ss_block_addr;
	reg  [2:0] ss_block_hdr_idx;
	reg        ss_load_in_block_hdr;
	reg  [7:0] ss_load_page;
	reg [63:0] ss_block_hdr;
	reg [31:0] ss_file_seq;
	reg [31:0] ss_file_payload_words;
	reg        ss_file_is_48k;
	reg        ss_file_is_plus3;
	reg [15:0] ss_file_ext_len;
	reg [17:0] ss_hdr_bytes_load;
	reg        ss_header_valid;

	reg  [2:0] ss_info_hold_cnt;
	reg  [7:0] ss_info_hold_val;
	reg  [3:0] ss_return_state;
	reg  [1:0] ss_rd_wait;
	reg [23:0] ss_prev_word;
	reg        ss_prev_word_valid;
	reg  [1:0] ss_reg_hold;
	reg        ss_reg_pending;

	integer psg_i;

	function automatic bit _valid_z80_page(input [7:0] p);
		begin
			_valid_z80_page = (p == 8'd4) || (p == 8'd5) || (p == 8'd8) || ((p >= 8'd3) && (p <= 8'd10));
		end
	endfunction

	function automatic [17:0] _align8(input [17:0] v);
		begin
			_align8 = (v + 18'd7) & ~18'd7;
		end
	endfunction

	// DDR connections - sin CDC, directo
	wire [27:0] ss_slot_offset = {6'd0, ss_slot, 18'd0};
	wire [27:0] ss_byte_offset = (ss_header_mode || ss_state == SS_SAVE_HEADER) ? 28'd0 : {10'd0, ss_write_addr};
	wire [27:0] ss_abs_addr = 28'hE000000 + ss_slot_offset + ss_byte_offset;
	assign ss_ddr_addr = ss_abs_addr[27:1];
	assign ss_ddr_req = (ss_state == SS_SAVE_HEADER) ||
						(ss_state == SS_WAIT_ACK) ||
						((ss_state == SS_LOAD_FETCH) && ({14'd0, ss_write_addr} < ss_total_size));
	assign ss_ddr_rnw = !ss_saving;
	assign ss_ddr_be  = 8'hFF;

	wire [31:0] ss_payload_bytes = ss_total_size - 32'd8;
	wire [31:0] ss_payload_words = (ss_payload_bytes + 32'd3) >> 2;
	assign ss_ddr_din = (ss_state == SS_SAVE_HEADER || ss_header_mode) ? {ss_payload_words[31:0], ss_seq_num[31:0]} : ss_ddr_data_buffer;

	wire [17:0] ss_payload_addr = ss_addr_counter - 18'd8;
	wire        ss_in_mister_header = (ss_addr_counter < 18'd8);
	wire        ss_map_is_48k = ss_saving ? is_48k : ss_file_is_48k;

	// Internal Z80 stream uses a padded base header and 8-byte alignment before RAM blocks.
	// Save format: base(30)+pad(2) + ext_len(2) + ext_body(55 incl. 1FFD @ offset 86 in standard) + align pad to 8.
	// Old files may have ext_body=54 and thus no alignment padding.
	wire [17:0] ss_hdr_bytes_save = 18'd96;
	wire [17:0] ss_hdr_bytes = ss_saving ? ss_hdr_bytes_save : ss_hdr_bytes_load;

	wire [2:0] ss_ram_bank_save = ss_map_is_48k ? (ss_block_idx == 0 ? 3'd2 : ss_block_idx == 1 ? 3'd0 : 3'd5) : ss_block_idx[2:0];
	wire [2:0] ss_ram_bank_load = ss_file_is_48k
		? (ss_load_page == 8'd4 ? 3'd2 : ss_load_page == 8'd5 ? 3'd0 : ss_load_page == 8'd8 ? 3'd5 : 3'd1)
		: ((ss_load_page >= 8'd3 && ss_load_page <= 8'd10) ? (ss_load_page[2:0] - 3'd3) : 3'd0);
	wire [2:0] ss_ram_bank = ss_saving ? ss_ram_bank_save : ss_ram_bank_load;
	assign ss_ram_addr = {4'b0000, ss_ram_bank, ss_block_addr[13:0]};

	always @(*) begin
		ss_data_save = 8'h00;
		if (ss_state == SS_SAVE) begin
			case (ss_payload_addr[5:0])
				6'd0:  ss_data_save = cpu_reg[7:0];
				6'd1:  ss_data_save = cpu_reg[15:8];
				6'd2:  ss_data_save = cpu_reg[87:80];
				6'd3:  ss_data_save = cpu_reg[95:88];
				6'd4:  ss_data_save = cpu_reg[119:112];
				6'd5:  ss_data_save = cpu_reg[127:120];
				6'd6:  ss_data_save = 8'h00;
				6'd7:  ss_data_save = 8'h00;
				6'd8:  ss_data_save = cpu_reg[55:48];
				6'd9:  ss_data_save = cpu_reg[63:56];
				6'd10: ss_data_save = cpu_reg[39:32];
				6'd11: ss_data_save = cpu_reg[47:40] & 8'h7F;
				6'd12: ss_data_save = {2'b00, 1'b0, 1'b0, border_color[2:0], cpu_reg[47]};
				6'd13: ss_data_save = cpu_reg[103:96];
				6'd14: ss_data_save = cpu_reg[111:104];
				6'd15: ss_data_save = cpu_reg[151:144];
				6'd16: ss_data_save = cpu_reg[159:152];
				6'd17: ss_data_save = cpu_reg[167:160];
				6'd18: ss_data_save = cpu_reg[175:168];
				6'd19: ss_data_save = cpu_reg[183:176];
				6'd20: ss_data_save = cpu_reg[191:184];
				6'd21: ss_data_save = cpu_reg[23:16];
				6'd22: ss_data_save = cpu_reg[31:24];
				6'd23: ss_data_save = cpu_reg[199:192];
				6'd24: ss_data_save = cpu_reg[207:200];
				6'd25: ss_data_save = cpu_reg[135:128];
				6'd26: ss_data_save = cpu_reg[143:136];
				6'd27: ss_data_save = {7'b0, cpu_reg[210]};
				6'd28: ss_data_save = {7'b0, cpu_reg[211]};
				6'd29: ss_data_save = {4'b0001, 2'b0, cpu_reg[209:208]};
				6'd30: ss_data_save = 8'h00;
				6'd31: ss_data_save = 8'h00;
				default: ss_data_save = 8'h00;
			endcase
		end else if (ss_state == SS_SAVE_EXT_HEADER) begin
			case (ss_payload_addr[5:0] - 6'd32)
				6'd0:  ss_data_save = 8'd55;
				6'd1:  ss_data_save = 8'd0;
				6'd2:  ss_data_save = cpu_reg[71:64];
				6'd3:  ss_data_save = cpu_reg[79:72];
				6'd4:  ss_data_save = is_48k ? 8'd0 : z80_hw_mode;
				6'd5:  ss_data_save = page_reg;
				6'd6:  ss_data_save = 8'd0;
				6'd7:  ss_data_save = 8'b00000100;  // Bit2=1: AY sound in use
				6'd8:  ss_data_save = psg_reg_addr;
				// Bytes 9-24 (payload 41-56): AY registers 0-15
				6'd9:  ss_data_save = psg_reg_shadow[0];
				6'd10: ss_data_save = psg_reg_shadow[1];
				6'd11: ss_data_save = psg_reg_shadow[2];
				6'd12: ss_data_save = psg_reg_shadow[3];
				6'd13: ss_data_save = psg_reg_shadow[4];
				6'd14: ss_data_save = psg_reg_shadow[5];
				6'd15: ss_data_save = psg_reg_shadow[6];
				6'd16: ss_data_save = psg_reg_shadow[7];
				6'd17: ss_data_save = psg_reg_shadow[8];
				6'd18: ss_data_save = psg_reg_shadow[9];
				6'd19: ss_data_save = psg_reg_shadow[10];
				6'd20: ss_data_save = psg_reg_shadow[11];
				6'd21: ss_data_save = psg_reg_shadow[12];
				6'd22: ss_data_save = psg_reg_shadow[13];
				6'd23: ss_data_save = psg_reg_shadow[14];
				6'd24: ss_data_save = psg_reg_shadow[15];
				// Z80 v2/v3 timing (standard offsets 55..58; internal offsets 57..60)
				6'd25: ss_data_save = z80_tstates[7:0];
				6'd26: ss_data_save = z80_tstates[15:8];
				6'd27: ss_data_save = z80_tstates[23:16];
				6'd28: ss_data_save = 8'h00; // Spectator flag (unused)
				// +3: last OUT to port 0x1FFD (standard Z80 offset 86; internal offset 88)
				6'd56: ss_data_save = page_reg_plus3;
				default: ss_data_save = 8'h00;
			endcase
		end else if (ss_state == SS_SAVE_BLOCK_HEADER) begin
			case (ss_byte_count)
				3'd0,3'd1,3'd2,3'd3,3'd4: ss_data_save = 8'h00;
				3'd5: ss_data_save = 8'hFF;
				3'd6: ss_data_save = 8'hFF;
				3'd7: begin
					if (is_48k) begin
						case (ss_block_idx)
							4'd0: ss_data_save = 8'd4;
							4'd1: ss_data_save = 8'd5;
							4'd2: ss_data_save = 8'd8;
							default: ss_data_save = 8'd0;
						endcase
					end else begin
						ss_data_save = {4'b0, ss_block_idx} + 8'd3;
					end
				end
				default: ss_data_save = 8'h00;
			endcase
		end else if (ss_state == SS_SAVE_WAIT || ss_state == SS_PRE_READ) begin
			ss_data_save = ram_dout;
		end
	end

	always @(*) begin
		case (ss_byte_count)
			3'd0: ss_data_load = ss_ddr_data_buffer[7:0];
			3'd1: ss_data_load = ss_ddr_data_buffer[15:8];
			3'd2: ss_data_load = ss_ddr_data_buffer[23:16];
			3'd3: ss_data_load = ss_ddr_data_buffer[31:24];
			3'd4: ss_data_load = ss_ddr_data_buffer[39:32];
			3'd5: ss_data_load = ss_ddr_data_buffer[47:40];
			3'd6: ss_data_load = ss_ddr_data_buffer[55:48];
			3'd7: ss_data_load = ss_ddr_data_buffer[63:56];
		endcase
	end

	always @(*) begin
		if (ss_info_hold_cnt != 3'd0) ss_info = ss_info_hold_val;
		else ss_info = {4'h0, ss_slot + 1'b1};
	end

	// CPU reset pulse generation during LOAD
	always @(posedge clk_sys) begin
		if (reset) begin
			ss_cpu_reset_cnt <= 3'd0;
		end else begin
			if (SS_CPU_RESET_ON_LOAD && ss_header_valid && (ss_state == SS_LOAD || ss_state == SS_LOAD_FETCH) && ss_payload_addr < ss_hdr_bytes) begin
				if (ss_cpu_reset_cnt != 3'd7) ss_cpu_reset_cnt <= 3'd7;
			end else if (ss_cpu_reset_cnt != 3'd0) begin
				ss_cpu_reset_cnt <= ss_cpu_reset_cnt - 3'd1;
			end
		end
	end

	// Main savestate FSM
	always @(posedge clk_sys) begin
		ss_save_old <= ss_save;
		ss_load_old <= ss_load;

		if (reset) begin
			ss_reg_set <= 0;
			ss_reg_accum <= 0;
			ss_border <= 0;
			ss_7ffd <= 0;
			ss_1ffd <= 0;
			ss_is_plus3 <= 1'b0;
			ss_reg_hold <= 0;
			ss_reg_pending <= 1'b0;
			ss_file_is_48k <= 1'b1;
			ss_file_is_plus3 <= 1'b0;
			ss_file_ext_len <= 16'd0;
			ss_hdr_bytes_load <= 18'd88;
			ss_header_valid <= 1'b0;
			ss_info_req <= 0;
			ss_state <= SS_IDLE;
			ss_wr_wait <= 0;
			ss_wr_seen_busy <= 0;
			ss_rd_seen_busy <= 0;
			ss_rd_wait <= 0;
			ss_rd_hi_cnt <= 0;
			ss_prev_word <= 24'd0;
			ss_prev_word_valid <= 1'b0;
			ss_ram_rd <= 0;
			ss_ram_we <= 0;
			ss_addr_counter <= 0;
			ss_write_addr <= 0;
			ss_byte_count <= 0;
			ss_saving <= 0;
			ss_header_mode <= 0;
			ss_seq_hi <= 24'd1;
			ss_save_old <= 0;
			ss_load_old <= 0;
			ss_block_idx <= 0;
			ss_block_addr <= 0;
			ss_block_hdr_idx <= 0;
			ss_load_in_block_hdr <= 0;
			ss_load_page <= 0;
			ss_block_hdr <= 64'd0;
			ss_file_seq <= 0;
			ss_file_payload_words <= 0;
			ss_total_size <= 0;
			ss_post_load_timer <= 0;
			ss_stall_cnt <= 0;
			ss_freeze_timeout <= 0;
			led_user_mode <= LED_OFF;
			led_disk_mode <= LED_OFF;
			ss_info_hold_cnt <= 0;
			ss_info_hold_val <= 0;
			ss_pending_save <= 1'b0;
			ss_pending_load <= 1'b0;
			cpu_m1_d <= 1'b0;
			ss_m1_wait_cnt <= 26'd0;
			ss_psg_restore <= 1'b0;
			ss_psg_sel <= 8'h00;
			for (psg_i = 0; psg_i < 16; psg_i = psg_i + 1) begin
				ss_psg_regs[psg_i] <= 8'h00;
			end
		end else begin
			cpu_m1_d <= cpu_m1;
			// Clear PSG restore flag after one cycle
			if (ss_psg_restore) ss_psg_restore <= 0;

			ss_info_req <= 0;
			ss_reg_set <= |ss_reg_hold;
			if (ss_reg_hold) ss_reg_hold <= ss_reg_hold - 1'd1;
			if (ss_info_hold_cnt != 3'd0) ss_info_hold_cnt <= ss_info_hold_cnt - 3'd1;
			ss_ram_rd <= 0;
			ss_ram_we <= 0;

			if (ss_load & ~ss_load_old) begin
				ss_file_is_48k <= 1'b1;
				ss_file_is_plus3 <= 1'b0;
				ss_file_ext_len <= 16'd0;
				ss_hdr_bytes_load <= 18'd88;
				ss_1ffd <= 8'h00;
				ss_is_plus3 <= 1'b0;
				ss_header_valid <= 1'b0;
				ss_reg_hold <= 0;
				ss_reg_pending <= 1'b0;
				ss_reg_set <= 0;
			end

			if (ss_state != SS_LOAD_FETCH && ss_state != SS_WRITE_RAM) ss_stall_cnt <= 0;

			case (ss_state)
				SS_IDLE: begin
					ss_stall_cnt <= 0;
					// Reflect armed requests on LEDs while still idle.
					if (ss_pending_save) begin
						led_user_mode <= LED_ON;
						led_disk_mode <= LED_OFF;
					end else if (ss_pending_load) begin
						led_user_mode <= LED_BLINK;
						led_disk_mode <= LED_BLINK;
					end else begin
						led_user_mode <= LED_OFF;
						led_disk_mode <= LED_OFF;
					end

					// Arm new requests (do not leave SS_IDLE yet).
					if (ss_save & ~ss_save_old) begin
						ss_pending_save <= 1'b1;
						ss_pending_load <= 1'b0;
						ss_m1_wait_cnt <= 26'd0;
					end else if (ss_load & ~ss_load_old) begin
						ss_pending_load <= 1'b1;
						ss_pending_save <= 1'b0;
						ss_m1_wait_cnt <= 26'd0;
					end

					// Start once we see an instruction-boundary (cpu_m1 rising edge), or if the
					// core is already paused. Timeout is a safety valve to avoid deadlock.
					if (ss_pending_save) begin
						if (core_paused || cpu_m1_rise || (ss_m1_wait_cnt > SS_M1_WAIT_MAX)) begin
							ss_pending_save <= 1'b0;
							ss_state <= SS_WAIT_PAUSE;
							ss_saving <= 1;
							ss_prev_word_valid <= 1'b0;
							ss_freeze_timeout <= 0;
						end else begin
							ss_m1_wait_cnt <= ss_m1_wait_cnt + 1'd1;
						end
					end else if (ss_pending_load) begin
						if (core_paused || cpu_m1_rise || (ss_m1_wait_cnt > SS_M1_WAIT_MAX)) begin
							ss_pending_load <= 1'b0;
							led_user_mode <= LED_BLINK;
							led_disk_mode <= LED_BLINK;
							ss_state <= SS_LOAD_FETCH;
							ss_saving <= 0;
							ss_header_valid <= 0;
							ss_total_size <= 32'd8;
							ss_addr_counter <= 18'd0;
							ss_write_addr <= 18'd0;
							ss_byte_count <= 0;
							ss_header_mode <= 0;
							ss_ddr_data_buffer <= 64'h0;
							ss_block_idx <= 0;
							ss_block_addr <= 0;
							ss_block_hdr_idx <= 0;
							ss_load_in_block_hdr <= 0;
							ss_load_page <= 0;
							ss_block_hdr <= 64'd0;
							ss_post_load_timer <= 0;
							ss_file_seq <= 0;
							ss_file_payload_words <= 0;
						end else begin
							ss_m1_wait_cnt <= ss_m1_wait_cnt + 1'd1;
						end
					end
				end

				SS_SAVE: begin
					led_user_mode <= LED_ON;
					led_disk_mode <= LED_OFF;
					if (ss_addr_counter < ss_total_size) begin
						if (ss_payload_addr < 32) begin
							case (ss_byte_count)
								3'd0: ss_ddr_data_buffer[7:0] <= ss_data_save;
								3'd1: ss_ddr_data_buffer[15:8] <= ss_data_save;
								3'd2: ss_ddr_data_buffer[23:16] <= ss_data_save;
								3'd3: ss_ddr_data_buffer[31:24] <= ss_data_save;
								3'd4: ss_ddr_data_buffer[39:32] <= ss_data_save;
								3'd5: ss_ddr_data_buffer[47:40] <= ss_data_save;
								3'd6: ss_ddr_data_buffer[55:48] <= ss_data_save;
								3'd7: ss_ddr_data_buffer[63:56] <= ss_data_save;
							endcase
							ss_addr_counter <= ss_addr_counter + 1'd1;
							if (ss_byte_count == 3'd7) begin
								ss_state <= SS_WAIT_ACK;
								ss_return_state <= SS_SAVE;
								ss_wait_timer <= 10;
								ss_byte_count <= 0;
							end else begin
								ss_byte_count <= ss_byte_count + 1'd1;
							end
						end else if (ss_payload_addr < ss_hdr_bytes) begin
							ss_state <= SS_SAVE_EXT_HEADER;
						end else begin
							ss_state <= SS_SAVE_BLOCK_HEADER;
							ss_block_idx <= 0;
							ss_block_addr <= 0;
							ss_block_hdr_idx <= 0;
						end
					end else begin
						ss_state <= SS_SAVE_HEADER;
						ss_addr_counter <= 0;
						ss_byte_count <= 0;
						ss_header_mode <= 1;
					end
				end

				SS_SAVE_EXT_HEADER: begin
					led_user_mode <= LED_OFF;
					led_disk_mode <= LED_ON;
					if (ss_payload_addr < ss_hdr_bytes) begin
						case (ss_byte_count)
							3'd0: ss_ddr_data_buffer[7:0] <= ss_data_save;
							3'd1: ss_ddr_data_buffer[15:8] <= ss_data_save;
							3'd2: ss_ddr_data_buffer[23:16] <= ss_data_save;
							3'd3: ss_ddr_data_buffer[31:24] <= ss_data_save;
							3'd4: ss_ddr_data_buffer[39:32] <= ss_data_save;
							3'd5: ss_ddr_data_buffer[47:40] <= ss_data_save;
							3'd6: ss_ddr_data_buffer[55:48] <= ss_data_save;
							3'd7: ss_ddr_data_buffer[63:56] <= ss_data_save;
						endcase
						ss_addr_counter <= ss_addr_counter + 1'd1;
						if (ss_byte_count == 3'd7) begin
							ss_state <= SS_WAIT_ACK;
							ss_return_state <= SS_SAVE_EXT_HEADER;
							ss_wait_timer <= 10;
							ss_byte_count <= 0;
						end else begin
							ss_byte_count <= ss_byte_count + 1'd1;
						end
					end else begin
						ss_state <= SS_SAVE_BLOCK_HEADER;
						ss_block_idx <= 0;
						ss_block_addr <= 0;
						ss_block_hdr_idx <= 0;
					end
				end

				SS_SAVE_BLOCK_HEADER: begin
					led_user_mode <= LED_ON;
					led_disk_mode <= LED_ON;
					case (ss_byte_count)
						3'd0: ss_ddr_data_buffer[7:0]   <= ss_data_save;
						3'd1: ss_ddr_data_buffer[15:8]  <= ss_data_save;
						3'd2: ss_ddr_data_buffer[23:16] <= ss_data_save;
						3'd3: ss_ddr_data_buffer[31:24] <= ss_data_save;
						3'd4: ss_ddr_data_buffer[39:32] <= ss_data_save;
						3'd5: ss_ddr_data_buffer[47:40] <= ss_data_save;
						3'd6: ss_ddr_data_buffer[55:48] <= ss_data_save;
						3'd7: ss_ddr_data_buffer[63:56] <= ss_data_save;
					endcase
					ss_addr_counter <= ss_addr_counter + 1'd1;
					ss_block_hdr_idx <= ss_byte_count;
					if (ss_byte_count == 3'd7) begin
						ss_state <= SS_WAIT_ACK;
						ss_return_state <= SS_PRE_READ;
						ss_wait_timer <= 10;
						ss_byte_count <= 0;
						ss_block_addr <= 0;
					end else begin
						ss_byte_count <= ss_byte_count + 1'd1;
					end
				end

				SS_PRE_READ: begin
					led_user_mode <= LED_BLINK;
					led_disk_mode <= LED_OFF;
					ss_rd_seen_busy <= 0;
					ss_rd_hi_cnt <= 0;
					ss_rd_wait <= 2'd1;
					ss_state <= SS_SAVE_WAIT;
					ss_wait_timer <= 0;
				end

				SS_SAVE_WAIT: begin
					led_user_mode <= LED_ON;
					led_disk_mode <= ram_ready ? LED_ON : LED_OFF;
					if (ss_rd_wait != 0) ss_rd_wait <= ss_rd_wait - 1'd1;
					if (!ram_ready) ss_rd_seen_busy <= 1'b1;
					if (ram_ready && !ss_rd_seen_busy && (ss_rd_hi_cnt != 2'd3)) ss_rd_hi_cnt <= ss_rd_hi_cnt + 1'd1;
					if (ram_ready && (ss_rd_wait == 0) && (ss_rd_seen_busy || (ss_prev_word_valid && (ss_prev_word == ss_ram_addr[24:1])) || (!ss_rd_seen_busy && (ss_rd_hi_cnt == 2'd2)))) begin
						case (ss_byte_count)
							3'd0: ss_ddr_data_buffer[7:0] <= ss_data_save;
							3'd1: ss_ddr_data_buffer[15:8] <= ss_data_save;
							3'd2: ss_ddr_data_buffer[23:16] <= ss_data_save;
							3'd3: ss_ddr_data_buffer[31:24] <= ss_data_save;
							3'd4: ss_ddr_data_buffer[39:32] <= ss_data_save;
							3'd5: ss_ddr_data_buffer[47:40] <= ss_data_save;
							3'd6: ss_ddr_data_buffer[55:48] <= ss_data_save;
							3'd7: ss_ddr_data_buffer[63:56] <= ss_data_save;
						endcase
						ss_prev_word <= ss_ram_addr[24:1];
						ss_prev_word_valid <= 1'b1;
						ss_addr_counter <= ss_addr_counter + 1'd1;

						if (ss_block_addr == 15'd16383) begin
							ss_block_addr <= 0;
							if (ss_byte_count == 3'd7) begin
								ss_state <= SS_WAIT_ACK;
								ss_wait_timer <= 10;
								ss_byte_count <= 0;
								if (ss_block_idx == (is_48k ? 4'd2 : 4'd7)) begin
									ss_return_state <= SS_SAVE_HEADER;
								end else begin
									ss_block_idx <= ss_block_idx + 1'd1;
									ss_return_state <= SS_SAVE_BLOCK_HEADER;
								end
							end else begin
								ss_state <= SS_WAIT_ACK;
								ss_wait_timer <= 10;
								ss_return_state <= (ss_block_idx == (is_48k ? 4'd2 : 4'd7)) ? SS_SAVE_HEADER : SS_SAVE_BLOCK_HEADER;
								ss_byte_count <= 0;
							end
						end else begin
							ss_block_addr <= ss_block_addr + 1'd1;
							if (ss_byte_count == 3'd7) begin
								ss_state <= SS_WAIT_ACK;
								ss_return_state <= SS_PRE_READ;
								ss_wait_timer <= 10;
								ss_byte_count <= 0;
							end else begin
								ss_byte_count <= ss_byte_count + 1'd1;
								ss_state <= SS_PRE_READ;
							end
						end
					end
				end

				SS_LOAD_FETCH: begin
					led_user_mode <= LED_BLINK;
					led_disk_mode <= ss_ddr_ready ? LED_OFF : LED_BLINK;

					if (!ss_ddr_ready) begin
						if (ss_stall_cnt != SS_STALL_MAX) ss_stall_cnt <= ss_stall_cnt + 1'd1;
					end else begin
						ss_stall_cnt <= 0;
					end
					if (ss_stall_cnt == SS_STALL_MAX) begin
						led_user_mode <= LED_BLINK;
						led_disk_mode <= LED_BLINK;
						ss_state <= SS_LOAD_FETCH;
					end else if ({14'd0, ss_write_addr} >= ss_total_size) begin
						ss_state <= SS_LOAD_DONE;
					ss_post_load_timer <= 20'd50000000;  // 1 second @ 50MHz - allow hardware to fully stabilize
					end else if (ss_ddr_ready) begin
						ss_ddr_data_buffer <= ss_ddr_dout;
						ss_byte_count <= 3'd0;
						ss_state <= SS_LOAD;
						ss_write_addr <= ss_write_addr + 18'd8;
					end
				end

				SS_LOAD: begin
					led_user_mode <= LED_BLINK;
					led_disk_mode <= LED_OFF;

					if (ss_addr_counter < ss_total_size) begin
						if (ss_addr_counter < 18'd8) begin
							case (ss_addr_counter[2:0])
								3'd0: ss_file_seq[7:0] <= ss_data_load;
								3'd1: ss_file_seq[15:8] <= ss_data_load;
								3'd2: ss_file_seq[23:16] <= ss_data_load;
								3'd3: ss_file_seq[31:24] <= ss_data_load;
								3'd4: ss_file_payload_words[7:0] <= ss_data_load;
								3'd5: ss_file_payload_words[15:8] <= ss_data_load;
								3'd6: ss_file_payload_words[23:16] <= ss_data_load;
								3'd7: begin
									reg [31:0] payload_words_temp;
									reg [31:0] total_size_temp;
									payload_words_temp = {ss_data_load, ss_file_payload_words[23:0]};
									total_size_temp = 32'd8 + (payload_words_temp << 2);
									
									// Validate MiSTer header: payload_words must be reasonable
									// Valid range: minimum 22 words (88 bytes Z80 header) to maximum 65536 words (262144 bytes)
									if (payload_words_temp == 32'd0 || payload_words_temp < 32'd22 || payload_words_temp > 32'd65536) begin
										// Invalid savestate - abort load and return to idle
										ss_state <= SS_IDLE;
										ss_header_valid <= 1'b0;
										led_user_mode <= LED_OFF;
										led_disk_mode <= LED_OFF;
									end else begin
										ss_file_payload_words[31:24] <= ss_data_load;
										ss_total_size <= (total_size_temp > 32'd262144) ? 32'd262144 : total_size_temp;
										ss_header_valid <= 1'b1;
									end
								end
							endcase

							if (ss_state != SS_IDLE) begin
								ss_addr_counter <= ss_addr_counter + 1'd1;
								if (ss_byte_count == 3'd7) ss_state <= SS_LOAD_FETCH;
								else ss_byte_count <= ss_byte_count + 1'd1;
							end
						end else if (ss_payload_addr < ss_hdr_bytes) begin
							if (ss_payload_addr == (ss_hdr_bytes - 18'd1)) begin
								ss_load_in_block_hdr <= 1;
								ss_block_hdr_idx <= 0;
								ss_block_idx <= 0;
								ss_block_addr <= 0;
								ss_block_hdr <= 64'd0;
							end
							ss_addr_counter <= ss_addr_counter + 1'd1;
							if (ss_byte_count == 3'd7) ss_state <= SS_LOAD_FETCH;
							else ss_byte_count <= ss_byte_count + 1'd1;
						end else if (ss_load_in_block_hdr) begin
							reg [63:0] hdr_tmp;
							reg [7:0]  page_a;
							reg [7:0]  page_b;
							reg        page_a_ok;
							reg        page_b_ok;
							hdr_tmp = ss_block_hdr;
							hdr_tmp[ss_block_hdr_idx*8 +: 8] = ss_data_load;
							ss_block_hdr <= hdr_tmp;

							ss_addr_counter <= ss_addr_counter + 1'd1;
							if (ss_block_hdr_idx == 3'd7) begin
								page_a = hdr_tmp[56 +: 8];
								page_b = hdr_tmp[16 +: 8];
								page_a_ok = _valid_z80_page(page_a) && (hdr_tmp[39:0] == 40'd0) && (hdr_tmp[55:40] == 16'hFFFF);
								page_b_ok = _valid_z80_page(page_b) && (hdr_tmp[15:0] == 16'hFFFF);
								ss_load_page <= page_a_ok ? page_a : (page_b_ok ? page_b : page_a);
								ss_block_hdr_idx <= 0;
								ss_load_in_block_hdr <= 0;
							end else begin
								ss_block_hdr_idx <= ss_block_hdr_idx + 1'd1;
							end
							if (ss_byte_count == 3'd7) ss_state <= SS_LOAD_FETCH;
							else ss_byte_count <= ss_byte_count + 1'd1;
						end else begin
							ss_data_latch <= ss_data_load;
							ss_wr_wait <= 1'b0;
							ss_state <= SS_WRITE_RAM;
						end
					end else begin
						ss_state <= SS_LOAD_DONE;
						ss_post_load_timer <= 20'd500000;
					end
				end

				SS_WRITE_RAM: begin
					led_user_mode <= LED_BLINK;
					led_disk_mode <= LED_ON;

					if (!ss_wr_wait) begin
						ss_ram_we <= 1;
						ss_wr_wait <= 1'b1;
						ss_wr_seen_busy <= 1'b0;
						ss_stall_cnt <= 0;
					end else begin
						if (!ram_ready) ss_wr_seen_busy <= 1'b1;
						if (!(ss_wr_seen_busy && ram_ready)) begin
							if (ss_stall_cnt != SS_STALL_MAX) ss_stall_cnt <= ss_stall_cnt + 1'd1;
						end else begin
							ss_stall_cnt <= 0;
						end

						if (ss_stall_cnt == SS_STALL_MAX) begin
							led_disk_mode <= LED_BLINK;
						end else if (ss_wr_seen_busy && ram_ready) begin
							ss_wr_wait <= 1'b0;
							ss_wr_seen_busy <= 1'b0;

							if (({14'd0, ss_addr_counter} + 32'd1) >= ss_total_size) begin
								ss_addr_counter <= ss_addr_counter + 1'd1;
								ss_state <= SS_LOAD_DONE;
							ss_post_load_timer <= 20'd50000000;  // 1 second @ 50MHz
							end else begin
								ss_addr_counter <= ss_addr_counter + 1'd1;
								if (ss_block_addr == 15'd16383) begin
									ss_block_addr <= 0;
									ss_block_idx <= ss_block_idx + 1'd1;
									ss_load_in_block_hdr <= 1;
									ss_block_hdr_idx <= 0;
									if (ss_byte_count == 3'd7) ss_state <= SS_LOAD_FETCH;
									else begin
										ss_byte_count <= ss_byte_count + 1'd1;
										ss_state <= SS_LOAD;
									end
								end else begin
									ss_block_addr <= ss_block_addr + 1'd1;
									if (ss_byte_count == 3'd7) ss_state <= SS_LOAD_FETCH;
									else begin
										ss_byte_count <= ss_byte_count + 1'd1;
										ss_state <= SS_LOAD;
									end
								end
							end
						end
					end
				end

				SS_LOAD_DONE: begin
					ss_stall_cnt <= 0;
					if (ss_post_load_timer) ss_post_load_timer <= ss_post_load_timer - 1'd1;
					else begin
						ss_info_hold_val <= {4'h2, ss_slot + 1'b1};  // 4'h2 = load indicator
						ss_info_hold_cnt <= 3'd4;
						ss_info_req <= 1;
						led_user_mode <= LED_PULSE;
						led_disk_mode <= LED_OFF;
						ss_state <= SS_IDLE;
					end
				end

				SS_WAIT_ACK: begin
					if (ss_ddr_ready) begin
						if (ss_header_mode) begin
							ss_state <= SS_IDLE;
						end else begin
							ss_byte_count <= 0;
							ss_ddr_data_buffer <= 64'h0;
							ss_write_addr <= ss_write_addr + 18'd8;
							ss_state <= ss_return_state;
							if (ss_return_state == SS_SAVE_BLOCK_HEADER) begin
								ss_block_addr <= 0;
								ss_block_hdr_idx <= 0;
							end
						end
					end
				end

				SS_SAVE_HEADER: begin
					ss_header_mode <= 1;
					if (ss_ddr_ready) begin
						ss_state <= SS_IDLE;
						ss_saving <= 0;
						ss_header_mode <= 0;
					end
				end
			
			SS_WAIT_PAUSE: begin
				led_user_mode <= LED_BLINK;
				led_disk_mode <= LED_OFF;
				
				// CPU already paused at M1 boundary (see ZX-Spectrum.sv line 375: ss_active & m1)
				// Just wait brief time (~2us = 100 cycles @ 50MHz) for signal stability
				if (!core_paused) begin
					ss_freeze_timeout <= 0;
				end else if (ss_freeze_timeout != SS_FREEZE_TIMEOUT_MAX) begin
					ss_freeze_timeout <= ss_freeze_timeout + 1'd1;
				end
				
				// Core paused at M1, PC is at instruction boundary - just wait for stability
				if (core_paused & (ss_freeze_timeout >= 16'd100)) begin
					led_user_mode <= LED_ON;
					led_disk_mode <= LED_OFF;
					ss_state <= SS_SAVE;
					ss_header_mode <= 0;
					ss_addr_counter <= 18'd8;
					ss_write_addr <= 18'd8;
					ss_byte_count <= 0;
					ss_ddr_data_buffer <= 64'h0;
					ss_seq_hi <= ss_seq_hi + 1'd1;
					if (ss_seq_hi == 24'hFFFFFF) ss_seq_hi <= 24'd1;
					ss_total_size <= 32'd8 + 32'd96 + ((32'd8 + 32'd16384) * (is_48k ? 32'd3 : 32'd8));
					ss_block_addr <= 0;
					ss_block_hdr_idx <= 0;
				end
			end
			
			endcase

			// Capture Z80 header bytes into ss_reg_accum while loading
			if (ss_state == SS_LOAD && !ss_in_mister_header) begin
				if (ss_payload_addr < 18'd32) begin
					case (ss_payload_addr[4:0])
						5'd0:  ss_reg_accum[7:0]     <= ss_data_load;
						5'd1:  ss_reg_accum[15:8]    <= ss_data_load;
						5'd2:  ss_reg_accum[87:80]   <= ss_data_load;
						5'd3:  ss_reg_accum[95:88]   <= ss_data_load;
						5'd4:  ss_reg_accum[119:112] <= ss_data_load;
						5'd5:  ss_reg_accum[127:120] <= ss_data_load;
						5'd8:  ss_reg_accum[55:48]   <= ss_data_load;
						5'd9:  ss_reg_accum[63:56]   <= ss_data_load;
						5'd10: ss_reg_accum[39:32]   <= ss_data_load;
						5'd11: ss_reg_accum[46:40]   <= ss_data_load[6:0];
						5'd12: begin
							ss_reg_accum[47] <= ss_data_load[0];
							ss_border <= ss_data_load[3:1];
						end
						5'd13: ss_reg_accum[103:96]  <= ss_data_load;
						5'd14: ss_reg_accum[111:104] <= ss_data_load;
						5'd15: ss_reg_accum[151:144] <= ss_data_load;
						5'd16: ss_reg_accum[159:152] <= ss_data_load;
						5'd17: ss_reg_accum[167:160] <= ss_data_load;
						5'd18: ss_reg_accum[175:168] <= ss_data_load;
						5'd19: ss_reg_accum[183:176] <= ss_data_load;
						5'd20: ss_reg_accum[191:184] <= ss_data_load;
						5'd21: ss_reg_accum[23:16]   <= ss_data_load;
						5'd22: ss_reg_accum[31:24]   <= ss_data_load;
						5'd23: ss_reg_accum[199:192] <= ss_data_load;
						5'd24: ss_reg_accum[207:200] <= ss_data_load;
						5'd25: ss_reg_accum[135:128] <= ss_data_load;
						5'd26: ss_reg_accum[143:136] <= ss_data_load;
						5'd27: ss_reg_accum[210]     <= ss_data_load[0];
						5'd28: ss_reg_accum[211]     <= ss_data_load[0];
						5'd29: ss_reg_accum[209:208] <= ss_data_load[1:0];
						default: ;
					endcase
				end

				// Internal stream stores the Z80 extension length word at offsets 32..33.
				if (ss_payload_addr == 18'd32) ss_file_ext_len[7:0] <= ss_data_load;
				if (ss_payload_addr == 18'd33) begin
					reg [15:0] ext_len_tmp;
					ext_len_tmp = {ss_data_load, ss_file_ext_len[7:0]};
					ss_file_ext_len[15:8] <= ss_data_load;
					// Header bytes before RAM blocks = base+pad(32) + ext_len_word(2) + ext_body(ext_len), then aligned to 8.
					ss_hdr_bytes_load <= _align8(18'd34 + {2'd0, ext_len_tmp});
				end

				if (ss_payload_addr == 18'd34) ss_reg_accum[71:64] <= ss_data_load;
				if (ss_payload_addr == 18'd35) ss_reg_accum[79:72] <= ss_data_load;
				if (ss_payload_addr == 18'd36) begin
					ss_file_is_48k <= (ss_data_load == 8'd0);
					ss_is_48k <= (ss_data_load == 8'd0);
					ss_file_is_plus3 <= (ss_data_load == 8'd7);
					ss_is_plus3 <= (ss_data_load == 8'd7);
				end
				if (ss_payload_addr == 18'd37) ss_7ffd <= ss_data_load;
				// 1FFD is at standard offset 86; with our padded base header it is at internal offset 88.
				if ((ss_payload_addr == 18'd88) && (ss_file_ext_len >= 16'd55)) ss_1ffd <= ss_data_load;
				// Restore selected PSG/AY register (payload 40 matches save byte 8 of ext header)
				if (ss_payload_addr == 18'd40) ss_psg_sel <= ss_data_load;
				// Restore PSG/AY registers (payload 41-56 matches save bytes 9-24 of ext header)
				if (ss_payload_addr == 18'd41) ss_psg_regs[0]  <= ss_data_load;
				if (ss_payload_addr == 18'd42) ss_psg_regs[1]  <= ss_data_load;
				if (ss_payload_addr == 18'd43) ss_psg_regs[2]  <= ss_data_load;
				if (ss_payload_addr == 18'd44) ss_psg_regs[3]  <= ss_data_load;
				if (ss_payload_addr == 18'd45) ss_psg_regs[4]  <= ss_data_load;
				if (ss_payload_addr == 18'd46) ss_psg_regs[5]  <= ss_data_load;
				if (ss_payload_addr == 18'd47) ss_psg_regs[6]  <= ss_data_load;
				if (ss_payload_addr == 18'd48) ss_psg_regs[7]  <= ss_data_load;
				if (ss_payload_addr == 18'd49) ss_psg_regs[8]  <= ss_data_load;
				if (ss_payload_addr == 18'd50) ss_psg_regs[9]  <= ss_data_load;
				if (ss_payload_addr == 18'd51) ss_psg_regs[10] <= ss_data_load;
				if (ss_payload_addr == 18'd52) ss_psg_regs[11] <= ss_data_load;
				if (ss_payload_addr == 18'd53) ss_psg_regs[12] <= ss_data_load;
				if (ss_payload_addr == 18'd54) ss_psg_regs[13] <= ss_data_load;
				if (ss_payload_addr == 18'd55) ss_psg_regs[14] <= ss_data_load;
				if (ss_payload_addr == 18'd56) begin
					ss_psg_regs[15] <= ss_data_load;
					ss_psg_restore <= 1'b1;  // Trigger restore
				end
			end

			if ((ss_state == SS_LOAD || ss_state == SS_LOAD_FETCH) && !ss_in_mister_header && ss_payload_addr == ss_hdr_bytes) begin
				ss_reg_pending <= 1'b1;
			end
			if (ss_reg_pending && !ss_cpu_reset_cnt) begin
				ss_reg_hold <= 2'b11;
				ss_reg_pending <= 1'b0;
			end
		end
	end

	assign ss_active = (ss_state != SS_IDLE);

endmodule
