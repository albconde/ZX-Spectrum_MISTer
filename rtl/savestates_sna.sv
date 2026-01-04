module savestates_sna(
	input  wire        clk_sys,
	input  wire        reset,

	input  wire  [1:0] ss_slot,
	input  wire        ss_save,
	input  wire        ss_load,
	input  wire        core_paused,
	input  wire        cpu_m1,
	input  wire        is_48k,

	input  wire [211:0] cpu_reg,
	input  wire  [2:0] border_color,
	input  wire  [7:0] page_reg,
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

	output reg   [7:0] ss_info,
	output reg         ss_info_req,
	output wire        ss_cpu_reset,
	output reg   [7:0] ss_psg_regs [16],
	output reg         ss_psg_restore,
	output reg         ss_is_48k,
	output reg   [1:0] led_user_mode,
	output reg   [1:0] led_disk_mode
);

	// CPU-only reset pulse during savestate LOAD.
	// T80 doesn't expose the internal EXX "Alternate" flip-flop via REG/DIR,
	// so without forcing a canonical internal state, restores can appear to swap
	// BC/DE/HL with BC'/DE'/HL' depending on what the CPU was doing when saved.
	localparam bit SS_CPU_RESET_ON_LOAD = 1'b1;
	reg  [2:0] ss_cpu_reset_cnt;
	assign ss_cpu_reset = (ss_cpu_reset_cnt != 3'd0);

	// Savestate state encoding (must match ZX-Spectrum.sv)
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

	// LED indicator FSM (must match ZX-Spectrum.sv)
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

	// SDRAM handshake tracking (sdram.sv consumes rd/we on rising edges)
	reg        ss_wr_wait;
	reg        ss_wr_seen_busy;
	reg        ss_rd_seen_busy;

	reg        ss_saving;
	reg        ss_header_mode;
	localparam [7:0] SS_FORMAT_ID = 8'd2; // 2 = SNA
	reg  [23:0] ss_seq_hi = 24'd1;
	wire [31:0] ss_seq_num = {ss_seq_hi, SS_FORMAT_ID};
	reg  [7:0] ss_wait_timer;
	reg [19:0] ss_post_load_timer;
	reg [25:0] ss_stall_cnt;
	localparam [25:0] SS_STALL_MAX = 26'd50000000; // ~1s at 50MHz
	reg        ss_save_old, ss_load_old;

	// Optional stability: arm SAVE/LOAD in SS_IDLE and only leave SS_IDLE
	// (asserting ss_active) on a cpu_m1 rising edge (instruction boundary),
	// or if the core is already paused.
	reg        ss_pending_save;
	reg        ss_pending_load;
	reg        cpu_m1_d;
	reg [25:0] ss_m1_wait_cnt;
	wire       cpu_m1_rise = cpu_m1 & ~cpu_m1_d;
	localparam [25:0] SS_M1_WAIT_MAX = 26'd1000000; // ~20ms at 50MHz (rough)

	wire save_edge = ss_save & ~ss_save_old;
	wire load_edge = ss_load & ~ss_load_old;

	// Unused for SNA, but keep ports compatible
	integer psg_i;

	reg  [3:0] ss_block_idx;
	reg [14:0] ss_block_addr;
	reg  [2:0] ss_block_hdr_idx;
	reg        ss_load_in_block_hdr;
	reg  [7:0] ss_load_page;
	reg [63:0] ss_block_hdr;
	reg [31:0] ss_file_seq;
	reg [31:0] ss_file_payload_words;
	reg        ss_file_is_48k;

	// Hold the slot number on ss_info long enough for the UI->HPS pipeline to
	// latch it on the info_req edge. Otherwise, post-load debug values (SP/R)
	// can overwrite the slot number and the MiSTer menu may ignore the banner.
	reg  [2:0] ss_info_hold_cnt;
	reg  [7:0] ss_info_hold_val;

	reg  [3:0] ss_return_state;

	// Used to delay the SDRAM read datapath a little (cached-word path)
	reg  [1:0] ss_rd_wait;

	// Register-restore pulse stretching (a few cycles)
	reg  [1:0] ss_reg_hold;
	reg        ss_reg_pending;

	// SNA format constants
	localparam [31:0] SNA_HDR_BYTES      = 32'd27;
	localparam [31:0] SNA_HDR_PAD_BYTES  = 32'd32;     // header padded to 8-byte boundary
	localparam [31:0] SNA_BANK_BYTES     = 32'd16384;
	localparam [31:0] SNA_TAIL_PAD_BYTES = 32'd8;      // PC(2)+7FFD(1)+TRDOS(1) padded to 8
	localparam [31:0] SNA_48_PAYLOAD     = SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3);
	localparam [31:0] SNA_128_PAYLOAD    = SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3) + SNA_TAIL_PAD_BYTES + (SNA_BANK_BYTES * 32'd5);
	localparam [17:0] SNA_TAIL_ABS_ADDR  = 18'd8 + SNA_HDR_PAD_BYTES[17:0] + (SNA_BANK_BYTES[17:0] * 18'd3);

	// Helpers for 128K extra bank order (ascending, skipping 2,5 and skipping the third bank)
	function automatic [2:0] _sna_extra_bank(input [2:0] third_bank, input [2:0] idx);
		integer b;
		integer count;
		begin
			count = 0;
			_sna_extra_bank = 3'd0;
			for (b = 0; b < 8; b = b + 1) begin
				if ((b != 2) && (b != 5) && (b[2:0] != third_bank)) begin
					if (count == idx) _sna_extra_bank = b[2:0];
					count = count + 1;
				end
			end
		end
	endfunction
	// For SNA128, the third bank in the stream is the currently paged bank at 0xC000,
	// except when it is 2 or 5 (those are already present), in which case bank 0 is stored.
	wire [2:0] ss_sna_paged_bank = ss_saving ? page_reg[2:0] : ss_7ffd[2:0];
	wire [2:0] ss_sna_third_bank = ((ss_sna_paged_bank == 3'd2) || (ss_sna_paged_bank == 3'd5)) ? 3'd0 : ss_sna_paged_bank;

	// LOAD-only: prefetch the 128K tail word early to learn 7FFD before writing RAM banks.
	reg        ss_sna_prefetch_tail;
	reg [17:0] ss_sna_prefetch_ret_addr_counter;
	reg [17:0] ss_sna_prefetch_ret_write_addr;

	// Connect FSM outputs to DDR
	// Base address inside MiSTer DDR window: 0x0E000000 corresponds to physical 0x3E000000 (ddram.sv adds 0x30000000).
	wire [27:0] ss_slot_offset = {6'd0, ss_slot, 18'd0};
	wire [27:0] ss_byte_offset = (ss_header_mode || ss_state == SS_SAVE_HEADER) ? 28'd0 : {10'd0, ss_write_addr};
	wire [27:0] ss_abs_addr = 28'hE000000 + ss_slot_offset + ss_byte_offset;
	assign ss_ddr_addr = ss_abs_addr[27:1];
	assign ss_ddr_req = (ss_state == SS_SAVE_HEADER) ||
						(ss_state == SS_WAIT_ACK) ||
						((ss_state == SS_LOAD_FETCH) && ({14'd0, ss_write_addr} < ss_total_size));
	assign ss_ddr_rnw = !ss_saving;
	assign ss_ddr_be  = 8'hFF;

	// CRITICAL: HPS reads [31:0]=counter [63:32]=size in 32-bit words (WITHOUT 64-bit header)
	// Size in bytes without header, divided by 4 to get 32-bit words (round UP)
	wire [31:0] ss_payload_bytes = ss_total_size - 32'd8;
	wire [31:0] ss_payload_words = (ss_payload_bytes + 32'd3) >> 2;
	assign ss_ddr_din = (ss_state == SS_SAVE_HEADER || ss_header_mode) ? {ss_payload_words[31:0], ss_seq_num[31:0]} : ss_ddr_data_buffer;

	// Savestate address decode
	wire [17:0] ss_payload_addr = ss_addr_counter - 18'd8;
	wire        ss_in_mister_header = (ss_addr_counter < 18'd8);
	wire        ss_map_is_48k = ss_saving ? is_48k : ss_file_is_48k;
	wire [31:0] ss_payload_addr32 = {14'd0, ss_payload_addr};
	// During LOAD, when finishing the MiSTer header (byte 7), decide if we must prefetch the SNA128 tail.
	wire        ss_sna_prefetch_trigger = (!ss_saving) && ss_in_mister_header && (ss_byte_count == 3'd7)
		&& ((({ss_data_load, ss_file_payload_words[23:0]}) << 2) > SNA_48_PAYLOAD);

	// CPU regs used for SNA packing
	wire [15:0] cpu_pc = cpu_reg[79:64];
	wire [15:0] cpu_sp = cpu_reg[63:48];
	wire [15:0] sna_sp_pushed = cpu_sp - 16'd2;
	wire [15:0] sna_sp_on_disk = is_48k ? sna_sp_pushed : cpu_sp;

	// File-derived values used during LOAD
	reg  [15:0] ss_sna_sp_file;
	reg  [15:0] ss_sna_pc_from_stack;
	reg         ss_sna_pc_stack_lo_seen;
	reg         ss_sna_pc_stack_hi_seen;
	wire [15:0] ss_sna_sp_file_plus2 = ss_sna_sp_file + 16'd2;
	wire [15:0] ss_sna_sp_file_minus2 = ss_sna_sp_file - 16'd2;
	wire [31:0] ss_tail_off_128 = ss_payload_addr32 - (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3));

	// Combinational scratch (Icarus doesn't like block-scoped reg decls)
	reg  [31:0] ss_dec_idx;
	reg  [2:0]  ss_dec_bank_idx;
	reg  [31:0] ss_tail_off;

	// Current decoded RAM location for the active byte
	reg         ss_byte_is_ram;
	reg  [2:0]  ss_ram_bank_sel;
	reg  [13:0] ss_ram_off_sel;
	reg  [15:0] ss_ram_abs_addr; // only meaningful for 48K first-48K region
	reg  [24:0] ss_ram_addr_r;
	assign ss_ram_addr = ss_ram_addr_r;

	always @(*) begin
		ss_byte_is_ram = 1'b0;
		ss_ram_bank_sel = 3'd0;
		ss_ram_off_sel = 14'd0;
		ss_ram_abs_addr = 16'h0000;
		ss_ram_addr_r = 25'd0;
		ss_dec_idx = 32'd0;
		ss_dec_bank_idx = 3'd0;

		// Decode based on payload offset
		if (!ss_in_mister_header) begin
			if (ss_payload_addr >= SNA_HDR_PAD_BYTES && ss_payload_addr < (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3))) begin
				// First three banks in file: 5,2,(0 or paged)
				ss_dec_idx = ss_payload_addr - SNA_HDR_PAD_BYTES;
				ss_byte_is_ram = 1'b1;
				if (ss_dec_idx < SNA_BANK_BYTES) begin
					ss_ram_bank_sel = 3'd5;
					ss_ram_off_sel = ss_dec_idx[13:0];
					ss_ram_abs_addr = 16'h4000 + ss_dec_idx[15:0];
				end else if (ss_dec_idx < (SNA_BANK_BYTES * 32'd2)) begin
					ss_ram_bank_sel = 3'd2;
					ss_ram_off_sel = (ss_dec_idx - SNA_BANK_BYTES);
					ss_ram_abs_addr = 16'h8000 + (ss_dec_idx[15:0] - 16'h4000);
				end else begin
					ss_ram_bank_sel = ss_map_is_48k ? 3'd0 : ss_sna_third_bank;
					ss_ram_off_sel = (ss_dec_idx - (SNA_BANK_BYTES * 32'd2));
					ss_ram_abs_addr = 16'hC000 + (ss_dec_idx[15:0] - 16'h8000);
				end
			end else if (!ss_map_is_48k && ss_payload_addr >= (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3) + SNA_TAIL_PAD_BYTES)
								 && ss_payload_addr < (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3) + SNA_TAIL_PAD_BYTES + (SNA_BANK_BYTES * 32'd5))) begin
				// Extra banks
				ss_dec_idx = ss_payload_addr - (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3) + SNA_TAIL_PAD_BYTES);
				ss_dec_bank_idx = ss_dec_idx[16:14];
				ss_byte_is_ram = 1'b1;
				ss_ram_bank_sel = _sna_extra_bank(ss_sna_third_bank, ss_dec_bank_idx);
				ss_ram_off_sel = ss_dec_idx[13:0];
			end
		end

		if (ss_byte_is_ram) begin
			ss_ram_addr_r = {4'b0000, ss_ram_bank_sel, ss_ram_off_sel};
		end
	end

	// SNA byte generator (header/tail are immediate; RAM bytes are provided in SS_SAVE_WAIT)
	always @(*) begin
		ss_data_save = 8'h00;

		if (ss_state == SS_SAVE) begin
			// Header (padded to 32) and 128K tail block (padded to 8)
			if (ss_payload_addr < SNA_HDR_PAD_BYTES) begin
				case (ss_payload_addr[5:0])
					// 0: I
					6'd0: ss_data_save = cpu_reg[39:32];
					// 1..8: HL', DE', BC', AF' (little-endian words)
					6'd1: ss_data_save = cpu_reg[183:176];
					6'd2: ss_data_save = cpu_reg[191:184];
					6'd3: ss_data_save = cpu_reg[167:160];
					6'd4: ss_data_save = cpu_reg[175:168];
					6'd5: ss_data_save = cpu_reg[151:144];
					6'd6: ss_data_save = cpu_reg[159:152];
					6'd7: ss_data_save = cpu_reg[23:16];  // A'
					6'd8: ss_data_save = cpu_reg[31:24];  // F'
					// 9..18: HL, DE, BC, IY, IX (little-endian words)
					6'd9:  ss_data_save = cpu_reg[119:112];
					6'd10: ss_data_save = cpu_reg[127:120];
					6'd11: ss_data_save = cpu_reg[103:96];
					6'd12: ss_data_save = cpu_reg[111:104];
					6'd13: ss_data_save = cpu_reg[87:80];
					6'd14: ss_data_save = cpu_reg[95:88];
					6'd15: ss_data_save = cpu_reg[199:192];
					6'd16: ss_data_save = cpu_reg[207:200];
					6'd17: ss_data_save = cpu_reg[135:128];
					6'd18: ss_data_save = cpu_reg[143:136];
					// 19: Interrupt (bit 2 contains IFF2)
					6'd19: ss_data_save = {5'b00000, cpu_reg[211], 2'b00};
					// 20: R
					6'd20: ss_data_save = cpu_reg[47:40];
					// 21..22: AF
					6'd21: ss_data_save = cpu_reg[7:0];
					6'd22: ss_data_save = cpu_reg[15:8];
					// 23..24: SP (48K stores SP after pushing PC)
					6'd23: ss_data_save = sna_sp_on_disk[7:0];
					6'd24: ss_data_save = sna_sp_on_disk[15:8];
					// 25: IM
					6'd25: ss_data_save = {6'b0, cpu_reg[209:208]};
					// 26: Border
					6'd26: ss_data_save = {5'b0, border_color[2:0]};
					default: ss_data_save = 8'h00; // padding
				endcase
			end else if (!is_48k
						&& (ss_payload_addr >= (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3)))
						&& (ss_payload_addr <  (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3) + SNA_TAIL_PAD_BYTES))) begin
				ss_tail_off = ss_payload_addr - (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3));
				case (ss_tail_off)
					32'd0: ss_data_save = cpu_pc[7:0];
					32'd1: ss_data_save = cpu_pc[15:8];
					32'd2: ss_data_save = page_reg;
					32'd3: ss_data_save = 8'd0; // TR-DOS ROM paged (unsupported)
					default: ss_data_save = 8'h00; // padding
				endcase
			end
		end else if (ss_state == SS_SAVE_WAIT || ss_state == SS_PRE_READ) begin
			// RAM byte; for 48K snapshots, inject PC on the stack (SNA standard)
			ss_data_save = ram_dout;
			if (ss_saving && ss_map_is_48k) begin
				if (ss_ram_abs_addr == sna_sp_pushed) ss_data_save = cpu_pc[7:0];
				else if (ss_ram_abs_addr == (sna_sp_pushed + 16'd1)) ss_data_save = cpu_pc[15:8];
			end
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

	// CPU reset pulse generation during LOAD (match behavior of Z80 engine)
	always @(posedge clk_sys) begin
		if (reset) begin
			ss_cpu_reset_cnt <= 3'd0;
		end else begin
			if (SS_CPU_RESET_ON_LOAD && (ss_state == SS_LOAD || ss_state == SS_LOAD_FETCH) && !ss_in_mister_header && (ss_payload_addr < SNA_HDR_PAD_BYTES)) begin
				if (ss_cpu_reset_cnt != 3'd7) ss_cpu_reset_cnt <= 3'd7;
			end else if (ss_cpu_reset_cnt != 3'd0) begin
				ss_cpu_reset_cnt <= ss_cpu_reset_cnt - 3'd1;
			end
		end
	end

	// Report slot number to the MiSTer OSD banner logic.
	// (Held briefly after LOAD to ensure HPS latches it on info_req edge.)
	always @(*) begin
		if (ss_info_hold_cnt != 3'd0) ss_info = ss_info_hold_val;
		else ss_info = {4'h0, ss_slot + 1'b1};
	end

	// Main savestate FSM
	always @(posedge clk_sys) begin
		cpu_m1_d <= cpu_m1;
		ss_save_old <= ss_save;
		ss_load_old <= ss_load;

		if (reset) begin
			cpu_m1_d <= 1'b0;
			ss_pending_save <= 1'b0;
			ss_pending_load <= 1'b0;
			ss_m1_wait_cnt <= 26'd0;

			ss_reg_set <= 0;
			ss_reg_accum <= 0;
			ss_border <= 0;
			ss_7ffd <= 0;
			ss_reg_hold <= 0;
			ss_reg_pending <= 1'b0;
			ss_file_is_48k <= 1'b1;

			ss_info_req <= 0;
			ss_state <= SS_IDLE;
			ss_wr_wait <= 0;
			ss_wr_seen_busy <= 0;
			ss_rd_seen_busy <= 0;
			ss_rd_wait <= 0;
			ss_ram_rd <= 0;
			ss_ram_we <= 0;
			ss_addr_counter <= 0;
			ss_write_addr <= 0;
			ss_byte_count <= 0;
			ss_saving <= 0;
			ss_header_mode <= 0;
			ss_seq_hi <= 24'd1;
			ss_psg_restore <= 1'b0;
			ss_is_48k <= 1'b1;
			ss_sna_sp_file <= 16'd0;
			ss_sna_pc_from_stack <= 16'd0;
			ss_sna_pc_stack_lo_seen <= 1'b0;
			ss_sna_pc_stack_hi_seen <= 1'b0;
			for(psg_i = 0; psg_i < 16; psg_i = psg_i + 1) begin
				ss_psg_regs[psg_i] <= 8'h00;
			end
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
			led_user_mode <= LED_OFF;
			led_disk_mode <= LED_OFF;
			ss_info_hold_cnt <= 0;
			ss_info_hold_val <= 0;
		end else begin
			// Default: clear one-shot banner request.
			ss_info_req <= 0;

			// Default: keep ss_reg_set asserted while the hold counter is non-zero.
			ss_reg_set <= |ss_reg_hold;
			if (ss_reg_hold) ss_reg_hold <= ss_reg_hold - 1'd1;

			// Decay the post-load info hold.
			if (ss_info_hold_cnt != 3'd0) ss_info_hold_cnt <= ss_info_hold_cnt - 3'd1;

			// Default: no SDRAM strobes.
			ss_ram_rd <= 0;
			ss_ram_we <= 0;

			// Re-init per load request. HW mode will be parsed from the extended header later.
			if (load_edge) begin
				ss_file_is_48k <= 1'b1;
				ss_is_48k <= 1'b1;
				ss_7ffd <= 8'h00;
				ss_reg_hold <= 0;
				ss_reg_pending <= 1'b0;
				ss_reg_set <= 0;
				ss_sna_sp_file <= 16'd0;
				ss_sna_pc_from_stack <= 16'd0;
				ss_sna_pc_stack_lo_seen <= 1'b0;
				ss_sna_pc_stack_hi_seen <= 1'b0;
				ss_sna_prefetch_tail <= 1'b0;
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
					if (save_edge) begin
						ss_pending_save <= 1'b1;
						ss_pending_load <= 1'b0;
						ss_m1_wait_cnt <= 26'd0;
					end else if (load_edge) begin
						ss_pending_load <= 1'b1;
						ss_pending_save <= 1'b0;
						ss_m1_wait_cnt <= 26'd0;
					end

					// Start once we see an instruction-boundary (cpu_m1 rising edge), or if the
					// core is already paused. Timeout is a safety valve to avoid deadlock.
					if (ss_pending_save) begin
						if (core_paused || cpu_m1_rise || (ss_m1_wait_cnt > SS_M1_WAIT_MAX)) begin
							ss_pending_save <= 1'b0;
							led_user_mode <= LED_ON;
							led_disk_mode <= LED_OFF;
							ss_state <= SS_SAVE;
							ss_saving <= 1;
							ss_header_mode <= 0;
							ss_addr_counter <= 18'd8;
							ss_write_addr <= 18'd8;
							ss_byte_count <= 0;
							ss_ddr_data_buffer <= 64'h0;
							ss_seq_hi <= ss_seq_hi + 1'd1;
							if (ss_seq_hi == 24'hFFFFFF) ss_seq_hi <= 24'd1;
							ss_total_size <= 32'd8 + (is_48k ? SNA_48_PAYLOAD : SNA_128_PAYLOAD);
							ss_block_idx <= 0;
							ss_block_addr <= 0;
							ss_block_hdr_idx <= 0;
						end else begin
							ss_m1_wait_cnt <= ss_m1_wait_cnt + 1'd1;
						end
					end else if (ss_pending_load) begin
						if (core_paused || cpu_m1_rise || (ss_m1_wait_cnt > SS_M1_WAIT_MAX)) begin
							ss_pending_load <= 1'b0;
							// LOAD start: blink USER, blink DISK until first DDR word arrives
							led_user_mode <= LED_BLINK;
							led_disk_mode <= LED_BLINK;
							ss_state <= SS_LOAD_FETCH;
							ss_saving <= 0;
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
							ss_sna_prefetch_tail <= 1'b0;
						end else begin
							ss_m1_wait_cnt <= ss_m1_wait_cnt + 1'd1;
						end
					end
				end

				SS_SAVE: begin
					led_user_mode <= LED_ON;
					led_disk_mode <= LED_OFF;
					if (ss_addr_counter < ss_total_size) begin
						// Immediate bytes (SNA header padding and 128K tail padding)
						if (!ss_byte_is_ram) begin
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
						end else begin
							// RAM byte - go through read pipeline
							ss_state <= SS_PRE_READ;
						end
					end else begin
						ss_state <= SS_SAVE_HEADER;
						ss_addr_counter <= 0;
						ss_byte_count <= 0;
						ss_header_mode <= 1;
					end
				end

				SS_PRE_READ: begin
					led_user_mode <= LED_BLINK;
					led_disk_mode <= LED_OFF;
					// NOTE: Top-level drives ram_rd during SS_SAVE_WAIT (legacy behavior).
					// Here we only set up the wait.
					ss_rd_seen_busy <= 0;
					ss_rd_wait <= 2'd1;
					ss_state <= SS_SAVE_WAIT;
					ss_wait_timer <= 0;
				end

				SS_SAVE_WAIT: begin
					led_user_mode <= LED_ON;
					led_disk_mode <= ram_ready ? LED_ON : LED_OFF;
					if (ss_rd_wait != 0) ss_rd_wait <= ss_rd_wait - 1'd1;
					if (!ram_ready) ss_rd_seen_busy <= 1'b1;
					// Complete read when output is expected to be valid:
					// - cached-word path: ram_ready stays high, use ss_rd_wait
					// - SDRAM path: ram_ready may go low then high; ss_rd_wait will have expired long before
					if (ram_ready && (ss_rd_wait == 0)) begin
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
							// Continue with next byte (SS_SAVE decides whether to PRE_READ)
							ss_state <= SS_SAVE;
						end
					end
				end

				SS_LOAD_FETCH: begin
					// DDR read pending: blink DISK while waiting, turn off when a word is captured.
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
						ss_post_load_timer <= 20'd500000;
					end else if (ss_ddr_ready) begin
						ss_ddr_data_buffer <= ss_ddr_dout;
						ss_byte_count <= 3'd0;
						ss_state <= SS_LOAD;
						ss_write_addr <= ss_write_addr + 18'd8;
					end
				end

				SS_LOAD: begin
					// Parsing header/data: USER blink, DISK off
					led_user_mode <= LED_BLINK;
					led_disk_mode <= LED_OFF;

					// If we're in the prefetch path, the DDR buffer contains the tail word at SNA_TAIL_ABS_ADDR.
					// Extract 7FFD from byte 2 and resume normal sequential load.
					if (ss_sna_prefetch_tail) begin
						ss_7ffd <= ss_ddr_data_buffer[23:16];
						ss_sna_prefetch_tail <= 1'b0;
						ss_addr_counter <= ss_sna_prefetch_ret_addr_counter;
						ss_write_addr <= ss_sna_prefetch_ret_write_addr;
						ss_byte_count <= 3'd0;
						ss_state <= SS_LOAD_FETCH;
					end else begin
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
									ss_file_payload_words[31:24] <= ss_data_load;
									ss_total_size <= ((32'd8 + (({ss_data_load, ss_file_payload_words[23:0]}) << 2)) > 32'd262144)
										? 32'd262144
										: (32'd8 + (({ss_data_load, ss_file_payload_words[23:0]}) << 2));
									ss_file_is_48k <= ((({ss_data_load, ss_file_payload_words[23:0]}) << 2) <= SNA_48_PAYLOAD);
									ss_is_48k <= ((({ss_data_load, ss_file_payload_words[23:0]}) << 2) <= SNA_48_PAYLOAD);
									if (((({ss_data_load, ss_file_payload_words[23:0]}) << 2) <= SNA_48_PAYLOAD)) begin
										ss_7ffd <= 8'h00;
									end
									// For 128K, prefetch the tail word early so ss_7ffd is valid before RAM writes.
									if (((({ss_data_load, ss_file_payload_words[23:0]}) << 2) > SNA_48_PAYLOAD)) begin
										ss_sna_prefetch_ret_addr_counter <= ss_addr_counter + 18'd1;
										ss_sna_prefetch_ret_write_addr <= ss_write_addr;
										ss_sna_prefetch_tail <= 1'b1;
										ss_addr_counter <= SNA_TAIL_ABS_ADDR;
										ss_write_addr <= SNA_TAIL_ABS_ADDR;
										ss_byte_count <= 3'd0;
										ss_state <= SS_LOAD_FETCH;
									end
								end
							endcase

							// Normal streaming continues unless we scheduled the 128K tail prefetch above.
							if (!ss_sna_prefetch_trigger) begin
								ss_addr_counter <= ss_addr_counter + 1'd1;
								if (ss_byte_count == 3'd7) ss_state <= SS_LOAD_FETCH;
								else ss_byte_count <= ss_byte_count + 1'd1;
							end
						end else if (ss_payload_addr < SNA_HDR_PAD_BYTES[17:0]) begin
							// SNA header+padding bytes
							ss_addr_counter <= ss_addr_counter + 1'd1;
							if (ss_byte_count == 3'd7) ss_state <= SS_LOAD_FETCH;
							else ss_byte_count <= ss_byte_count + 1'd1;
						end else if (!ss_file_is_48k
												&& (ss_payload_addr >= (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3)))
												&& (ss_payload_addr <  (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3) + SNA_TAIL_PAD_BYTES))) begin
							// 128K tail bytes
							ss_addr_counter <= ss_addr_counter + 1'd1;
							if (ss_byte_count == 3'd7) ss_state <= SS_LOAD_FETCH;
							else ss_byte_count <= ss_byte_count + 1'd1;
						end else begin
							// RAM byte
							ss_data_latch <= ss_data_load;
							// 128K SNA: synthesize the 48K-style RETN trampoline by pushing the real PC
							// onto the stack (SP-2) while loading RAM. This allows RETN to both pop PC
							// and copy IFF2->IFF1, without directly forcing IFF1 during restore.
							if (!ss_file_is_48k) begin
								if (ss_ram_abs_addr == ss_sna_sp_file_minus2) ss_data_latch <= ss_reg_accum[71:64];
								if (ss_ram_abs_addr == (ss_sna_sp_file_minus2 + 16'd1)) ss_data_latch <= ss_reg_accum[79:72];
							end
							ss_wr_wait <= 1'b0;
							ss_state <= SS_WRITE_RAM;
						end
						end else begin
							ss_state <= SS_LOAD_DONE;
							ss_post_load_timer <= 20'd500000;
						end
					end
				end

				SS_WRITE_RAM: begin
					// Writing RAM: USER blink, DISK on
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
								ss_post_load_timer <= 20'd500000;
							end else begin
								ss_addr_counter <= ss_addr_counter + 1'd1;
								if (ss_byte_count == 3'd7) ss_state <= SS_LOAD_FETCH;
								else begin
									ss_byte_count <= ss_byte_count + 1'd1;
									ss_state <= SS_LOAD;
								end
							end
						end
					end
				end

				SS_LOAD_DONE: begin
					ss_stall_cnt <= 0;
					if (ss_post_load_timer) ss_post_load_timer <= ss_post_load_timer - 1'd1;
					else begin
						// Use RETN trampoline for both 48K and 128K:
						// - 48K: PC is already stored on stack in the snapshot data.
						// - 128K: we injected PC bytes into RAM at SP-2/SP-1 during load.
						ss_reg_accum[71:64] <= 8'h72;
						ss_reg_accum[79:72] <= 8'h00;
						if (ss_file_is_48k) begin
							ss_reg_accum[55:48] <= ss_sna_sp_file[7:0];
							ss_reg_accum[63:56] <= ss_sna_sp_file[15:8];
						end else begin
							ss_reg_accum[55:48] <= ss_sna_sp_file_minus2[7:0];
							ss_reg_accum[63:56] <= ss_sna_sp_file_minus2[15:8];
						end
						ss_reg_pending <= 1'b1;
						ss_info_hold_val <= {4'h0, ss_slot + 1'b1};
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
				default: begin
				end
			endcase

			// Capture SNA header bytes into ss_reg_accum while loading.
			if (ss_state == SS_LOAD && !ss_in_mister_header) begin
				// SNA header (first 27 bytes) padded to 32
				if (ss_payload_addr < SNA_HDR_PAD_BYTES[17:0]) begin
					case (ss_payload_addr[5:0])
						6'd0: ss_reg_accum[39:32] <= ss_data_load; // I
						6'd1: ss_reg_accum[183:176] <= ss_data_load; // L'
						6'd2: ss_reg_accum[191:184] <= ss_data_load; // H'
						6'd3: ss_reg_accum[167:160] <= ss_data_load; // E'
						6'd4: ss_reg_accum[175:168] <= ss_data_load; // D'
						6'd5: ss_reg_accum[151:144] <= ss_data_load; // C'
						6'd6: ss_reg_accum[159:152] <= ss_data_load; // B'
						6'd7: ss_reg_accum[23:16]   <= ss_data_load; // A'
						6'd8: ss_reg_accum[31:24]   <= ss_data_load; // F'
						6'd9: ss_reg_accum[119:112] <= ss_data_load; // L
						6'd10: ss_reg_accum[127:120] <= ss_data_load; // H
						6'd11: ss_reg_accum[103:96] <= ss_data_load; // E
						6'd12: ss_reg_accum[111:104] <= ss_data_load; // D
						6'd13: ss_reg_accum[87:80] <= ss_data_load; // C
						6'd14: ss_reg_accum[95:88] <= ss_data_load; // B
						6'd15: ss_reg_accum[199:192] <= ss_data_load; // IYL
						6'd16: ss_reg_accum[207:200] <= ss_data_load; // IYH
						6'd17: ss_reg_accum[135:128] <= ss_data_load; // IXL
						6'd18: ss_reg_accum[143:136] <= ss_data_load; // IXH
						6'd19: begin
							// SNA stores IFF2 in bit2. Keep IFF1 cleared.
							// A RETN trampoline (PC=0x0072) copies IFF2->IFF1 after resume.
							ss_reg_accum[211] <= ss_data_load[2];
							ss_reg_accum[210] <= 1'b0;
						end
						6'd20: ss_reg_accum[47:40] <= ss_data_load; // R
						6'd21: ss_reg_accum[7:0] <= ss_data_load; // A
						6'd22: ss_reg_accum[15:8] <= ss_data_load; // F
						6'd23: begin ss_reg_accum[55:48] <= ss_data_load; ss_sna_sp_file[7:0] <= ss_data_load; end
						6'd24: begin ss_reg_accum[63:56] <= ss_data_load; ss_sna_sp_file[15:8] <= ss_data_load; end
						6'd25: ss_reg_accum[209:208] <= ss_data_load[1:0];
						6'd26: ss_border <= ss_data_load[2:0];
						default: ;
					endcase
				end

				// 128K tail (PC, 7FFD, TRDOS) padded to 8
				if (!ss_file_is_48k
									&& (ss_payload_addr >= (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3)))
									&& (ss_payload_addr <  (SNA_HDR_PAD_BYTES + (SNA_BANK_BYTES * 32'd3) + SNA_TAIL_PAD_BYTES))) begin
					if (ss_tail_off_128 == 32'd0) ss_reg_accum[71:64] <= ss_data_load;
					if (ss_tail_off_128 == 32'd1) ss_reg_accum[79:72] <= ss_data_load;
					if (ss_tail_off_128 == 32'd2) ss_7ffd <= ss_data_load;
				end
			end

			// Capture PC bytes from stack for 48K SNA during RAM write (address derived from current RAM absolute address)
			if (ss_state == SS_WRITE_RAM && ss_file_is_48k) begin
				if (ss_ram_abs_addr == ss_sna_sp_file) begin
					ss_sna_pc_from_stack[7:0] <= ss_data_latch;
					ss_sna_pc_stack_lo_seen <= 1'b1;
				end
				if (ss_ram_abs_addr == (ss_sna_sp_file + 16'd1)) begin
					ss_sna_pc_from_stack[15:8] <= ss_data_latch;
					ss_sna_pc_stack_hi_seen <= 1'b1;
				end
			end

			// Apply DIRSet once reset pulse is finished.
			if (ss_reg_pending && !ss_cpu_reset_cnt) begin
				ss_reg_hold <= 2'b11;
				ss_reg_pending <= 1'b0;
			end
		end
	end

	assign ss_active = (ss_state != SS_IDLE);

endmodule
