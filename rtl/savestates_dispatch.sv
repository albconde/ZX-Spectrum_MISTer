module savestates_dispatch (
	input  wire        clk_sys,
	input  wire        reset,

	input  wire  [1:0] ss_type,

	input  wire  [1:0] ss_slot,
	input  wire        ss_save,
	input  wire        ss_load,
	input  wire        core_paused,
	input  wire        cpu_m1,
	input  wire        is_48k,
	input  wire  [7:0] z80_hw_mode,
	input  wire [23:0] z80_tstates,
	input  wire  [7:0] port_fe,

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
	output wire        ss_ram_rd,
	output wire        ss_ram_we,
	output wire  [3:0] ss_state,

	output wire  [7:0] ss_data_save,
	output wire  [7:0] ss_data_load,
	output wire  [7:0] ss_data_latch,

	output wire [211:0] ss_reg_accum,
	output wire         ss_reg_set,
	output wire  [2:0]  ss_border,
	output wire  [7:0]  ss_fe,
	output wire  [7:0]  ss_7ffd,
	output wire  [7:0]  ss_1ffd,
	output wire         ss_is_plus3,

	output wire  [7:0] ss_info,
	output wire        ss_info_req,
	output wire        ss_cpu_reset,
	output wire  [7:0] ss_psg_sel,
	output wire  [7:0] ss_psg_regs [16],
	output wire        ss_psg_restore,
	output wire        ss_load_z80_pulse,
	output wire        ss_load_szx_pulse,
	output wire        ss_is_48k,
	output wire  [1:0] led_user_mode,
	output wire  [1:0] led_disk_mode
);

	// MiSTer header format-id values (stored in seq[7:0])
	localparam [7:0] SS_FMT_SZX = 8'd0;
	localparam [7:0] SS_FMT_Z80 = 8'd1;

	// Only SZX and Z80 are supported for MiSTer savestates.
	// Any non-zero menu value maps to Z80.
	wire [1:0] sel_menu = (ss_type == 2'b00) ? 2'b00 : 2'b01;

	// Dispatcher-controlled selection:
	// - SAVE: use menu
	// - LOAD: auto-detect from the saved header
	reg  [1:0] sel_lat;
	reg  [1:0] slot_lat;
	reg        prefetch_hdr;
	reg        load_pending;
	reg        ss_save_old, ss_load_old;
	reg        ss_save_pulse;
	reg        ss_load_pulse;
	reg        wait_active;
	reg [25:0] wait_active_cnt;
	wire       any_active;

	wire save_edge = ss_save & ~ss_save_old;
	wire load_edge = ss_load & ~ss_load_old;

	// MiSTer DDR base for savestates (match engines)
	wire [27:0] hdr_slot_offset = {6'd0, slot_lat, 18'd0};
	wire [27:0] hdr_abs_addr    = 28'hE000000 + hdr_slot_offset;
	wire [27:1] ddr_addr_hdr    = hdr_abs_addr[27:1];
	wire [7:0]  hdr_fmt_id      = ss_ddr_dout[7:0];

	function automatic [1:0] map_fmt_to_sel(input [7:0] fmt, input [1:0] fallback);
		begin
			case (fmt)
				SS_FMT_SZX: map_fmt_to_sel = 2'b00;
				SS_FMT_Z80: map_fmt_to_sel = 2'b01;
				default:    map_fmt_to_sel = fallback;
			endcase
		end
	endfunction

	always @(posedge clk_sys) begin
		ss_save_old <= ss_save;
		ss_load_old <= ss_load;
		ss_save_pulse <= 1'b0;
		ss_load_pulse <= 1'b0;

		if (reset) begin
			sel_lat <= 2'b00;
			slot_lat <= 2'b00;
			prefetch_hdr <= 1'b0;
			load_pending <= 1'b0;
			wait_active <= 1'b0;
			wait_active_cnt <= 26'd0;
		end else begin
			// Hold selection stable after a SAVE/LOAD pulse until the chosen engine
			// actually asserts active. This prevents DDR mux switching away during
			// engines that start on a safe boundary (cpu_m1/core_paused).
			if (wait_active) begin
				if (wait_active_cnt != 26'd50000000) wait_active_cnt <= wait_active_cnt + 1'd1;
				// Clear once selected engine is active, or on timeout (~1s @ 50MHz)
				if ((sel_lat == 2'b00 && active_szx) || (sel_lat == 2'b01 && active_z80)
					|| (wait_active_cnt == 26'd50000000)) begin
					wait_active <= 1'b0;
					wait_active_cnt <= 26'd0;
				end
			end

			// Update default selection while idle.
			if (!any_active) begin
				sel_lat <= sel_menu;
				slot_lat <= ss_slot;
			end

			// Start SAVE immediately using menu selection.
			if (!any_active && save_edge) begin
				sel_lat <= sel_menu;
				slot_lat <= ss_slot;
				ss_save_pulse <= 1'b1;
				wait_active <= 1'b1;
				wait_active_cnt <= 26'd0;
			end

			// Start LOAD: prefetch MiSTer header first to discover format-id.
			if (!any_active && load_edge) begin
				slot_lat <= ss_slot;
				prefetch_hdr <= 1'b1;
			end

			// Header prefetch handshake
			if (prefetch_hdr && ss_ddr_ready) begin
				sel_lat <= map_fmt_to_sel(hdr_fmt_id, sel_menu);
				prefetch_hdr <= 1'b0;
				load_pending <= 1'b1;
			end

			// Launch the selected engine's LOAD on the next cycle.
			if (load_pending) begin
				ss_load_pulse <= 1'b1;
				load_pending <= 1'b0;
				wait_active <= 1'b1;
				wait_active_cnt <= 26'd0;
			end
		end
	end

	wire [1:0] sel = sel_lat;

	wire sel_szx = (sel == 2'b00);
	wire sel_z80 = (sel == 2'b01);

	// Gate save/load pulses to one implementation.
	wire ss_save_szx = ss_save_pulse & sel_szx;
	wire ss_load_szx = ss_load_pulse & sel_szx;
	wire ss_save_z80 = ss_save_pulse & sel_z80;
	wire ss_load_z80 = ss_load_pulse & sel_z80;

	// --- SZX ---
	wire [27:1] ddr_addr_szx;
	wire [63:0] ddr_din_szx;
	wire        ddr_req_szx;
	wire        ddr_rnw_szx;
	wire  [7:0] ddr_be_szx;
	wire        active_szx;
	wire [24:0] ram_addr_szx;
	wire        ram_rd_szx;
	wire        ram_we_szx;
	wire  [3:0] state_szx;
	wire  [7:0] data_save_szx;
	wire  [7:0] data_load_szx;
	wire  [7:0] data_latch_szx;
	wire [211:0] reg_accum_szx;
	wire        reg_set_szx;
	wire  [2:0] border_szx;
	wire  [7:0] fe_szx;
	wire  [7:0] ff7_szx;
	wire  [7:0] ff1ffd_szx;
	wire        isplus3_szx;
	wire  [7:0] info_szx;
	wire        info_req_szx;
	wire        cpu_reset_szx;
	wire  [7:0] psg_sel_szx;
	wire  [7:0] psg_regs_szx [16];
	wire        psg_restore_szx;
	wire        is48_szx;
	wire  [1:0] led_user_szx;
	wire  [1:0] led_disk_szx;

	// Derive +3 from the core's Z80 hw mode (7 == +3).
	wire        is_plus3 = (z80_hw_mode == 8'd7);

	savestates_core u_szx (
		.clk_sys(clk_sys),
		.reset(reset),
		.ss_slot(slot_lat),
		.ss_save(ss_save_szx),
		.ss_load(ss_load_szx),
		.core_paused(core_paused),
		.cpu_m1(cpu_m1),
		.is_48k(is_48k),
		.is_plus3(is_plus3),
		.cpu_reg(cpu_reg),
		.border_color(border_color),
		.z80_tstates(z80_tstates),
		.page_reg(page_reg),
		.page_reg_plus3(page_reg_plus3),
		.port_fe(port_fe),
		.psg_reg_addr(psg_reg_addr),
		.psg_reg_shadow(psg_reg_shadow),
		.ram_ready(ram_ready),
		.ram_dout(ram_dout),
		.ss_ddr_dout(ss_ddr_dout),
		.ss_ddr_ready(ss_ddr_ready),
		.ss_ddr_addr(ddr_addr_szx),
		.ss_ddr_din(ddr_din_szx),
		.ss_ddr_req(ddr_req_szx),
		.ss_ddr_rnw(ddr_rnw_szx),
		.ss_ddr_be(ddr_be_szx),
		.ss_active(active_szx),
		.ss_ram_addr(ram_addr_szx),
		.ss_ram_rd(ram_rd_szx),
		.ss_ram_we(ram_we_szx),
		.ss_state(state_szx),
		.ss_data_save(data_save_szx),
		.ss_data_load(data_load_szx),
		.ss_data_latch(data_latch_szx),
		.ss_reg_accum(reg_accum_szx),
		.ss_reg_set(reg_set_szx),
		.ss_border(border_szx),
		.ss_fe(fe_szx),
		.ss_7ffd(ff7_szx),
		.ss_1ffd(ff1ffd_szx),
		.ss_info(info_szx),
		.ss_info_req(info_req_szx),
		.ss_cpu_reset(cpu_reset_szx),
		.ss_psg_sel(psg_sel_szx),
		.ss_psg_regs(psg_regs_szx),
		.ss_psg_restore(psg_restore_szx),
		.ss_is_48k(is48_szx),
		.ss_is_plus3(isplus3_szx),
		.led_user_mode(led_user_szx),
		.led_disk_mode(led_disk_szx)
	);

	// --- Z80 ---
	wire [27:1] ddr_addr_z80;
	wire [63:0] ddr_din_z80;
	wire        ddr_req_z80;
	wire        ddr_rnw_z80;
	wire  [7:0] ddr_be_z80;
	wire        active_z80;
	wire [24:0] ram_addr_z80;
	wire        ram_rd_z80;
	wire        ram_we_z80;
	wire  [3:0] state_z80;
	wire  [7:0] data_save_z80;
	wire  [7:0] data_load_z80;
	wire  [7:0] data_latch_z80;
	wire [211:0] reg_accum_z80;
	wire        reg_set_z80;
	wire  [2:0] border_z80;
	wire  [7:0] ff7_z80;
	wire  [7:0] ff1ffd_z80;
	wire  [7:0] info_z80;
	wire        info_req_z80;
	wire        cpu_reset_z80;
	wire  [7:0] psg_sel_z80;
	wire  [7:0] psg_regs_z80 [16];
	wire        psg_restore_z80;
	wire        is48_z80;
	wire        isplus3_z80;
	wire  [1:0] led_user_z80;
	wire  [1:0] led_disk_z80;

	savestates_z80 u_z80 (
		.clk_sys(clk_sys),
		.reset(reset),
		.ss_slot(slot_lat),
		.ss_save(ss_save_z80),
		.ss_load(ss_load_z80),
		.core_paused(core_paused),
		.cpu_m1(cpu_m1),
		.is_48k(is_48k),
		.z80_hw_mode(z80_hw_mode),
		.z80_tstates(z80_tstates),
		.cpu_reg(cpu_reg),
		.border_color(border_color),
		.page_reg(page_reg),
		.page_reg_plus3(page_reg_plus3),
		.psg_reg_addr(psg_reg_addr),
		.psg_reg_shadow(psg_reg_shadow),
		.ram_ready(ram_ready),
		.ram_dout(ram_dout),
		.ss_ddr_dout(ss_ddr_dout),
		.ss_ddr_ready(ss_ddr_ready),
		.ss_ddr_addr(ddr_addr_z80),
		.ss_ddr_din(ddr_din_z80),
		.ss_ddr_req(ddr_req_z80),
		.ss_ddr_rnw(ddr_rnw_z80),
		.ss_ddr_be(ddr_be_z80),
		.ss_active(active_z80),
		.ss_ram_addr(ram_addr_z80),
		.ss_ram_rd(ram_rd_z80),
		.ss_ram_we(ram_we_z80),
		.ss_state(state_z80),
		.ss_data_save(data_save_z80),
		.ss_data_load(data_load_z80),
		.ss_data_latch(data_latch_z80),
		.ss_reg_accum(reg_accum_z80),
		.ss_reg_set(reg_set_z80),
		.ss_border(border_z80),
		.ss_7ffd(ff7_z80),
		.ss_1ffd(ff1ffd_z80),
		.ss_info(info_z80),
		.ss_info_req(info_req_z80),
		.ss_cpu_reset(cpu_reset_z80),
		.ss_psg_sel(psg_sel_z80),
		.ss_psg_regs(psg_regs_z80),
		.ss_psg_restore(psg_restore_z80),
		.ss_is_48k(is48_z80),
		.ss_is_plus3(isplus3_z80),
		.led_user_mode(led_user_z80),
		.led_disk_mode(led_disk_z80)
	);

	assign any_active = prefetch_hdr | load_pending | wait_active | active_szx | active_z80;

	// Output mux (during header prefetch we override DDR bus)
	assign ss_ddr_addr = prefetch_hdr ? ddr_addr_hdr : (sel_szx ? ddr_addr_szx : ddr_addr_z80);
	assign ss_ddr_din  = prefetch_hdr ? 64'h0       : (sel_szx ? ddr_din_szx  : ddr_din_z80);
	assign ss_ddr_req  = prefetch_hdr ? 1'b1        : (sel_szx ? ddr_req_szx  : ddr_req_z80);
	assign ss_ddr_rnw  = prefetch_hdr ? 1'b1        : (sel_szx ? ddr_rnw_szx  : ddr_rnw_z80);
	assign ss_ddr_be   = prefetch_hdr ? 8'hFF       : (sel_szx ? ddr_be_szx   : ddr_be_z80);

	assign ss_active   = sel_szx ? active_szx   : active_z80;
	assign ss_ram_addr = sel_szx ? ram_addr_szx : ram_addr_z80;
	assign ss_ram_rd   = sel_szx ? ram_rd_szx   : ram_rd_z80;
	assign ss_ram_we   = sel_szx ? ram_we_szx   : ram_we_z80;
	assign ss_state    = sel_szx ? state_szx    : state_z80;

	assign ss_data_save  = sel_szx ? data_save_szx  : data_save_z80;
	assign ss_data_load  = sel_szx ? data_load_szx  : data_load_z80;
	assign ss_data_latch = sel_szx ? data_latch_szx : data_latch_z80;

	assign ss_reg_accum = sel_szx ? reg_accum_szx : reg_accum_z80;
	assign ss_reg_set   = sel_szx ? reg_set_szx   : reg_set_z80;
	assign ss_border    = sel_szx ? border_szx    : border_z80;
	assign ss_fe        = sel_szx ? fe_szx        : {3'b000, 1'b0, 1'b0, border_z80};
	assign ss_7ffd      = sel_szx ? ff7_szx       : ff7_z80;
	assign ss_1ffd      = sel_szx ? ff1ffd_szx    : ff1ffd_z80;

	assign ss_info      = sel_szx ? info_szx      : info_z80;
	assign ss_info_req  = sel_szx ? info_req_szx  : info_req_z80;
	assign ss_cpu_reset = sel_szx ? cpu_reset_szx : cpu_reset_z80;
	assign ss_psg_sel   = sel_szx ? psg_sel_szx   : psg_sel_z80;

	genvar gi;
	generate
		for(gi=0; gi<16; gi=gi+1) begin : GEN_PSG_MUX
			assign ss_psg_regs[gi] = sel_szx ? psg_regs_szx[gi] : psg_regs_z80[gi];
		end
	endgenerate
	assign ss_psg_restore = sel_szx ? psg_restore_szx : psg_restore_z80;
	assign ss_load_z80_pulse = ss_load_z80;
	assign ss_load_szx_pulse = ss_load_szx;
	assign ss_is_48k      = sel_szx ? is48_szx        : is48_z80;
	assign ss_is_plus3    = sel_szx ? isplus3_szx     : isplus3_z80;
	assign led_user_mode  = sel_szx ? led_user_szx    : led_user_z80;
	assign led_disk_mode  = sel_szx ? led_disk_szx    : led_disk_z80;

endmodule
