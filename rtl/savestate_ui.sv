module savestate_ui
(
	input             clk_vid,
	input             ce_pix,
	input             de,
	input             clk_sys,
	input      [10:0] ps2_key,
	input      [31:0] joystick_a,
	input      [31:0] joystick_b,
	input     [127:0] status,
	input             ss_active,

	output reg        slot_save,
	output reg        slot_load,
	output reg        slot_info,
	output reg        dbg1,
	output reg        dbg2,
	output reg        dbg3,
	output reg        dbg4,
	input       [7:0] slot_num,
	input       [7:0] slot_num_dbg,
	input             slot_info_dbg,
	output reg  [1:0] selected_slot,
	output reg        req_pause
);

parameter INFO_TIMEOUT_BITS = 25;

reg [INFO_TIMEOUT_BITS-1:0] info_timeout;
assign req_pause = ss_active;

reg [10:0] ps2_key_last;
wire pressed = ps2_key[9];
wire [8:0] code = ps2_key[8:0];

localparam int SS_BTN_IDX = 6; // 3rd button in CONF_STR "J," (Fire1=4, Fire2=5, Savestates=6)
localparam int SS_BTN_ALT = 16; // compatibility with cores that map savestate modifier to bit 16

wire joy_ss_mod = joystick_a[SS_BTN_IDX] | joystick_b[SS_BTN_IDX] | joystick_a[SS_BTN_ALT] | joystick_b[SS_BTN_ALT];
wire joy_up     = joystick_a[3] | joystick_b[3];
wire joy_down   = joystick_a[2] | joystick_b[2];
wire joy_left   = joystick_a[1] | joystick_b[1];
wire joy_right  = joystick_a[0] | joystick_b[0];

reg joy_up_d = 0;
reg joy_down_d = 0;
reg joy_left_d = 0;
reg joy_right_d = 0;

reg alt_pressed  = 1'b0;
reg ctrl_pressed = 1'b0;
reg shift_pressed = 1'b0;

reg init_done = 1'b0;

reg home_down = 1'b0;
reg end_down  = 1'b0;
reg pgup_down = 1'b0;
reg pgdn_down = 1'b0;

reg [1:0] last_osd_slot;
reg       last_osd_save;
reg       last_osd_load;
reg       last_osd_dbg1;
reg       last_osd_dbg2;
reg       last_osd_dbg3;
reg       last_osd_dbg4;

always @(posedge clk_sys) begin
	reg old_state;

	if(!init_done) begin
		init_done <= 1'b1;
		alt_pressed <= 1'b0;
		ctrl_pressed <= 1'b0;
		shift_pressed <= 1'b0;
		home_down <= 1'b0;
		end_down <= 1'b0;
		pgup_down <= 1'b0;
		pgdn_down <= 1'b0;
		selected_slot <= 2'd0;
	end
	
	ps2_key_last <= ps2_key;
	
	slot_save <= 0;
	slot_load <= 0;
	slot_info <= 0;
	dbg1 <= 0;
	dbg2 <= 0;
	dbg3 <= 0;
	dbg4 <= 0;

	if(ss_active) info_timeout <= '1;
	else if(info_timeout) info_timeout <= info_timeout - 1'd1;

	// Joystick savestate shortcuts (PSX-style): hold Savestates button,
	// then use D-pad:
	//   Up    = Load state
	//   Down  = Save state
	//   Left  = Previous slot
	//   Right = Next slot
	// These are edge-triggered to avoid repeats while holding a direction.
	joy_up_d    <= joy_up;
	joy_down_d  <= joy_down;
	joy_left_d  <= joy_left;
	joy_right_d <= joy_right;

	if(joy_ss_mod && !ss_active) begin
		if(joy_left && !joy_left_d) begin
			selected_slot <= selected_slot - 2'd1;
		end
		if(joy_right && !joy_right_d) begin
			selected_slot <= selected_slot + 2'd1;
		end
		if(joy_down && !joy_down_d) begin
			slot_save <= 1;
			slot_info <= 1;
			info_timeout <= '1;
		end
		if(joy_up && !joy_up_d) begin
			slot_load <= 1;
			slot_info <= 1;
			info_timeout <= '1;
		end
	end

	// OSD Handling
	last_osd_slot <= status[43:42];
	last_osd_save <= status[44];
	last_osd_load <= status[45];
	last_osd_dbg1 <= status[46];
	last_osd_dbg2 <= status[47];
	last_osd_dbg3 <= status[48];
	last_osd_dbg4 <= status[49];

	if (status[43:42] != last_osd_slot) begin
		selected_slot <= status[43:42];
	end

// If debug override is present, set selected slot num accordingly
if (slot_info_dbg) begin
	// Optionally present selected slot as lower bits of debug byte
	selected_slot <= slot_num_dbg[1:0];
end

	if (status[44] && !last_osd_save) begin
		slot_save <= 1;
		slot_info <= 1;
		info_timeout <= '1;
	end

	if (status[45] && !last_osd_load) begin
		slot_load <= 1;
		slot_info <= 1;
		info_timeout <= '1;
	end

	if (status[46] && !last_osd_dbg1) begin
		dbg1 <= 1;
		slot_info <= 1;
		info_timeout <= '1;
	end

	if (status[47] && !last_osd_dbg2) begin
		dbg2 <= 1;
		slot_info <= 1;
		info_timeout <= '1;
	end

	if (status[48] && !last_osd_dbg3) begin
		dbg3 <= 1;
		slot_info <= 1;
		info_timeout <= '1;
	end

	if (status[49] && !last_osd_dbg4) begin
		dbg4 <= 1;
		slot_info <= 1;
		info_timeout <= '1;
	end

	// External debug-driven info request
	if (slot_info_dbg) begin
		slot_info <= 1;
		info_timeout <= '1;
	end

	if(ps2_key_last != ps2_key) begin
		if(code == 9'h011) alt_pressed <= pressed; // L-Alt
		if(code == 9'h111) alt_pressed <= pressed; // R-Alt
		if(code == 9'h014) ctrl_pressed <= pressed; // L-Ctrl
		if(code == 9'h114) ctrl_pressed <= pressed; // R-Ctrl
		if(code == 9'h012) shift_pressed <= pressed; // L-Shift
		if(code == 9'h059) shift_pressed <= pressed; // R-Shift

		if(code == 9'h16C) home_down <= pressed; // Home
		if(code == 9'h169) end_down  <= pressed; // End
		if(code == 9'h17D) pgup_down <= pressed; // PageUp
		if(code == 9'h17A) pgdn_down <= pressed; // PageDown
		
		if(pressed && !ss_active) begin
			case(code)
				// New savestate hotkeys (ZX Spectrum doesn't use these keys):
				// - Home/End/PageUp/PageDown = LOAD slots 1..4
				// - Alt + (same key)         = SAVE slots 1..4
				9'h16C: begin // Home (E0 6C)
					if(!home_down) begin
						selected_slot <= 0;
						if(alt_pressed) slot_save <= 1;
						else            slot_load <= 1;
						slot_info <= 1;
						info_timeout <= '1;
					end
				end
				9'h169: begin // End (E0 69)
					if(!end_down) begin
						selected_slot <= 1;
						if(alt_pressed) slot_save <= 1;
						else            slot_load <= 1;
						slot_info <= 1;
						info_timeout <= '1;
					end
				end
				9'h17D: begin // PageUp (E0 7D)
					if(!pgup_down) begin
						selected_slot <= 2;
						if(alt_pressed) slot_save <= 1;
						else            slot_load <= 1;
						slot_info <= 1;
						info_timeout <= '1;
					end
				end
				9'h17A: begin // PageDown (E0 7A)
					if(!pgdn_down) begin
						selected_slot <= 3;
						if(alt_pressed) slot_save <= 1;
						else            slot_load <= 1;
						slot_info <= 1;
						info_timeout <= '1;
					end
				end
			endcase
		end
	end
end

endmodule
