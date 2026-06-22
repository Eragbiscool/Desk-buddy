/*
 * The Luck Engine — tt_um_luck_engine
 * ========================================
 * Six modes, selected live via ui_in[2:0]:
 *
 *   000  COIN     Press go -> hardware coin flip (0/1)
 *   001  BUSY     Tap to start a 30-60 min "busy" timer with a status LED
 *   010  SPRINT   Press go -> random 15-30 min focus sprint, random break type at end
 *   011  MEETING  Tap N times (1-6), press go -> N*5 min countdown with warning
 *   100  TASK     Tap N times (1-8), press go -> random task number 1..N
 *   101  DUEL     Ready-Countdown-Go reaction duel, fastest press wins
 *
 * Display: an external TM1637 4-digit 7-segment module, driven over a 2-wire
 * serial bus (uio[0]=TM_CLK, uio[1]=TM_DIO). Moving to an external module 
 * freesuo_out entirely for status flags, and gives every mode a full 4-digit
 * readout (e.g. "REDY"/"-C--"/"-G--" style state cues, or up to 4-digit
 * numbers) with no digit-alternation blinking required anywhere.
 *
 * Pin map
 * -------
 * ui_in[2:0]  mode_sel    : live mode selector (change resets current mode)
 * ui_in[3]    btn_go      : main trigger / confirm (debounced)
 * ui_in[4]    btn_tap     : count tap / player-1 button (debounced; also the
 *                           single button used for BUSY mode's add-time and
 *                           double-click controls)
 * ui_in[5]    btn_p2      : player-2 button in DUEL (debounced)
 * ui_in[6]    btn_reset   : cancel / return to idle (debounced)
 * ui_in[7]    (reserved, tie low)
 *
 * uo_out[0]   mode_active : 1 whenever not in IDLE
 * uo_out[1]   p1_ready    : player-1 armed in DUEL_R
 * uo_out[2]   p2_ready    : player-2 armed in DUEL_R
 * uo_out[3]   warn        : meeting 80%-elapsed warning
 * uo_out[4]   timer_run   : 1 during SPRINT_RUN, MEET_RUN, or BUSY_ACTIVE/QMARK
 * uo_out[5]   alarm       : 1 during SPRINT_DONE, MEET_DONE, or DUEL_RESULT
 * uo_out[6]   busy_light  : 1 whenever BUSY mode's timer is live (the actual
 *                           "I'm busy" desk indicator -- wire an LED here)
 * uo_out[7]   penalty_win : 1 alongside alarm in DUEL_RESULT specifically
 *                           when the win was a false-start penalty, not a
 *                           clean reaction-time race (see DUEL section)
 *
 * uio[0]      tm_clk      : TM1637 CLK, always driven (uio_oe[0]=1 always)
 * uio[1]      tm_dio      : TM1637 DIO, bidirectional (driven while writing,
 *                           released while sampling the slave's ACK bit)
 * uio[7:2]    (unused, driven 0, oe=0)
 *
 * Digit code convention used throughout (4 bits, fed to the TM1637 driver):
 *   0-9 = that digit. 10=blank. 11=dash("-"). 12='C'. 13='G'. 14='r'
 *   (lowercase, used as a 7-segment approximation of "R"). 15='?'
 *   (best-effort 7-segment approximation; a 7-segment digit cannot draw a
 *   true question mark, this is a recognizable stand-in, not a faithful glyph).
 */

`default_nettype none

// ──────────────────────────────────────────────────────────────────────────
// Generic button debounce + rising-edge-pulse generator
// ──────────────────────────────────────────────────────────────────────────
module debounce_edge #(
    parameter DEBOUNCE_CYCLES = 4
) (
    input  wire clk,
    input  wire rst_n,
    input  wire in_raw,
    output reg  pulse_out
);
    localparam CNT_W = (DEBOUNCE_CYCLES <= 1) ? 1 : $clog2(DEBOUNCE_CYCLES);

    reg sync_ff1, sync_ff2, stable_state;
    reg [CNT_W-1:0] cnt;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin sync_ff1 <= 1'b0; sync_ff2 <= 1'b0; end
        else        begin sync_ff1 <= in_raw; sync_ff2 <= sync_ff1; end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stable_state <= 1'b0;
            cnt          <= {CNT_W{1'b0}};
            pulse_out    <= 1'b0;
        end else begin
            pulse_out <= 1'b0;
            if (sync_ff2 == stable_state)
                cnt <= {CNT_W{1'b0}};
            else if (cnt == DEBOUNCE_CYCLES[CNT_W-1:0] - 1'b1) begin
                if (!stable_state && sync_ff2) pulse_out <= 1'b1;
                stable_state <= sync_ff2;
                cnt          <= {CNT_W{1'b0}};
            end else
                cnt <= cnt + 1'b1;
        end
    end
endmodule


// ──────────────────────────────────────────────────────────────────────────
// Single/double-click classifier
// ──────────────────────────────────────────────────────────────────────────
// Takes an already-debounced rising-edge pulse stream and classifies each
// press as part of a single-click or a double-click, mouse-style.
//
// On the first pulse, a WINDOW_CYCLES timer starts. If a second pulse
// arrives before the timer expires, double_click pulses immediately. If the
// timer expires with no second pulse, single_click pulses at that moment.
// This means a lone click's action is reported WINDOW_CYCLES after the
// press, not instantly -- an inherent, unavoidable property of distinguishing
// "click" from "click-click" without a faster button.
module click_classifier #(
    parameter WINDOW_CYCLES = 12
) (
    input  wire clk,
    input  wire rst_n,
    input  wire clear,      // abort any in-progress classification, no output pulse
    input  wire pulse_in,
    output reg  single_click,
    output reg  double_click
);
    localparam CNT_W = $clog2(WINDOW_CYCLES + 1);
    reg waiting;
    reg [CNT_W-1:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            waiting      <= 1'b0;
            cnt          <= {CNT_W{1'b0}};
            single_click <= 1'b0;
            double_click <= 1'b0;
        end else if (clear) begin
            waiting      <= 1'b0;
            cnt          <= {CNT_W{1'b0}};
            single_click <= 1'b0;
            double_click <= 1'b0;
        end else begin
            single_click <= 1'b0;
            double_click <= 1'b0;
            if (!waiting) begin
                if (pulse_in) begin
                    waiting <= 1'b1;
                    cnt     <= {CNT_W{1'b0}};
                end
            end else begin
                if (pulse_in) begin
                    double_click <= 1'b1;
                    waiting      <= 1'b0;
                end else if (cnt == WINDOW_CYCLES[CNT_W-1:0] - 1'b1) begin
                    single_click <= 1'b1;
                    waiting      <= 1'b0;
                end else
                    cnt <= cnt + 1'b1;
            end
        end
    end
endmodule


// ──────────────────────────────────────────────────────────────────────────
// TM1637 4-digit display driver — free-running refresh
// ──────────────────────────────────────────────────────────────────────────
// Continuously re-sends the current d0..d3 digit codes to the display in a
// loop: [0x40 write-cmd] [0xC0 addr0, d0,d1,d2,d3] [0x8F display-on-cmd],
// each bracketed group its own START..STOP transaction, the middle group
// using TM1637's auto-increment addressing so all 4 digits go out in one
// transaction. Free-running means the caller never needs to pulse a
// "refresh" signal or wait for "busy" -- just hold d0..d3 at the current
// value and the display catches up within one refresh period
// (~7 bytes * 9 bits * 2*HALF_BIT_CYCLES, plus the gap). A value changing
// mid-transmission can show a one-refresh-period stale digit; at real
// silicon timing (refresh period well under 100ms) this is not visible to
// a human.
module tm1637_driver #(
    parameter HALF_BIT_CYCLES   = 4,  // sim-fast default; real silicon must
                                       // keep TM_CLK well under the 250kHz
                                       // datasheet ceiling -- size this from
                                       // your real clock_hz before tapeout
    parameter REFRESH_GAP_CYCLES = 8
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [3:0] d0, d1, d2, d3,  // leftmost .. rightmost digit codes
    output reg        tm_clk_out,
    output reg        tm_dio_out,
    output reg        tm_dio_oe,       // 1 = chip drives DIO, 0 = released
    input  wire       tm_dio_in
);
    localparam CMD_WRITE = 8'h40;
    localparam CMD_ADDR0 = 8'hC0;
    localparam CMD_CTRL  = 8'h8F;      // display on, max brightness

    function [6:0] seg_of;
        input [3:0] code;
        begin
            case (code)
                4'd0:  seg_of = 7'h3F;
                4'd1:  seg_of = 7'h06;
                4'd2:  seg_of = 7'h5B;
                4'd3:  seg_of = 7'h4F;
                4'd4:  seg_of = 7'h66;
                4'd5:  seg_of = 7'h6D;
                4'd6:  seg_of = 7'h7D;
                4'd7:  seg_of = 7'h07;
                4'd8:  seg_of = 7'h7F;
                4'd9:  seg_of = 7'h6F;
                4'd10: seg_of = 7'h00; // blank
                4'd11: seg_of = 7'h40; // dash
                4'd12: seg_of = 7'h39; // 'C'
                4'd13: seg_of = 7'h3D; // 'G'
                4'd14: seg_of = 7'h50; // 'r' (lowercase approximation of R)
                default: seg_of = 7'h53; // '?' best-effort glyph
            endcase
        end
    endfunction

    function [7:0] byte_of;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: byte_of = CMD_WRITE;
                3'd1: byte_of = CMD_ADDR0;
                3'd2: byte_of = {1'b0, seg_of(d0)};
                3'd3: byte_of = {1'b0, seg_of(d1)};
                3'd4: byte_of = {1'b0, seg_of(d2)};
                3'd5: byte_of = {1'b0, seg_of(d3)};
                default: byte_of = CMD_CTRL; // idx==6
            endcase
        end
    endfunction

    function stop_after;
        input [2:0] idx;
        begin
            stop_after = (idx == 3'd0) || (idx == 3'd5) || (idx == 3'd6);
        end
    endfunction

    localparam [3:0]
        S_GAP     = 4'd0,
        S_START_A = 4'd1,
        S_START_B = 4'd2,
        S_BIT_LO  = 4'd3,
        S_BIT_HI  = 4'd4,
        S_ACK_LO  = 4'd5,
        S_ACK_HI  = 4'd6,
        S_STOP_A  = 4'd7,
        S_STOP_B  = 4'd8,
        S_STOP_C  = 4'd9;

    reg [3:0] state;
    reg [3:0] phase_cnt;     // counts up to HALF_BIT_CYCLES or REFRESH_GAP_CYCLES
    reg [2:0] byte_idx;
    reg [2:0] bit_idx;
    reg [7:0] cur_byte;

    wire phase_done_half = (phase_cnt == HALF_BIT_CYCLES[3:0] - 4'd1);
    wire phase_done_gap  = (phase_cnt == REFRESH_GAP_CYCLES[3:0] - 4'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_GAP;
            phase_cnt  <= 4'd0;
            byte_idx   <= 3'd0;
            bit_idx    <= 3'd0;
            cur_byte   <= 8'd0;
            tm_clk_out <= 1'b1;
            tm_dio_out <= 1'b1;
            tm_dio_oe  <= 1'b1;
        end else begin
            case (state)

                S_GAP: begin
                    tm_clk_out <= 1'b1;
                    tm_dio_out <= 1'b1;
                    tm_dio_oe  <= 1'b1;
                    if (phase_done_gap) begin
                        phase_cnt <= 4'd0;
                        byte_idx  <= 3'd0;
                        state     <= S_START_A;
                    end else
                        phase_cnt <= phase_cnt + 4'd1;
                end

                S_START_A: begin
                    // bus idle: clk=1, dio=1 already held
                    if (phase_done_half) begin
                        phase_cnt <= 4'd0;
                        state     <= S_START_B;
                    end else
                        phase_cnt <= phase_cnt + 4'd1;
                end

                S_START_B: begin
                    tm_dio_out <= 1'b0;  // start condition: dio falls while clk high
                    if (phase_done_half) begin
                        phase_cnt <= 4'd0;
                        cur_byte  <= byte_of(byte_idx);
                        bit_idx   <= 3'd0;
                        state     <= S_BIT_LO;
                    end else
                        phase_cnt <= phase_cnt + 4'd1;
                end

                S_BIT_LO: begin
                    tm_clk_out <= 1'b0;
                    tm_dio_oe  <= 1'b1;
                    tm_dio_out <= cur_byte[bit_idx];
                    if (phase_done_half) begin
                        phase_cnt <= 4'd0;
                        state     <= S_BIT_HI;
                    end else
                        phase_cnt <= phase_cnt + 4'd1;
                end

                S_BIT_HI: begin
                    tm_clk_out <= 1'b1;
                    if (phase_done_half) begin
                        phase_cnt <= 4'd0;
                        if (bit_idx == 3'd7)
                            state <= S_ACK_LO;
                        else begin
                            bit_idx <= bit_idx + 3'd1;
                            state   <= S_BIT_LO;
                        end
                    end else
                        phase_cnt <= phase_cnt + 4'd1;
                end

                S_ACK_LO: begin
                    tm_clk_out <= 1'b0;
                    tm_dio_oe  <= 1'b0;  // release DIO so the slave can pull it low
                    if (phase_done_half) begin
                        phase_cnt <= 4'd0;
                        state     <= S_ACK_HI;
                    end else
                        phase_cnt <= phase_cnt + 4'd1;
                end

                S_ACK_HI: begin
                    tm_clk_out <= 1'b1;
                    // tm_dio_in sampled here is the ACK bit; not currently
                    // surfaced as a status output (see module header).
                    if (phase_done_half) begin
                        phase_cnt <= 4'd0;
                        if (stop_after(byte_idx))
                            state <= S_STOP_A;
                        else begin
                            byte_idx <= byte_idx + 3'd1;
                            cur_byte <= byte_of(byte_idx + 3'd1);
                            bit_idx  <= 3'd0;
                            state    <= S_BIT_LO;
                        end
                    end else
                        phase_cnt <= phase_cnt + 4'd1;
                end

                S_STOP_A: begin
                    tm_clk_out <= 1'b0;
                    tm_dio_oe  <= 1'b1;
                    tm_dio_out <= 1'b0;
                    if (phase_done_half) begin
                        phase_cnt <= 4'd0;
                        state     <= S_STOP_B;
                    end else
                        phase_cnt <= phase_cnt + 4'd1;
                end

                S_STOP_B: begin
                    tm_clk_out <= 1'b1;
                    if (phase_done_half) begin
                        phase_cnt <= 4'd0;
                        state     <= S_STOP_C;
                    end else
                        phase_cnt <= phase_cnt + 4'd1;
                end

                S_STOP_C: begin
                    tm_dio_out <= 1'b1;
                    if (phase_done_half) begin
                        phase_cnt <= 4'd0;
                        if (byte_idx == 3'd6)
                            state <= S_GAP;           // full refresh complete
                        else begin
                            byte_idx <= byte_idx + 3'd1;
                            state    <= S_START_A;     // next bracketed group
                        end
                    end else
                        phase_cnt <= phase_cnt + 4'd1;
                end

                default: state <= S_GAP;
            endcase
        end
    end
endmodule


// ──────────────────────────────────────────────────────────────────────────
// Top level
// ──────────────────────────────────────────────────────────────────────────
module tt_um_luck_engine #(
    parameter PRESCALE          = 100_000_000,  // cycles per simulated "second"
    parameter ROLL_CYCLES       = 48,   // coin-flip animation length (internal only now)
    parameter TM_HALF_BIT_CYCLES = 4,
    parameter CLICK_WINDOW_CYCLES = 12
) (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // ─── Input aliases ──────────────────────────────────────────────────────
    wire [2:0] mode_sel = ui_in[2:0];
    wire raw_go    = ui_in[3];
    wire raw_tap   = ui_in[4];
    wire raw_p2    = ui_in[5];
    wire raw_reset = ui_in[6];

    localparam [2:0]
        MODE_COIN    = 3'd0,
        MODE_BUSY    = 3'd1,
        MODE_SPRINT  = 3'd2,
        MODE_MEETING = 3'd3,
        MODE_TASK    = 3'd4,
        MODE_DUEL    = 3'd5;

    // ─── Debounced rising-edge pulses ───────────────────────────────────────
    wire pulse_go, pulse_tap, pulse_p2, pulse_reset;

    debounce_edge #(.DEBOUNCE_CYCLES(4)) db_go
        (.clk(clk), .rst_n(rst_n), .in_raw(raw_go),    .pulse_out(pulse_go));
    debounce_edge #(.DEBOUNCE_CYCLES(4)) db_tap
        (.clk(clk), .rst_n(rst_n), .in_raw(raw_tap),   .pulse_out(pulse_tap));
    debounce_edge #(.DEBOUNCE_CYCLES(4)) db_p2
        (.clk(clk), .rst_n(rst_n), .in_raw(raw_p2),    .pulse_out(pulse_p2));
    debounce_edge #(.DEBOUNCE_CYCLES(4)) db_reset
        (.clk(clk), .rst_n(rst_n), .in_raw(raw_reset), .pulse_out(pulse_reset));

   

    wire tap_single_click, tap_double_click;
   

    // ─── 16-bit Galois LFSR — polynomial x^16+x^15+x^13+x^4+1 ────────────────
    reg [15:0] lfsr;
    wire lfsr_fb = lfsr[15] ^ lfsr[14] ^ lfsr[12] ^ lfsr[3];
    always @(posedge clk or negedge rst_n)
        if (!rst_n) lfsr <= 16'hACE1;
        else        lfsr <= {lfsr[14:0], lfsr_fb};

    // ─── Prescaler -> 1-second tick ─────────────────────────────────────────
    reg [23:0] presc_cnt;
    wire tick_sec = (presc_cnt == PRESCALE[23:0] - 24'd1);
    always @(posedge clk or negedge rst_n)
        if (!rst_n)        presc_cnt <= 24'd0;
        else if (tick_sec) presc_cnt <= 24'd0;
        else               presc_cnt <= presc_cnt + 24'd1;

    // Fast blink (~4 Hz at default PRESCALE) for DONE/result/blink-off cues
    reg [23:0] fast_cnt;
    reg        blink_fast;
    wire [23:0] fast_half = PRESCALE >> 3;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            fast_cnt   <= 24'd0;
            blink_fast <= 1'b0;
        end else if (fast_cnt >= fast_half - 24'd1) begin
            fast_cnt   <= 24'd0;
            blink_fast <= ~blink_fast;
        end else
            fast_cnt <= fast_cnt + 24'd1;

    // ─── Mode change detection ──────────────────────────────────────────────
    reg [2:0] prev_mode;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) prev_mode <= 3'd0;
        else        prev_mode <= mode_sel;
    wire mode_changed = (mode_sel != prev_mode);

    // ─── FSM state encoding ─────────────────────────────────────────────────
    localparam [4:0]
        ST_IDLE        = 5'd0,
        ST_COIN_ROLL   = 5'd1,
        ST_COIN_RESULT = 5'd2,
        ST_BUSY_ACTIVE = 5'd3,
        ST_BUSY_QMARK  = 5'd4,
        ST_SPRINT_SHOW = 5'd5,
        ST_SPRINT_RUN  = 5'd6,
        ST_SPRINT_DONE = 5'd7,
        ST_MEET_SET    = 5'd8,
        ST_MEET_RUN    = 5'd9,
        ST_MEET_DONE   = 5'd10,
        ST_TASK_SET    = 5'd11,
        ST_TASK_RESULT = 5'd12,
        ST_DUEL_R      = 5'd13,
        ST_DUEL_C      = 5'd14,
        ST_DUEL_G      = 5'd15,
        ST_DUEL_RESULT = 5'd16;

    reg [4:0] state;
  
  	// The press that ENTERS BUSY mode must never also be classified as a
    // later single-click (which would wrongly add 10 minutes the moment
    // the classifier's window expires, a real bug caught in simulation:
    // busy_remaining jumped 30->40 on its own ~20 cycles after entry).
    // Clearing the classifier on the exact entry cycle makes it ignore
    // that specific pulse entirely and start fresh from the next press.
    wire click_clear = (state == ST_IDLE) && (mode_sel == MODE_BUSY) && pulse_tap;
  
  	 click_classifier #(.WINDOW_CYCLES(CLICK_WINDOW_CYCLES)) click_tap (
        .clk(clk), .rst_n(rst_n), .clear(click_clear), .pulse_in(pulse_tap),
        .single_click(tap_single_click), .double_click(tap_double_click)
    );

    // shared / per-mode registers
    reg [3:0] result_r;        // coin(0/1), task(1-8), duel winner(1/2)
    reg [3:0] tap_count;       // meeting(1-6) / task(1-8) tap tally
    reg [7:0] roll_cnt;        // coin animation counter
    reg [4:0] timer_min;       // generic minutes-remaining countdown
    reg [5:0] timer_sec;       // generic seconds-within-minute counter
    reg [4:0] sprint_min_r;    // latched sprint duration (15-30)
    reg [1:0] break_r;         // latched sprint break type (0-3 -> shown 1-4)
    reg [3:0] show_show_sec;   // SPRINT_SHOW hold counter (seconds)

    // BUSY mode
    reg [6:0] busy_remaining;  // minutes remaining, 0-60
    reg [6:0] busy_elapsed;    // minutes elapsed since the ORIGINAL start, 0-60
    reg       busy_no_effect;  // one-shot: add-10 attempted at the 60 min cap

    // DUEL
    reg p1_armed, p2_armed;
    reg [3:0] duel_delay_sec;  // random 2-10 second pre-go delay
    reg duel_penalty;          // latched: result came from a false-start penalty

    // random helpers
    wire [4:0] sprint_rnd = {1'b0, lfsr[3:0]} + 5'd15;          // 15-30, uniform by construction
    wire [1:0] break_rnd = lfsr[9:8];                            // 0-3, exactly uniform (2 raw bits)
    // NOTE: 3 raw bits give 0-7, +2 = 2-9, not quite the requested 2-10. Widen
    // by one bit so the full 2-10 range (9 values) is reachable; rejection on
    // value 10 falls back into [2,9] (very slight, documented, non-uniform
    // tail) rather than truncating the range outright.
    wire [3:0] duel_delay_w   = lfsr[3:0];                        // 0-15
    wire [3:0] duel_delay_final = (duel_delay_w < 4'd9) ? (duel_delay_w + 4'd2) // 2-10
                                                          : (duel_delay_w - 4'd9 + 4'd2); // fallback, still 2-10 range

    wire [4:0] meet_total_min = {tap_count[2:0], 2'b00} + {2'b00, tap_count[2:0]}; // tap_count*5
    wire meet_warn = (state == ST_MEET_RUN) && (timer_min <= {2'b00, tap_count[2:0]});

    // DUEL_G note: the round ends the instant EITHER player's debounced
    // pulse arrives (first press wins outright, the round is over -- the
    // second player doesn't need to press at all). Both buttons go through
    // identical debounce_edge instances with the same DEBOUNCE_CYCLES, so
    // neither player is advantaged or disadvantaged by asymmetric debounce
    // latency. The only special case is both pulses landing in the exact
    // same clock cycle, which is treated as a tie (see ST_DUEL_G below) --
    // an earlier version of this logic instead waited for BOTH players to
    // press and compared latched timestamps, which is a different game
    // (measuring relative order) than what was actually wanted (first
    // press wins, round over); caught via simulation when a one-player
    // press correctly reached DUEL_G but the round never concluded because
    // the FSM was still waiting on a second press that was never coming.

    // ─── FSM (sequential) ────────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            result_r       <= 4'd0;
            tap_count      <= 4'd0;
            roll_cnt       <= 8'd0;
            timer_min      <= 5'd0;
            timer_sec      <= 6'd0;
            sprint_min_r   <= 5'd15;
            break_r        <= 2'd0;
            show_show_sec  <= 4'd0;
            busy_remaining <= 7'd0;
            busy_elapsed   <= 7'd0;
            busy_no_effect <= 1'b0;
            p1_armed       <= 1'b0;
            p2_armed       <= 1'b0;
            duel_delay_sec <= 4'd0;
            duel_penalty   <= 1'b0;
        end else if (mode_changed || pulse_reset) begin
            state          <= ST_IDLE;
            tap_count      <= 4'd0;
            roll_cnt       <= 8'd0;
            busy_remaining <= 7'd0;
            busy_elapsed   <= 7'd0;
            p1_armed       <= 1'b0;
            p2_armed       <= 1'b0;
            duel_penalty   <= 1'b0;
        end else begin
            busy_no_effect <= 1'b0; // default; pulsed true only on the cap-hit cycle below
            case (state)

                // ── IDLE ─────────────────────────────────────────────────
                ST_IDLE: begin
                    case (mode_sel)
                        MODE_COIN: if (pulse_go) begin
                            roll_cnt <= 8'd0;
                            state    <= ST_COIN_ROLL;
                        end
                        MODE_BUSY: if (pulse_tap) begin
                            busy_remaining <= 7'd30;
                            busy_elapsed   <= 7'd0;
                            timer_sec      <= 6'd59;
                            state          <= ST_BUSY_ACTIVE;
                        end
                        MODE_SPRINT: if (pulse_go) begin
                            sprint_min_r  <= sprint_rnd;
                            show_show_sec <= 4'd0;
                            state         <= ST_SPRINT_SHOW;
                        end
                        MODE_MEETING: if (pulse_tap) begin
                            tap_count <= 4'd1;
                            state     <= ST_MEET_SET;
                        end
                        MODE_TASK: if (pulse_tap) begin
                            tap_count <= 4'd1;
                            state     <= ST_TASK_SET;
                        end
                        MODE_DUEL: state <= ST_DUEL_R; // enter R immediately on mode select
                        default: ;
                    endcase
                end

                // ── COIN ─────────────────────────────────────────────────
                ST_COIN_ROLL: begin
                    roll_cnt <= roll_cnt + 8'd1;
                    if (roll_cnt == ROLL_CYCLES[7:0] - 8'd1) begin
                        result_r <= {3'd0, lfsr[0]};
                        state    <= ST_COIN_RESULT;
                    end
                end
                ST_COIN_RESULT: ; // hold until mode change/reset

                // ── BUSY ─────────────────────────────────────────────────
                ST_BUSY_ACTIVE: begin
                    if (tick_sec) begin
                        if (timer_sec == 6'd0) begin
                            timer_sec <= 6'd59;
                            if (busy_remaining > 7'd0) begin
                                busy_remaining <= busy_remaining - 7'd1;
                                if (busy_remaining == 7'd1) state <= ST_IDLE; // will become 0
                            end
                            if (busy_elapsed < 7'd60) busy_elapsed <= busy_elapsed + 7'd1;
                        end else begin
                            timer_sec <= timer_sec - 6'd1;
                        end
                    end
                    if (tap_double_click)
                        state <= ST_BUSY_QMARK;
                    else if (tap_single_click) begin
                        // widen to 8 bits for this comparison: busy_remaining(max 60) +
                        // 10 + busy_elapsed(max 60) can reach 130, which does not fit
                        // in the 7-bit registers themselves -- only this comparison
                        // needs the extra headroom, the registers stay 7-bit.
                        if ({1'b0, busy_remaining} + 8'd10 + {1'b0, busy_elapsed} > 8'd60)
                            busy_no_effect <= 1'b1; // at/past the cap, no-op + blink cue
                        else
                            busy_remaining <= busy_remaining + 7'd10;
                    end
                end

                ST_BUSY_QMARK: begin
                    // background clock keeps running, identical to ACTIVE
                    if (tick_sec) begin
                        if (timer_sec == 6'd0) begin
                            timer_sec <= 6'd59;
                            if (busy_remaining > 7'd0) begin
                                busy_remaining <= busy_remaining - 7'd1;
                                if (busy_remaining == 7'd1) state <= ST_IDLE; // will become 0
                            end
                            if (busy_elapsed < 7'd60) busy_elapsed <= busy_elapsed + 7'd1;
                        end else begin
                            timer_sec <= timer_sec - 6'd1;
                        end
                    end
                    if (tap_double_click) begin
                        busy_remaining <= 7'd0;
                        busy_elapsed   <= 7'd0;
                        state          <= ST_IDLE;
                    end else if (tap_single_click)
                        state <= ST_BUSY_ACTIVE; // resume, display shows live value again
                end

                // ── SPRINT ───────────────────────────────────────────────
                ST_SPRINT_SHOW: begin
                    if (tick_sec) begin
                        if (show_show_sec == 4'd2) begin
                            timer_min <= sprint_min_r - 5'd1;
                            timer_sec <= 6'd59;
                            state     <= ST_SPRINT_RUN;
                        end else
                            show_show_sec <= show_show_sec + 4'd1;
                    end
                end
                ST_SPRINT_RUN: begin
                    if (tick_sec) begin
                        if (timer_sec == 6'd0) begin
                            if (timer_min == 5'd0) begin
                                break_r <= break_rnd;
                                state   <= ST_SPRINT_DONE;
                            end else begin
                                timer_min <= timer_min - 5'd1;
                                timer_sec <= 6'd59;
                            end
                        end else
                            timer_sec <= timer_sec - 6'd1;
                    end
                end
                ST_SPRINT_DONE: ; // hold, blinks combinationally below

                // ── MEETING ──────────────────────────────────────────────
                ST_MEET_SET: begin
                    if (pulse_tap && tap_count < 4'd6)
                        tap_count <= tap_count + 4'd1;
                    if (pulse_go) begin
                        timer_min <= meet_total_min - 5'd1;
                        timer_sec <= 6'd59;
                        state     <= ST_MEET_RUN;
                    end
                end
                ST_MEET_RUN: begin
                    if (tick_sec) begin
                        if (timer_sec == 6'd0) begin
                            if (timer_min == 5'd0)
                                state <= ST_MEET_DONE;
                            else begin
                                timer_min <= timer_min - 5'd1;
                                timer_sec <= 6'd59;
                            end
                        end else
                            timer_sec <= timer_sec - 6'd1;
                    end
                end
                ST_MEET_DONE: ;

                // ── TASK ─────────────────────────────────────────────────
                ST_TASK_SET: begin
                    if (pulse_tap && tap_count < 4'd8)
                        tap_count <= tap_count + 4'd1;
                    if (pulse_go) begin
                        // simple modulo pick (acceptable for an 8-max pool;
                        // see v1 spec for the exhaustively-verified 5-window
                        // technique used where strict uniformity mattered more)
                        result_r <= (lfsr[3:0] % tap_count) + 4'd1;
                        state    <= ST_TASK_RESULT;
                    end
                end
                ST_TASK_RESULT: ;

                // ── DUEL ─────────────────────────────────────────────────
                ST_DUEL_R: begin
                    if (pulse_tap) p1_armed <= 1'b1;
                    if (pulse_p2)  p2_armed <= 1'b1;
                    if ((p1_armed || pulse_tap) && (p2_armed || pulse_p2)) begin
                        duel_delay_sec <= duel_delay_final;
                        timer_sec      <= 6'd0; // reused as elapsed-delay counter
                        state          <= ST_DUEL_C;
                    end
                end

                ST_DUEL_C: begin
                    if (pulse_tap || pulse_p2) begin
                        // false start: whichever player did NOT press wins by penalty.
                        // simultaneous presses in the same cycle -> rematch instead.
                        if (pulse_tap && pulse_p2) begin
                            p1_armed <= 1'b0; p2_armed <= 1'b0;
                            state    <= ST_DUEL_R;
                        end else begin
                            result_r     <= pulse_tap ? 4'd2 : 4'd1; // presser loses
                            duel_penalty <= 1'b1;
                            state        <= ST_DUEL_RESULT;
                        end
                    end else if (tick_sec) begin
                        if (timer_sec == {2'b00, duel_delay_sec} - 6'd1) begin
                            state <= ST_DUEL_G;
                        end else
                            timer_sec <= timer_sec + 6'd1;
                    end
                end

                ST_DUEL_G: begin
                    // First press wins outright -- the round is over the
                    // instant either player reacts, the other player never
                    // needs to press at all. Both debounce paths share the
                    // same DEBOUNCE_CYCLES, so neither player carries extra
                    // latency the other doesn't. Simultaneous presses in the
                    // exact same cycle are the one case with no fair winner
                    // -- treated as a tie, sent to a rematch.
                    if (pulse_tap && pulse_p2) begin
                        p1_armed <= 1'b0; p2_armed <= 1'b0;
                        state    <= ST_DUEL_R;
                    end else if (pulse_tap) begin
                        result_r     <= 4'd1;
                        duel_penalty <= 1'b0;
                        state        <= ST_DUEL_RESULT;
                    end else if (pulse_p2) begin
                        result_r     <= 4'd2;
                        duel_penalty <= 1'b0;
                        state        <= ST_DUEL_RESULT;
                    end
                end

                ST_DUEL_RESULT: ; // hold; blinks combinationally below

                default: state <= ST_IDLE;
            endcase
        end
    end

    // ─── Display digit codes (combinational) ────────────────────────────────
    reg [3:0] dig0, dig1, dig2, dig3; // leftmost .. rightmost
    reg [4:0] disp_minutes; // shared scratch for any 2-digit minutes readout

    always @* begin
        dig0 = 4'd10; dig1 = 4'd10; dig2 = 4'd10; dig3 = 4'd10; // all blank by default
        disp_minutes = 5'd0;
        case (state)
            ST_IDLE: ; // all blank

            ST_COIN_ROLL: begin
                dig3 = {2'b00, lfsr[1:0]} % 4'd2; // fast-changing animation digit
            end
            ST_COIN_RESULT: dig3 = result_r[0] ? 4'd1 : 4'd0;

            ST_BUSY_ACTIVE, ST_BUSY_QMARK: begin
                if (state == ST_BUSY_QMARK) begin
                    dig3 = 4'd15; // '?'
                end else begin
                    dig2 = (busy_remaining >= 7'd10) ? (busy_remaining / 7'd10) : 4'd10;
                    dig3 = busy_remaining % 7'd10;
                    if (busy_no_effect && blink_fast) begin dig2 = 4'd10; dig3 = 4'd10; end
                end
            end

            ST_SPRINT_SHOW, ST_SPRINT_RUN: begin
                disp_minutes = (state == ST_SPRINT_SHOW) ? sprint_min_r : timer_min;
                dig2 = (disp_minutes >= 5'd10) ? (disp_minutes / 5'd10) : 4'd10;
                dig3 = disp_minutes % 5'd10;
            end
            ST_SPRINT_DONE: begin
                if (blink_fast) dig3 = {2'b00, break_r} + 4'd1;
                else begin dig2 = 4'd10; dig3 = 4'd10; end
            end

            ST_MEET_SET:  dig3 = tap_count;
            ST_MEET_RUN: begin
                dig2 = (timer_min >= 5'd10) ? (timer_min / 5'd10) : 4'd10;
                dig3 = timer_min % 5'd10;
            end
            ST_MEET_DONE: begin
                if (!blink_fast) begin dig2 = 4'd10; dig3 = 4'd10; end
                else begin dig2 = 4'd8; dig3 = 4'd8; end
            end

            ST_TASK_SET:    dig3 = tap_count;
            ST_TASK_RESULT: dig3 = result_r;

            ST_DUEL_R: dig3 = 4'd14; // 'r'
            ST_DUEL_C: dig3 = 4'd12; // 'C'
            ST_DUEL_G: dig3 = 4'd13; // 'G'
            ST_DUEL_RESULT: begin
                if (duel_penalty) begin
                    dig3 = result_r; // steady (non-blinking) = penalty win cue
                end else begin
                    if (blink_fast) dig3 = result_r; else dig3 = 4'd10;
                end
            end

            default: ;
        endcase
    end

    // ─── TM1637 driver instance ──────────────────────────────────────────────
    wire tm_clk_w, tm_dio_out_w, tm_dio_oe_w, tm_dio_in_w;
    assign tm_dio_in_w = uio_in[1];

    tm1637_driver #(
        .HALF_BIT_CYCLES(TM_HALF_BIT_CYCLES)
    ) tm_disp (
        .clk(clk), .rst_n(rst_n),
        .d0(dig0), .d1(dig1), .d2(dig2), .d3(dig3),
        .tm_clk_out(tm_clk_w),
        .tm_dio_out(tm_dio_out_w),
        .tm_dio_oe(tm_dio_oe_w),
        .tm_dio_in(tm_dio_in_w)
    );

    // ─── Output assignments ───────────────────────────────────────────────────
    wire duel_penalty_out = (state == ST_DUEL_RESULT) && duel_penalty;

    assign uo_out = {
        duel_penalty_out,
        (state == ST_BUSY_ACTIVE) || (state == ST_BUSY_QMARK),                      // busy_light
        (state == ST_SPRINT_DONE) || (state == ST_MEET_DONE) || (state == ST_DUEL_RESULT), // alarm
        (state == ST_SPRINT_RUN)  || (state == ST_MEET_RUN)  ||
            (state == ST_BUSY_ACTIVE) || (state == ST_BUSY_QMARK),                  // timer_run
        meet_warn,
        p2_armed,
        p1_armed,
        (state != ST_IDLE)
    };

    assign uio_out = {6'b000000, tm_dio_oe_w ? tm_dio_out_w : 1'b1, tm_clk_w};
    assign uio_oe  = {6'b000000, tm_dio_oe_w, 1'b1};

    wire _unused = &{ena, uio_in[7:2], uio_in[0], ui_in[7], 1'b0};

endmodule
