`default_nettype none
`timescale 1ns / 1ps

module tb ();

  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;

  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

`ifdef GL_TEST
  // ── Gate-level simulation ────────────────────────────────────────────────
  // The synthesized netlist has no parameters — they were resolved at
  // synthesis time and baked into the gates. PRESCALE=10 is irrelevant
  // here because the GLS netlist already has a fixed (synthesized) clock
  // domain and the testbench drives timing differently in GLS anyway.
  wire VPWR = 1'b1;
  wire VGND = 1'b0;

  tt_um_luck_engine user_project (
      .VPWR   (VPWR),
      .VGND   (VGND),
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

`else
  // ── RTL simulation ───────────────────────────────────────────────────────
  // Sim-fast parameters passed explicitly so both Icarus and Questa pick
  // them up without any -P / -g flag juggling in the Makefile.
  tt_um_luck_engine #(
      .PRESCALE           (10),
      .ROLL_CYCLES        (48),
      .TM_HALF_BIT_CYCLES (2),
      .CLICK_WINDOW_CYCLES(20)
  ) user_project (
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

`endif

endmodule
`default_nettype wire
