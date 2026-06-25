`default_nettype none
`timescale 1ns / 1ps

module tb ();

  // Dump signals for GTKWave
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // Inputs
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;

  // Outputs
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  // DUT instantiation
 tt_um_luck_engine #(
      .PRESCALE          (10),
      .ROLL_CYCLES       (48),
      .TM_HALF_BIT_CYCLES(2),
      .CLICK_WINDOW_CYCLES(20)
  ) user_project (

`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif

      .ui_in   (ui_in),    // Dedicated inputs
      .uo_out  (uo_out),   // Dedicated outputs
      .uio_in  (uio_in),   // Bidirectional inputs
      .uio_out (uio_out),  // Bidirectional outputs
      .uio_oe  (uio_oe),   // Output enables
      .ena     (ena),      // Design selected
      .clk     (clk),      // Clock
      .rst_n   (rst_n)     // Active-low reset
  );

endmodule

`default_nettype wire
