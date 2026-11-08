// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Symmetric / asymmetric saturating clamp for a signed value.
//
// Input is `WIDE_W` bits (post-sum width that may overflow the output
// range), the limits are `OUT_W` bits (the final output width). The
// module emits the clamped `OUT_W`-bit value plus a `saturated_o` flag
// that the integrator uses to gate anti-windup updates.
//
// Combinational. No registers. The synthesiser collapses this to a pair
// of magnitude comparisons plus a 3:1 mux.

module pid_saturate #(
  parameter int WIDE_W = 40,
  parameter int OUT_W  = 16
) (
  input  logic signed [WIDE_W-1:0] in_i,
  input  logic signed [OUT_W-1:0]  out_min_i,
  input  logic signed [OUT_W-1:0]  out_max_i,
  output logic signed [OUT_W-1:0]  out_o,
  output logic                     saturated_hi_o,
  output logic                     saturated_lo_o
);

  logic signed [WIDE_W-1:0] min_ext;
  logic signed [WIDE_W-1:0] max_ext;

  always_comb begin
    min_ext = WIDE_W'(out_min_i);
    max_ext = WIDE_W'(out_max_i);

    if (in_i > max_ext) begin
      out_o          = out_max_i;
      saturated_hi_o = 1'b1;
      saturated_lo_o = 1'b0;
    end else if (in_i < min_ext) begin
      out_o          = out_min_i;
      saturated_hi_o = 1'b0;
      saturated_lo_o = 1'b1;
    end else begin
      out_o          = in_i[OUT_W-1:0];
      saturated_hi_o = 1'b0;
      saturated_lo_o = 1'b0;
    end
  end

endmodule
