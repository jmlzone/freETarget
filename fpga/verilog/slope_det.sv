module slope_det (
  input        clk64M,
  input        clk8M,
  input        reset_n,
  input        div,
  input        comp,
  input        slope_neg, 
  input [5:0]  slope, 
  output logic sdm,
  output logic det,
  output logic [2:0] conv
  );
logic [2:0]    acc;
logic [2:0]    srLPF0;
logic [2:0]    srLPF1;
logic [2:0]    srLPF2;
logic [2:0]    srLPF3;
logic [2:0]    srLPF4;
logic [2:0]    srLPF5;
logic [2:0]    srLPF6;
logic [2:0]    srLPF7;
logic [5:0]    lpf2, lpf3, diff;
logic 	       det_nxt;

always_ff @(posedge clk64M or negedge reset_n)
  if(~reset_n)
    begin
      sdm    <= 0;
      acc    <= 0;
    end
  else
    begin
       sdm   <= comp;
      if(div)
	acc<= 0;
      else
	acc<=acc+comp;
    end // else: !if(~reset_n)
always_ff @(posedge clk8M or negedge reset_n)
  if(~reset_n)
    begin
      // second stage lpf
      srLPF0 <= 0;
      srLPF1 <= 0;
      srLPF2 <= 0;
      srLPF3 <= 0;
      // third stage lpf
      srLPF4 <= 0;
      srLPF5 <= 0;
      srLPF6 <= 0;
      srLPF7 <= 0;
      // final output
      det <= 0;
    end
  else
    begin
      srLPF0 = acc;
      srLPF1 <= srLPF0;
      srLPF2 <= srLPF1;
      srLPF3 <= srLPF2;
      srLPF4 <= srLPF3;
      srLPF5 <= srLPF4;
      srLPF6 <= srLPF5;
      srLPF7 <= srLPF6;
      det    <= det_nxt;
    end // else: !if(~reset_n)

always_comb
  begin
    lpf2 = srLPF0 +  srLPF1 +  srLPF2 + srLPF3;
    lpf3 = srLPF4 +  srLPF5 +  srLPF6 + srLPF7;
    conv = lpf3 [5:2];
    if(slope_neg)
      diff = lpf3 - lpf2;
    else
      diff = lpf2 - lpf3;
    det_nxt = (diff > slope);
  end

endmodule // slope_det

