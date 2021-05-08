/*----------------------------------------------------------------------
Slope detector on the front end of what coudl be a delta sigma or
integrating adc.  
 with 64 MHz clock use 3.3K and 200 pf cap.  
      48 MHz clock use 3.3K and 400 pf cap.  
 
 New_Value = Current_Value + (RC_Input_Value – Current_Value) * (1-exp(-T/RC)) 

 the factor (1-exp(-T/RC)) is a constant calculate as 2 (0x02) so its
 a simple shift and add 0.01565893 = 2/128 == 0.015625 
 thus divide by 64 before adding or >>6 if you are actually integrating.
----------------------------------------------------------------------*/

module slope_det (
  input clk64M, // SD modulator and first stage at 64MHz
  input clk8M,  // filters and down stream logic at 8MHz. Known phase relationship
  input reset_n, 
  input div,    // this fires when the 8MHz is divided down from the 64MHz for the know phase
  input comp,   // input from the comparitor in the LVDS pin used as the ADC
  input slope_neg, // for slop detection positive or negative slope
  input [5:0] slope, // delta slope looked for (at 8mhz (0.125us) for delta t)
  output logic sdm,  // The sdm bit back to the analog integrator
  output logic det,  // slope detected (at 8mhz rate)
  output logic [2:0] conv // the 3 bit adc value at 8MHz.
); 
logic [2:0] acc;
logic [2:0] srLPF0;  // It would have been nice to be a 2d array but iVerilog / VCD does not handle 2d arrays.
logic [2:0] srLPF1;
logic [2:0] srLPF2;
logic [2:0] srLPF3; 
logic [2:0] srLPF4; 
logic [2:0] srLPF5; 
logic [2:0] srLPF6;
logic [2:0] srLPF7; 
logic [5:0] lpf2, lpf3, diff; 
logic det_nxt;

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

