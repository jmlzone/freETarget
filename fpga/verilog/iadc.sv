/*----------------------------------------------------------------------
integrating adc, model the integration of the RC vs countign ones in a delta sigma ADC.
it just makes sense, there is even an article:
https://intentionallogic.com/2018/01/23/adc-in-an-fpga-part-1/
https://intentionallogic.com/2018/01/23/adc-in-an-fpga-part-2/
http://intentionallogic.com/wp-content/uploads/2018/01/adc_constants.xls
not patentable due to prior art and also being obvious.

with 64 MHz clock use 3.3K and 200 pf cap.
     48 MHz clock use 3.3K and 400 pf cap.
New_Value = Current_Value + (RC_Input_Value – Current_Value) * (1-exp(-T/RC))
the factor (1-exp(-T/RC)) is a constant calculate as 2 (0x02) so its a simple shift and add
 0.01565893 = 2/128 == 0.015625 thus divide by 64 before adding or >>6
----------------------------------------------------------------------*/

module iadc (
  input clk,
  input reset_n,
  input comp,
  output logic sdm,
  output logic [7:0] q,
  output logic wr
  );

logic [11:0]    acc; //, inc; //, dec;
logic [5:0]    inc;
//logic [9:0]    acc; //, inc; //, dec;
//logic [3:0]    inc;
logic [2:0]    count;
//localparam DIV = 6; // 48 mhz
localparam DIV = 8; // 64 mhz
logic [7:0]    srLPF0;
logic [7:0]    srLPF1;
logic [7:0]    srLPF2;
logic [7:0]    srLPF3;
int 	       i ; // loopvar unrolled
always_ff @(posedge clk or negedge reset_n)
  if(~reset_n)
    begin
      sdm    <= 0;
      count  <= 0;
      acc    <= 0;
      inc    <= 0;
      // second stage lpf
      srLPF0 <= 0;
      srLPF1 <= 0;
      srLPF2 <= 0;
      srLPF3 <= 0;
      // final output
      q <= 0;
    end
  else
    begin
      // pre calculate increment and decriment amount
      if(! (&acc)) 
	inc <= (12'hFFA - acc) >> 6;
//	inc <= (10'h3FA - acc) >> 6;
      else
	inc <= 0;
      //dec <= acc;
      // SDM and divider
      sdm   <= comp;
      if(count == (DIV -1))
	begin
	  count <= 0;
	  wr <= 1;
	end
      else
	begin
	  count <= count + 1'b1;
	  wr <= 0;
	end
      // accumulator
      if(comp)
	acc <= acc + inc;
      else
	acc <= acc - (acc>>6); // decriment amount
      if(wr)
	begin
	  srLPF0 = acc[11:4]; // truncate value to 8 bits
//	  srLPF0 = acc[9:2]; // truncate value to 8 bits
	  srLPF1 <= srLPF0;
	  srLPF2 <= srLPF1;
	  srLPF3 <= srLPF2;
	  q <= (10'(srLPF0 +  srLPF1 +  srLPF2 + srLPF3)) >>2;
	end
    end // else: !if(~reset_n)
endmodule // iadc

