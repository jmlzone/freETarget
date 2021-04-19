/*----------------------------------------------------------------------
   Sigma delta ADC based on concepts of many papers and Ideas from
   lattice website.
----------------------------------------------------------------------*/

module sd_adc #(parameter WIDTH=8, ACC_WIDTH=10, LPF_DEPTH=3) (
  input clk,
  input ares,
  input comp,
  output logic sdm,
  output logic [(WIDTH-1):0] q,
  output logic wr
  );

logic [(ACC_WIDTH-1):0] acc,count;
logic [(WIDTH-1):0] decRaw,decLPF;
logic 		    last, decVld;
localparam TRUNK = ACC_WIDTH - WIDTH;

logic [(LPF_DEPTH -1):0] lpfCount;
logic [(WIDTH + LPF_DEPTH -1):0] accLPF;

always_ff @(posedge clk or posedge ares)
  if(ares)
    begin
      sdm    <= 0;
      acc    <= 0;
      decRaw <= 0;
      count  <= 0;
      last   <= 0;
      decVld <= 0;
      // second stage lpf
      decLPF <= 0;
      lpfCount <= 0;
      accLPF   <= 0;
      // final output
      q <= 0;
    end
  else
    begin
      // SDM and first stage decimater
      sdm   <= comp;
      count <= count +1;
      last  <= &count; // first stage decimator is power of 2 of the first accumulator
      if(last)
	begin
	  acc  <= sdm;
	  decRaw <= acc >> TRUNK;
	  decVld <= 1'b1;
	end
      else
	begin
	  if(sdm)
	    acc  <= acc + 1;
	  else
	    acc  <= acc -1;
    
	  decVld <= 1'b0;
	end // else: !if(last)
      // second stage lpf
      if(decVld)
	begin
	  lpfCount <= lpfCount +1;
	  if(& lpfCount)
	    begin
	      accLPF <= decRaw;
	      wr <= 1'b1;
	      q  <= accLPF >> LPF_DEPTH;
	    end
	  else
	    begin
	      accLPF <= accLPF + decRaw;
	      wr <= 1'b0;
	    end
	end // if (decVld)
    end // else: !if(ares)
endmodule // sd_adc
