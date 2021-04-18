/*----------------------------------------------------------------------
 simple test module for adc
----------------------------------------------------------------------*/
`timescale 1ns/1ns
module test_sd_adc();
logic clk, ares, cmp;

logic sdm, wr;
localparam WIDTH=8;
localparam ACC_WIDTH=10;
localparam LPF_DEPTH=3;

logic [(WIDTH-1):0] q;

int aval, intval;


// instanitiate dut
sd_adc #(.WIDTH(WIDTH), .ACC_WIDTH(ACC_WIDTH), .LPF_DEPTH(LPF_DEPTH)) dut(.*);

// clock source
always begin
  #5 clk = 0;
  #5 clk = 1;
end
initial
  begin
    $dumpfile("adc.vcd");
    $dumpvars(0,test_sd_adc);
    
    ares = 1;
    intval = 0;
    aval = 5;
    repeat(5) @(negedge clk) ;
    ares = 0;
    repeat (10)
      begin
	
	repeat (5) 
	  begin
	    @(posedge wr)
	      if(intval != aval) $write("Error:");
	    $display(" adc write int val =%d input was %d", intval, aval);
	  end
	aval = $random() % 8'hff;
      end // repeat (10)
    
    $finish(2);
    
  end // initial begin

// integrator response
always @(negedge clk)
  if(sdm)
    intval=intval + 1;
  else if(!ares)
    intval = intval -1;

assign comp = aval > intval;
endmodule // test_sd_adc


    
