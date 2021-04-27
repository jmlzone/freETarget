`timescale 1ns/1ns
module adc_drv(input clk, output logic sd, input pcm);

int adrv, aint, inc, dec;
localparam maxVal = 1<<16; // do 16 bit math for the analog value
localparam midScale = maxVal/2;
wire [7:0] res;
assign res = aint>>8;

initial
  begin
    sd = 0;
    adrv = midScale;
    aint = 0;
  end

always @(negedge clk)
  begin
    inc = (maxVal - aint) / 64;
    dec = aint / 64;
    // integrator
    if(pcm)
      aint <= aint + inc;
    else
      aint <= aint - dec;
    // comparitor
    sd <= (adrv > aint);
    
  end

task ramp(input int delay, initVal, transTime, finalVal);
int rampVal, rampStep, i;
  begin
    adrv = initVal;
    // ramp in 256 steps
    # delay;
    rampVal = (finalVal - initVal) / 256;
    rampStep = transTime/256;
    for(i=0; i<256;i++)	
      begin
	#rampStep ;
	adrv = adrv + rampVal;
	//$display("Ramping %m time %d step %d %d (driving %d)", $time(), i, rampVal, adrv);
      end
  end
endtask // ramp

task impulse(input int delay, initVal, transTime, peakVal, duration, fallTime);
  begin
    ramp(delay, initVal, transTime, peakVal);
    ramp(duration, peakVal, fallTime, initVal);
  end
endtask
  
  
endmodule // adc_drv

