/*----------------------------------------------------------------------
  testbech for etarget
 ----------------------------------------------------------------------*/
`timescale 1ns/1ns
module tb_etarget();
logic clk64M,ares;
logic mic_north, mic_south, mic_east, mic_west;
logic mode;

wire  sd0p, sd1p,sd2p,sd3p,sd4p,sd5p;
wire  pcm0,pcm1,pcm2,pcm3,pcm4,pcm5;

wire sda, scl;
// tb transactors;
i2c_master i2c(.*);
adc_drv a0 (.clk(clk64M),.sd(sd0p),.pcm(pcm0));
adc_drv a1 (.clk(clk64M),.sd(sd1p),.pcm(pcm1));
adc_drv a2 (.clk(clk64M),.sd(sd2p),.pcm(pcm2));
adc_drv a3 (.clk(clk64M),.sd(sd3p),.pcm(pcm2));
adc_drv a4 (.clk(clk64M),.sd(sd4p),.pcm(pcm2));
adc_drv a5 (.clk(clk64M),.sd(sd5p),.pcm(pcm5));

//dut
etarget dut(.sda_pin(sda), .*);
initial
  begin
    $dumpfile("etarget.vcd");
    $dumpvars(0,tb_etarget);

    mode = 0;
    mic_north = 0;
    mic_south = 0;
    mic_east  = 0;
    mic_west  = 0;
    ares = 1;
    repeat(5) @(negedge clk64M) ;
    ares = 0;
    repeat(5) @(negedge clk64M) ;
    $display("Initial read all");
    i2c.dev_addr='h10;
    i2c.reg_addr = 0;
    i2c.read(32);
    i2c.display_read(32);
    #1ms; // things should be stable ADC's midscale
    $display("Read all after 1 ms");
    i2c.read(32);
    i2c.display_read(32);
    fork
      a0.impulse(0,a0.midScale,10240,a0.maxVal,100000,10240);
      a1.impulse(896,a0.midScale,10240,a0.maxVal,100000,10240);
      a2.impulse(1536,a0.midScale,10240,a0.maxVal,100000,10240);
      a3.impulse(0,a0.midScale,10240,10,100000,10240);
      a4.impulse(896,a0.midScale,10240,10,100000,10240);
      a5.impulse(1536,a0.midScale,10240,10,100000,10240);
      join
    #1ms; // things should be stable ADC's midscale
    $display("Final adc read");
    i2c.dev_addr='h10;
    i2c.reg_addr = 'h10;
    i2c.read(6);
    i2c.display_read(6);
    #100;
    
    $finish(2);
  end // initial begin
always
  begin
    #8 clk64M = 0;
    #8 clk64M = 1;
  end

endmodule // tb_etarget
