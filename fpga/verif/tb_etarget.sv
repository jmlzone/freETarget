/*----------------------------------------------------------------------
  testbech for etarget
 ----------------------------------------------------------------------*/
`timescale 1ns/1ns
module tb_etarget();
logic clk64M,reset_n;
logic mic_north, mic_south, mic_east, mic_west;
logic mode;
logic DIPA, DIPB, DIPC, DIPD;


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
    {DIPD, DIPC, DIPB, DIPA} = 4'ha;
    reset_n = 0;
    repeat(5) @(negedge clk64M) ;
    reset_n = 1;
    repeat(5) @(negedge clk64M) ;
    $display("Initial read all");
    i2c.dev_addr='h10;
    i2c.reg_addr = 0;
    i2c.read(32);
    i2c.display_read(32);
    i2c.reg_addr = 1; //control
    i2c.data[0] = 3;  // clear[0] and stop[1]
    i2c.write(1);
    i2c.data[0] = 0;  // clear[0] and stop[1]
    i2c.write(1);
    i2c.data[0] = 8'b01000100;  // rec_sel
    i2c.data[1] = 8'b10111111;  // rec_ctl
    i2c.data[2] = 8'h20;  // trigger_depth
    i2c.reg_addr = 8'h0d; //control
    i2c.write(3); // burst write the controls

    #1us;
    mic_north = 1;
    #1us;
    mic_north = 0;
    mic_south = 1;
    #1us;
    mic_south = 0;
    mic_east  = 1;
    #1us;
    mic_east  = 0;
    mic_west  = 1;
    #1us;
    mic_west  = 0;
    #100us;
    i2c.reg_addr = 1; //control
    i2c.data[0] = 2;  //stop[1]
    i2c.write(1);
    $display("Read target counter");
    i2c.reg_addr = 2;
    i2c.read(8);
    i2c.display_read(8);
    
    #1ms; // things should be stable ADC's midscale
    $display("Read all after 1 ms");
    i2c.reg_addr = 0;
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
    i2c.reg_addr = 'h30;
    i2c.read(1023);
    i2c.display_read(1023);
    #100;
    
    $finish(2);
  end // initial begin
always
  begin
    #8 clk64M = 0;
    #8 clk64M = 1;
  end

endmodule // tb_etarget

