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
wire  led_red, led_green, led_blue;
wire [2:0] debug;
int 	   i,j;
logic 	   passed;
time 	   random_delay;
time 	   now;
integer    x, x1, x2, xd;

wire sda, scl;
// tb transactors;
i2c_master i2c(.*);
adc_drv a0 (.clk(clk64M),.sd(sd0p),.pcm(pcm0));
adc_drv a1 (.clk(clk64M),.sd(sd1p),.pcm(pcm1));
adc_drv a2 (.clk(clk64M),.sd(sd2p),.pcm(pcm2));
adc_drv a3 (.clk(clk64M),.sd(sd3p),.pcm(pcm2));
adc_drv a4 (.clk(clk64M),.sd(sd4p),.pcm(pcm2));
adc_drv a5 (.clk(clk64M),.sd(sd5p),.pcm(pcm5));
localparam STARTDELAY=124736;
localparam   ENDDELAY= 84096;
localparam DELAYERROR = STARTDELAY + ENDDELAY;

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
// mimic post like tests
    $display("Post 2A check stopping and arming 5x");
    for(i=0;i!=5; i++)
      begin
	stop_counters;
	arm_counters;
	is_running(0);
      end
    passed = 1;
    $display("Post 2B check stopping and arming 10x");
    for(i=0;i!=10; i++)
      begin
	random_delay = (($random() & 16'hfff) +1 ) * 1000;
	stop_counters;
	arm_counters;
	trip_counters;
	is_running(4'hf);
	$display("Iteration %d random delay %t",i,random_delay);
	#random_delay;
	stop_counters;
	is_running(4'h0);
	xd = ((random_delay + DELAYERROR) * 8) / 1024;
	for(j=0;j<=3;j++)
	  begin
	    read_counter(j,x1);
	    read_counter(j,x2);
	    x=x1 - xd;
	    x= (x<0) ? -x : x;
	    if(x1!=x2)
	      begin
		passed = 0;
		$display("Error: counter reads did not match x1=%h, x2=%h",x1,x2);
	      end
	    if(x>1000)
	      begin
		passed = 0;
		$display("Error: Counter value faled, expected %d got %d",xd,x1);
	      end
	    else
	      	$display("Pass: Counter value: expected %d got %d",xd,x1);
	  end // for (j=0;j<=3;j++)
      end // for (i=0;i!=10; i++)
    if(passed)
      $display("Passed");
    else
      $display("failed");
    $finish(2);
    
	    
    
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

    i2c.reg_addr = 2;
    i2c.read(2);
    i2c.display_read(2);

    i2c.reg_addr = 2;
    i2c.read(2);
    i2c.display_read(2);
    
    i2c.reg_addr = 2;
    i2c.read(2);
    i2c.display_read(2);
    
    i2c.reg_addr = 2;
    i2c.read(2);
    i2c.display_read(2);
    
    i2c.reg_addr = 2;
    i2c.read(2);
    i2c.display_read(2);
    
    i2c.reg_addr = 2;
    i2c.read(2);
    i2c.display_read(2);
    
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

//i2c related tasks and stuff
localparam FPGA_ADDR = 'h10;
localparam CONTROL = 1;
localparam NORTH_LOW = 2;
localparam NORTH_HIGH = 3;
localparam EAST_LOW = 4;
localparam EAST_HIGH = 5;
localparam SOUTH_LOW = 6;
localparam SOUTH_HIGH = 7;
localparam WEST_LOW = 8;
localparam WEST_HIGH = 9;
localparam STATUS = 'h0a;
localparam DIP = 'h0b;
localparam SLOPECONTROL = 'h0c;
localparam RECSEL = 'h0d;
localparam RECCTL = 'h0e;
localparam TRIGGERDEPTH = 'h0f;
localparam CONV0 = 'h10;
localparam CONV1 = 'h11;
localparam CONV2 = 'h12;
localparam CONV3 = 'h13;
localparam CONV4 = 'h14;
localparam CONV5 = 'h15;
localparam TRACE01 = 'h30;
localparam TRACE23 = 'h31;

// Bit definitions in control
localparam CLEAR = 1<<0;
localparam STOP  = 1<<1;
localparam QUIET = 1<<2;
localparam START = 1<<3;

// Bit definitions in slope control
localparam SLOPE_MODE = 1<<7;
localparam SLOPE_NEG  = 1<<6;

// bit definitions RECCTL
localparam REC_ENABLE = 1<<7;
task i2c_write_reg(input int dev, input int regaddr, input int val);
  begin
    i2c.dev_addr=dev;
    i2c.reg_addr=regaddr;
    i2c.data[0] = val;
    i2c.write(1);
  end
endtask // i2c_write_reg
task i2c_read_reg(input int dev, input int regaddr);
  begin
    i2c.dev_addr=dev;
    i2c.reg_addr=regaddr;
    i2c.read(1);
  end
endtask // i2c_read_reg

task stop_counters;
  begin
    i2c_write_reg(FPGA_ADDR,CONTROL,(STOP|QUIET));
  end
endtask // stop_counters
task arm_counters;
  begin
    i2c_write_reg(FPGA_ADDR,CONTROL,(CLEAR|STOP));
  end
endtask // arm_counters
task is_running(input [3:0] expected);
  begin
    i2c_read_reg(FPGA_ADDR,STATUS);
    if((i2c.data[0] & 'h0f) != expected)
      $display("ERROR %t: Is_running expected %b got %b",$time,expected,i2c.data[0][3:0]);
  end
endtask // is_running
task read_counter(input int ctr, output [15:0] r);
  begin
    i2c.dev_addr=FPGA_ADDR;
    i2c.reg_addr=(ctr*2)+2;
    i2c.read(2);
    r={i2c.data[1],i2c.data[0]};
  end
endtask // read_counter
task trip_counters;
  begin
    i2c_write_reg(FPGA_ADDR,CONTROL,START);
  end
endtask // trip_counters

endmodule // tb_etarget

