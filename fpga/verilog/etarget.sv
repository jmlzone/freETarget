module etarget(
  input        clk64M,
  input        ares,
  input        scl,
  inout        sda_pin,
  input        mic_north,
  input        mic_south,
  input        mic_east,
  input        mic_west,
  input        sd0p,
  input        sd1p,
  input        sd2p,
  input        sd3p,
  input        sd4p,
  input        sd5p,
  input        mode,
  input DIPA,
  input DIPB,
  input DIPC,
  input DIPD,
        
  output logic pcm0,
  output logic pcm1,
  output logic pcm2,
  output logic pcm3,
  output logic pcm4,
  output logic pcm5
  );
// pin instances for non trivial io
logic 	 sda, sda_drv_lo;
logic 	 clk_8m;
logic 	 comp0, comp1, comp2, comp3, comp4, comp5;
// I2C SDA Pin
SB_IO #(
    .PIN_TYPE(6'b 1010_01),
    .PULLUP(1'b0)
)   myi2cpin (
    .PACKAGE_PIN(sda_pin),
    .OUTPUT_ENABLE(sda_drv_lo),
    .D_OUT_0(1'b0),
    .D_IN_0(sda)
);
/*
//comparitor pins
SB_IO #(
    .PIN_TYPE(6'b000000),
    .PULLUP(1'b0),
    .IO_STANDARD("SB_LVDS_INPUT")
) comp_pin0 (
  .PACKAGE_PIN(sd0p),
  .LATCH_INPUT_VALUE ( ),
  .CLOCK_ENABLE ( ),
  .INPUT_CLK (clk64M),
  .OUTPUT_CLK ( ),
  .OUTPUT_ENABLE ( ),
  .D_OUT_0 ( ),
  .D_OUT_1 ( ),
  .D_IN_0 (comp0),
  .D_IN_1 ()
);
SB_IO #(
    .PIN_TYPE(6'b000000),
    .PULLUP(1'b0),
    .IO_STANDARD("SB_LVDS_INPUT")
) comp_pin1 (
  .PACKAGE_PIN(sd1p),
  .LATCH_INPUT_VALUE ( ),
  .CLOCK_ENABLE ( ),
  .INPUT_CLK (clk64M),
  .OUTPUT_CLK ( ),
  .OUTPUT_ENABLE ( ),
  .D_OUT_0 ( ),
  .D_OUT_1 ( ),
  .D_IN_0 (comp1),
  .D_IN_1 ()
);
SB_IO #(
    .PIN_TYPE(6'b000000),
    .PULLUP(1'b0),
    .IO_STANDARD("SB_LVDS_INPUT")
) comp_pin2 (
  .PACKAGE_PIN(sd2p),
  .LATCH_INPUT_VALUE ( ),
  .CLOCK_ENABLE ( ),
  .INPUT_CLK (clk64M),
  .OUTPUT_CLK ( ),
  .OUTPUT_ENABLE ( ),
  .D_OUT_0 ( ),
  .D_OUT_1 ( ),
  .D_IN_0 (comp2),
  .D_IN_1 ()
);
SB_IO #(
    .PIN_TYPE(6'b000000),
    .PULLUP(1'b0),
    .IO_STANDARD("SB_LVDS_INPUT")
) comp_pin3 (
  .PACKAGE_PIN(sd3p),
  .LATCH_INPUT_VALUE ( ),
  .CLOCK_ENABLE ( ),
  .INPUT_CLK (clk64M),
  .OUTPUT_CLK ( ),
  .OUTPUT_ENABLE ( ),
  .D_OUT_0 ( ),
  .D_OUT_1 ( ),
  .D_IN_0 (comp3),
  .D_IN_1 ()
);
SB_IO #(
    .PIN_TYPE(6'b000000),
    .PULLUP(1'b0),
    .IO_STANDARD("SB_LVDS_INPUT")
) comp_pin4 (
  .PACKAGE_PIN(sd4p),
  .LATCH_INPUT_VALUE ( ),
  .CLOCK_ENABLE ( ),
  .INPUT_CLK (clk64M),
  .OUTPUT_CLK ( ),
  .OUTPUT_ENABLE ( ),
  .D_OUT_0 ( ),
  .D_OUT_1 ( ),
  .D_IN_0 (comp4),
  .D_IN_1 ()
);
SB_IO #(
    .PIN_TYPE(6'b000000),
    .PULLUP(1'b0),
    .IO_STANDARD("SB_LVDS_INPUT")
) comp_pin5 (
  .PACKAGE_PIN(sd5p),
  .LATCH_INPUT_VALUE ( ),
  .CLOCK_ENABLE ( ),
  .INPUT_CLK (clk64M),
  .OUTPUT_CLK ( ),
  .OUTPUT_ENABLE ( ),
  .D_OUT_0 ( ),
  .D_OUT_1 ( ),
  .D_IN_0 (comp5),
  .D_IN_1 ()
);
*/
/*----------------------------------------------------------------------
  register address map
 0 RO 0x10 -- we only want to shoot 10's
 1 RW control 
 2 RO north low
 3 RO north high
 4 RO south low
 5 RO south hi
 6 RO east low
 7 RO east high
 8 RO west low
 9 RO west hi
 a run status west,south, east,north
----------------------------------------------------------------------*/

logic [7:0] version;
assign version = 8'h10;
logic [7:0] control;
logic 	    clear;
logic 	    stop;
logic       quiet;

logic [15:0] count_north;
logic [15:0] count_south;
logic [15:0] count_east;
logic [15:0] count_west;
logic [5:0]  addr;
logic [7:0]  read_data;
logic [7:0]  write_data;
logic [7:0]  conv0, conv1, conv2, conv3, conv4, conv5;

logic 	     read;
logic 	     write;
logic 	     cnt_clk;
logic 	     run_north, run_south, run_east, run_west;


//I2c slave port
jml_i2c #(.MYI2C_ADDR('h10)) i2c(
  .reset_n(~ares),
  .*
  );
// clock divider
// 64 MHZ will be used for sd adc
// 8 mhz for direction counter
logic [2:0]  clkdiv;
logic 	     clk8m;
always @(posedge clk64M or posedge ares)
  if(ares)
    clkdiv <= 0;
  else
    clkdiv <= clkdiv + 1'b1;
assign clk8m = &clkdiv;


// read mux
always @*
  begin
    case (addr)
      6'h0 : read_data = version;
      6'h1 : read_data = control;
      6'h2 : read_data = count_north[7:0];
      6'h3 : read_data = count_north[15:8];
      6'h4 : read_data = count_south[7:0];
      6'h5 : read_data = count_south[15:8];
      6'h6 : read_data = count_east[7:0];
      6'h7 : read_data = count_east[15:8];
      6'h8 : read_data = count_west[7:0];
      6'h9 : read_data = count_west[15:8];
      6'ha : read_data = {4'h0, run_west,run_south,run_east,run_north};
      6'hb : read_data = {4'h0, DIPA,DIPB,DIPC,DIPD};
      6'h10: read_data = conv0;
      6'h11: read_data = conv1;
      6'h12: read_data = conv2;
      6'h13: read_data = conv3;
      6'h14: read_data = conv4;
      6'h15: read_data = conv5;
      
      default: read_data = 8'h00;
    endcase // case (addr)
  end // always @ *
//write data
always @(posedge clk8m or posedge ares)
  begin
    if(ares)
      begin
	control <= 8'b0;
      end
    else
      begin
	if(write) begin
	  if(addr == 6'h1) control <= write_data;
      end
      if(clear | stop)
         control <= control & 8'hfc;  //auto clean low bits
    end // else: !if(ares)
  end // always @ (posedge clk8m or posedge ares)

// control decode
assign clear = control[0];
assign stop = control[1];
assign quiet = control[2];

// counters
 target_counter north(
   .clk(clk8m),
   .start(mic_north),
   .run(run_north),
   .count(count_north),
   .*
  );
 target_counter south(
   .clk(clk8m),
   .start(mic_south),
   .run(run_south),
   .count(count_south),
   .*
  );
 target_counter east(
   .clk(clk8m),
   .start(mic_east),
   .run(run_east),
   .count(count_east),
   .*
  );
 target_counter west(
   .clk(clk8m),
   .start(mic_west),
   .run(run_west),
   .count(count_west),
   .*
  );

/*----------------------------------------------------------------------
 instances of the adc's for now  no signal processing
----------------------------------------------------------------------*/
 sd_adc #(.WIDTH(8), .ACC_WIDTH(10), .LPF_DEPTH(3)) adc0 (
  .clk(clk64M),
  .ares(ares),
  .comp(sd0p),
//  .comp(comp0),
  .sdm(pcm0),
  .q(conv0),
  .wr() );
/*
sd_adc #(.WIDTH(8), .ACC_WIDTH(10), .LPF_DEPTH(3)) adc1 (
  .clk(clk64M),
  .ares(ares),
  .comp(sd1p),
//  .comp(comp1),
  .sdm(pcm1),
  .q(conv1),
  .wr() );
sd_adc #(.WIDTH(8), .ACC_WIDTH(10), .LPF_DEPTH(3)) adc2 (
  .clk(clk64M),
  .ares(ares),
  .comp(sd2p),
//  .comp(comp2),
  .sdm(pcm2),
  .q(conv2),
  .wr() );
sd_adc #(.WIDTH(8), .ACC_WIDTH(10), .LPF_DEPTH(3)) adc3 (
  .clk(clk64M),
  .ares(ares),
  .comp(sd3p),
//  .comp(comp3),
  .sdm(pcm3),
  .q(conv3),
  .wr() );
sd_adc #(.WIDTH(8), .ACC_WIDTH(10), .LPF_DEPTH(3)) adc4 (
  .clk(clk64M),
  .ares(ares),
  .comp(sd4p),
//  .comp(comp4),
  .sdm(pcm4),
  .q(conv4),
  .wr() );
sd_adc #(.WIDTH(8), .ACC_WIDTH(10), .LPF_DEPTH(3)) adc5 (
  .clk(clk64M),
  .ares(ares),
  .comp(sd5p),
//  .comp(comp5),
  .sdm(pcm5),
  .q(conv5),
  .wr() );
*/
endmodule // etarget
