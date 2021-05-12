module etarget(
  input        clk64M,
  input        reset_n,
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
  input        DIPA,
  input        DIPB,
  input        DIPC,
  input        DIPD,
        
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
 b dip d, dip c, dip b, dip a
 c slope control [7] = slope mode [6] = negative slope [5:0] = slope
----------------------------------------------------------------------*/

logic [7:0] version;
assign version = 8'h10;
logic [7:0] control;
logic 	    clear;
logic 	    stop;
logic       quiet;
logic 	    start;
logic [7:0] slope_control;
logic       slope_mode;
logic       slope_neg;
logic [5:0] slope;
logic [7:0] rec_sel;
logic [7:0] rec_ctl;
logic [7:0] trigger_depth;

logic [15:0] count_north;
logic [15:0] count_south;
logic [15:0] count_east;
logic [15:0] count_west;
logic [5:0]  addr;
logic [7:0]  read_data;
logic [7:0]  write_data;
logic [2:0]  conv0, conv1, conv2, conv3, conv4, conv5;
wire         det0, det1, det2, det3, det4;

logic 	     read;
logic 	     write;
logic 	     cnt_clk;
wire 	     pin_north, pin_south, pin_east, pin_west;
wire 	     hold_north, hold_south, hold_east, hold_west;
logic 	     latch_north, latch_south, latch_east, latch_west;
wire 	     run_north, run_south, run_east, run_west;
logic	     start_north, start_south, start_east, start_west;
logic 	     rd_pop;
logic 	     trace_rd_reset;
logic [7:0]  trace_data;


//I2c slave port
jml_i2c #(.MYI2C_ADDR('h10)) i2c(
  .*
  );
// clock divider
// 64 MHZ will be used for sd adc
// 8 mhz for direction counter
logic [2:0]  clkdiv;
logic 	     clk8M;
logic 	     div, div_nxt;

always @(posedge clk64M or negedge reset_n)
  if(~reset_n)
    begin
      clkdiv <= 0;
      div <= 0;
    end
  else
    begin
      clkdiv <= clkdiv + 1'b1;
      clk8M <= div_nxt;
      div <= div_nxt;
    end
assign div_nxt = &clkdiv;


// read mux
always @*
  begin
    casez (addr)
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
      6'hc : read_data = slope_control;
      6'hd : read_data = rec_sel;
      6'he : read_data = rec_ctl;
      6'hf : read_data = trigger_depth;      
      6'h10: read_data = conv0;
      6'h11: read_data = conv1;
      6'h12: read_data = conv2;
      6'h13: read_data = conv3;
      6'h14: read_data = conv4;
      6'h15: read_data = conv5;
      6'h3?: read_data = trace_data;
      default: read_data = 8'h00;
    endcase // case (addr)
  end // always @ *
//write data
always @(posedge clk8M or negedge reset_n)
  begin
    if(!reset_n)
      begin
	control <= 8'b0;
	slope_control <= 8'b0;
	rec_sel <= 8'b0;
	rec_ctl <= 8'b0;
	trigger_depth <= 8'b0;
      end
    else
      begin
	if(write) begin
	  if(addr == 6'h1) control <= write_data;
	  if(addr == 6'hc) slope_control <= write_data;
	  if(addr == 6'hd) rec_sel <= write_data;
	  if(addr == 6'he) rec_ctl <= write_data;
	  if(addr == 6'hf) trigger_depth <= write_data;
      end
      if(clear | stop | start)
         control <= control & 8'hf4;  //auto clear 'clear', 'stop' and 'start' bits
      end // else: !if(!reset_n)
  end // always @ (posedge clk8M or negedge reset_n)


// control decode
assign clear = control[0];
assign stop = control[1];
assign quiet = control[2];
assign start = control[3];
assign slope_mode = slope_control[7];
assign slope_neg  = slope_control[6];
assign slope = slope_control[5:0];

// start control muxing
always_comb
  begin
    if(slope_mode)
      begin
	start_north = det0 | start;
	start_east  = det2 | start;
	start_south = det1 | start;
	start_west  = det3 | start;
      end
    else
      begin
	start_north = latch_north | start;
	start_east  = latch_east  | start;
	start_south = latch_south | start;
	start_west  = latch_west  | start;
      end // else: !if(slope_mode)
  end // always_comb

      
// counters
 target_counter north(
   .clk(clk8M),
   .start(start_north),
   .run(run_north),
   .count(count_north),
   .*
  );
 target_counter east(
   .clk(clk8M),
   .start(start_east),
   .run(run_east),
   .count(count_east),
   .*
  );
 target_counter south(
   .clk(clk8M),
   .start(start_south),
   .run(run_south),
   .count(count_south),
   .*
  );
 target_counter west(
   .clk(clk8M),
   .start(start_west),
   .run(run_west),
   .count(count_west),
   .*
  );

/*----------------------------------------------------------------------
 instances of the slope detectors
----------------------------------------------------------------------*/
slope_det sd0(.comp(comp0), .sdm(pcm0), .conv(conv0), .det(det0), .*);
slope_det sd1(.comp(comp1), .sdm(pcm1), .conv(conv1), .det(det1), .*);
slope_det sd2(.comp(comp2), .sdm(pcm2), .conv(conv2), .det(det2), .*);
slope_det sd3(.comp(comp3), .sdm(pcm3), .conv(conv3), .det(det3), .*);
slope_det sd4(.comp(comp4), .sdm(pcm4), .conv(conv4), .det(det4), .*);
slope_det sd5(.comp(comp5), .sdm(pcm5), .conv(conv5), .det(det5), .*);
data_recorder data_recorder(
 .det({det5,det4,det3,det2,det1,det0}),
 .*
  );

/* input cells to use the latch the mic comparitors
 IO types 
    6'b000000 -- registered inpput
    6'b000011 -- combinational latch closed with latch input value 
    6'b000010 -- registered input value and held with latch input value
 */
assign hold_north = latch_north & ~run_north;
always @(posedge clk64M or negedge reset_n)
  if(!reset_n)
    begin
      latch_north <= 1'b0;
      latch_east  <= 1'b0;
      latch_south <= 1'b0;
      latch_west  <= 1'b0;
    end
  else
    begin
      if(run_north)
	latch_north <= 1'b0;
      else if(pin_north)
	latch_north <= 1'b1;
      else
	latch_north <= latch_north; // hold

      if(run_east)
	latch_east <= 1'b0;
      else if(pin_east)
	latch_east <= 1'b1;
      else
	latch_east <= latch_east; // hold

      if(run_south)
	latch_south <= 1'b0;
      else if(pin_south)
	latch_south <= 1'b1;
      else
	latch_south <= latch_south; // hold

      if(run_west)
	latch_west <= 1'b0;
      else if(pin_west)
	latch_west <= 1'b1;
      else
	latch_west <= latch_west; // hold
    end // else: !if(!reset_n)


SB_IO #(
    .PIN_TYPE(6'b000000), // registered
    .PULLUP(1'b0),
    .IO_STANDARD("SB_LVCMOS")
) north_in (
  .PACKAGE_PIN(mic_north),
  .LATCH_INPUT_VALUE (),
  .CLOCK_ENABLE ( ),
  .INPUT_CLK (clk64M),
  .OUTPUT_CLK ( ),
  .OUTPUT_ENABLE ( ),
  .D_OUT_0 ( ),
  .D_OUT_1 ( ),
  .D_IN_0 (pin_north),
  .D_IN_1 ()
);

assign hold_east = latch_east & ~run_east;
SB_IO #(
    .PIN_TYPE(6'b000000),
    .PULLUP(1'b0),
    .IO_STANDARD("SB_LVCMOS")
) east_in (
  .PACKAGE_PIN(mic_east),
  .LATCH_INPUT_VALUE (),
  .CLOCK_ENABLE ( ),
  .INPUT_CLK (clk64M),
  .OUTPUT_CLK ( ),
  .OUTPUT_ENABLE ( ),
  .D_OUT_0 ( ),
  .D_OUT_1 ( ),
  .D_IN_0 (pin_east),
  .D_IN_1 ()
);

assign hold_south = latch_south & ~run_south;
SB_IO #(
    .PIN_TYPE(6'b000000),
    .PULLUP(1'b0),
    .IO_STANDARD("SB_LVCMOS")
) south_in (
  .PACKAGE_PIN(mic_south),
  .LATCH_INPUT_VALUE (),
  .CLOCK_ENABLE ( ),
  .INPUT_CLK (clk64M),
  .OUTPUT_CLK ( ),
  .OUTPUT_ENABLE ( ),
  .D_OUT_0 ( ),
  .D_OUT_1 ( ),
  .D_IN_0 (pin_south),
  .D_IN_1 ()
);

assign hold_west = latch_west & ~run_west;
SB_IO #(
    .PIN_TYPE(6'b000000),
    .PULLUP(1'b0),
    .IO_STANDARD("SB_LVCMOS")
) west_in (
  .PACKAGE_PIN(mic_west),
  .LATCH_INPUT_VALUE (),
  .CLOCK_ENABLE ( ),
  .INPUT_CLK (clk64M),
  .OUTPUT_CLK ( ),
  .OUTPUT_ENABLE ( ),
  .D_OUT_0 ( ),
  .D_OUT_1 ( ),
  .D_IN_0 (pin_west),
  .D_IN_1 ()
);

endmodule // etarget
