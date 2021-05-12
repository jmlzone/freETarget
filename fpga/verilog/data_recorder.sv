module data_recorder (
  input 	     clk8M, // filters and down stream logic at 8MHz. Known phase relationship
  input 	     reset_n,
  input [5:0] 	     det,
  input [2:0] 	     conv0,
  input [2:0] 	     conv1,
  input [2:0] 	     conv2,
  input [2:0] 	     conv3,
  input [2:0] 	     conv4,
  input [2:0] 	     conv5,
  input [7:0] 	     rec_sel,
  input [7:0] 	     rec_ctl,
  input [7:0] 	     trigger_depth,
  input 	     scl,
  input [5:0] 	     addr, 
  input 	     trace_rd_reset,
  input 	     rd_pop,
  output logic [7:0] trace_data
  );


/* ----------------------------------------------------------------------
 rec_sel[1:0] ch0 selects 00 = 0, 01 = 1, 10 = 2, 11 = 3 
 rec_sel[3:2] ch1 selects 00 = 0, 01 = 1, 10 = 2, 11 = 3 
 rec_sel[5:4] ch2 selects 00 = 2, 01 = 3, 10 = 4, 11 = 5 
 rec_sel[7:6] ch3 selects 00 = 2, 01 = 3, 10 = 4, 11 = 5
 
 rec_ctl [5:0] trigger source mask
 rec_ctl [6] auto re-trigger
 rec_ctl [7] enable
 
----------------------------------------------------------------------*/
logic [9:0]   wr_addr, rd_addr, wr_addr_nxt, rd_addr_nxt;
logic [9:0]   trigger_point, trigger_point_nxt;

logic 	      triggered;
logic 	      stop;

logic [3:0]   wdata0, wdata1, wdata2, wdata3;
localparam IDLE = 2'b00;
localparam PRE_TRIG = 2'b01;
localparam TRIGGERED = 2'b11;
localparam STOP = 2'b10;
logic [1:0]   state, state_nxt;
logic 	      we;
logic [9:0]   stop_point;

// Write side control on clk8M
assign stop_point = trigger_point - {trigger_depth,2'b00} -1;

always_comb
  begin
    case(state)
      IDLE: if(rec_ctl[7])
	state_nxt = PRE_TRIG;
      else
	state_nxt = IDLE;
      PRE_TRIG: if(|(rec_ctl[5:0] & det))
	state_nxt = TRIGGERED;
      else
	state_nxt = PRE_TRIG;
      TRIGGERED:
	if(wr_addr == stop_point)
	  state_nxt = STOP;
	else
	  state_nxt = TRIGGERED;
      STOP:
	if((!rec_ctl[7]) || (rec_ctl[6] && 1'b0)) // retrigger condition TBD
	  state_nxt = IDLE;
	else
	  state_nxt = STOP;
    endcase // case (state)
    if((state==PRE_TRIG) || (state==TRIGGERED))
      begin
	wr_addr_nxt = wr_addr + 1;
	we = 1;
      end
    else
      begin
	wr_addr_nxt = wr_addr;			
	we = 0;
      end
    if((state==PRE_TRIG) || (state_nxt==TRIGGERED))
      trigger_point_nxt = wr_addr;
    else
      trigger_point_nxt = trigger_point;
  end

always_ff @(posedge clk8M or negedge reset_n)
  begin
    if(!reset_n)
      begin
	wr_addr <= 0;
	trigger_point <= 0;
	wdata0 <= 0;
	wdata1 <= 0;
	wdata2 <= 0;
	wdata3 <= 0;
	state <= IDLE;
      end
    else
      begin
	state <= state_nxt;
	wr_addr <= wr_addr_nxt;
	trigger_point <= trigger_point_nxt;	
	case(rec_sel[1:0])
	  2'b00: wdata0 <= {1'b0, conv0};
	  2'b01: wdata0 <= {1'b0, conv1};
	  2'b10: wdata0 <= {1'b0, conv2};
	  default: wdata0 <= {1'b0, conv3};
	endcase // case (rec_sel[1:0])
	case(rec_sel[3:2])
	  2'b00: wdata1 <= {1'b0, conv0};
	  2'b01: wdata1 <= {1'b0, conv1};
	  2'b10: wdata1 <= {1'b0, conv2};
	  default: wdata1 <= {1'b0, conv3};
	endcase // case (rec_sel[3:2])
	case(rec_sel[5:4])
	  2'b00: wdata2 <= {1'b0, conv2};
	  2'b01: wdata2 <= {1'b0, conv3};
	  2'b10: wdata2 <= {1'b0, conv4};
	  default: wdata2 <= {1'b0, conv5};
	endcase // case (rec_sel[5:4])
	case(rec_sel[7:6])
	  2'b00: wdata3 <= {1'b0, conv2};
	  2'b01: wdata3 <= {1'b0, conv3};
	  2'b10: wdata3 <= {1'b0, conv4};
	  default: wdata3 <= {1'b0, conv5};
	endcase // case (rec_sel[7:6])
      end // else: !if(!reset_n)
  end // always_ff @ (posedge clk8M or negedge reset_n)
// Read side on SCL falling edge
always @(negedge scl or negedge reset_n)
  if(!reset_n)
    begin
      rd_addr <= 0;
    end
  else
    begin
      if(trace_rd_reset)
	rd_addr <= stop_point+1;
      else if(rd_pop)
	rd_addr <= rd_addr +1;
    end

//rams instances
wire [3:0] rdata0, rdata1, rdata2, rdata3;
logic 	   read01, read23;

logic rd,trace_rd_reset_q;
always @(posedge scl)
  trace_rd_reset_q <=trace_rd_reset;

always_comb
  begin
    read01 = (addr==6'h30) | trace_rd_reset_q ;
    read23 = (addr==6'h31) | trace_rd_reset_q ;
    if(read01)
      trace_data = {rdata1,rdata0};
    else if(read23)
      trace_data = {rdata3,rdata2};
    else
      trace_data = 8'h00; 
  end
 
assign rd = rd_pop | trace_rd_reset_q;

SB_RAM1024x4NR ds0(
  .RDATA(rdata0),
  .RADDR(rd_addr),
  .RCLKN(scl),
  .RCLKE(rd),
  .RE(read01),
  .WADDR(wr_addr),
  .WCLK(clk8M),
  .WCLKE(we),
  .WDATA(wdata0),
  .WE(we)
  );
SB_RAM1024x4NR ds1(
  .RDATA(rdata1),
  .RADDR(rd_addr),
  .RCLKN(scl),
  .RCLKE(rd),
  .RE(read01),
  .WADDR(wr_addr),
  .WCLK(clk8M),
  .WCLKE(we),
  .WDATA(wdata1),
  .WE(we)
  );
SB_RAM1024x4NR ds2(
  .RDATA(rdata2),
  .RADDR(rd_addr),
  .RCLKN(scl),
  .RCLKE(rd),
  .RE(read23),
  .WADDR(wr_addr),
  .WCLK(clk8M),
  .WCLKE(we),
  .WDATA(wdata2),
  .WE(we)
  );
SB_RAM1024x4NR ds3(
  .RDATA(rdata3),
  .RADDR(rd_addr),
  .RCLKN(scl),
  .RCLKE(rd),
  .RE(read23),
  .WADDR(wr_addr),
  .WCLK(clk8M),
  .WCLKE(we),
  .WDATA(wdata3),
  .WE(we)
  );
endmodule // data_recorder

// wrapper for the native ram
module SB_RAM1024x4NR(
  	output [3:0] RDATA,
	input         RCLKN, RCLKE, RE,
	input  [9:0] RADDR,
	input         WCLK, WCLKE, WE,
	input  [9:0] WADDR,
	input  [3:0] WDATA
  );
logic [15:0] 	     rdata_int;

	SB_RAM40_4K #(
		.WRITE_MODE(2),
		.READ_MODE (2)
	) RAM (
		.RDATA(rdata_int),
		.RCLK (~RCLKN),
		.RCLKE(RCLKE),
		.RE   (RE   ),
		.RADDR({1'b0,RADDR}),
		.WCLK (WCLK ),
		.WCLKE(WCLKE),
		.WE   (WE   ),
		.WADDR({1'b0,WADDR}),
		.MASK (16'hffff),
		.WDATA({2'b0,WDATA[3],3'b0,WDATA[2],3'b0,WDATA[1],3'b0,WDATA[0],1'b0})
	);
assign RDATA = {rdata_int[13],rdata_int[9],rdata_int[5],rdata_int[1]};


endmodule
