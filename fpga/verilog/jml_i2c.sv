/*----------------------------------------------------------------------
According to 
http://forums.semiconductors.philips.com/viewtopic.php?t=3773&sid=5982e7876a7021e1278c6fbd46d98ff6

The assumption is that parts use the falling edge of the SCL to clock
their state machine and store the data or to output the updated data.

----------------------------------------------------------------------*/

module jml_i2c  #(
  parameter MYI2C_ADDR = 7'h10
 ) (
  input 	     reset_n,
// interface to pins
  input 	     scl, //i2c clock
  input 	     sda,
  output logic 	     sda_drv_lo,
//serial bridge signals
  output logic [5:0] addr,
  output logic 	     read,
  output logic 	     write,
  output logic [7:0] write_data,
  input [7:0] 	     read_data,
  output logic 	     rd_pop,
  output logic 	     trace_rd_reset
);
/*----------------------------------------------------------------------
  Declarations
----------------------------------------------------------------------*/
logic [0:7] read_data_rev;

//common

logic [7:0] write_data_nxt;
logic byte_done, byte_done_nxt;
logic [7:0] srdata, srdata_nxt;
logic [5:0] addr_nxt;
logic [2:0] bit_cnt, bit_cnt_nxt;
logic write_nxt;
logic bit_cnt7;
assign bit_cnt7 = (bit_cnt==7);
// unused logic bit_cnt_nxt7 = (bit_cnt_nxt==7);

localparam
          I2C_IDLE = 3'd0,
          I2C_START = 3'd7,
          I2C_ADDR = 3'd1,
          I2C_READ = 3'd2,
          I2C_READ_ACK = 3'd3,
          I2C_LOGIC_ADDR = 3'd4,
          I2C_WRITE = 3'd5;

logic i2c_resetS_n;
logic i2cS, i2cP;

logic [2:0] i2c_state, i2c_nxt;
logic my_addr;
logic got_ack, got_ack_nxt;
logic i2c_read;
assign i2c_resetS_n = reset_n && ~(i2c_state==I2C_ADDR);
logic first_write_done, first_write_done_nxt;

/*----------------------------------------------------------------------
  I2C behavior
  i2c related behavior
  a transfer starts with start
  a transfer sequence consists of an address transfer
  with a logic write 
   (S) (My ADDR) (W) - (8 bit logic address) - (8 bit write data) (P)
   * broken write transfers are not supported
   * multi (data) byte writes are supported if MBWRITE is defined,  each write is a 3+ byte transaction.
   unsupported (S) (My ADDR) (W) - (8 bit logic address)  (S) (My ADDR) (W) - (8 bit write data) (P)
  or a read
   (S) (My ADDR) (W) - (8 bit logic address)
  followed by a one or more reads with auto increment
   (S) (My ADDR) (R) - (8 bit data sent) [(8 bit data sent) ...](P)

 The toggling of state to get back to writting the address occurs when
  a stop followed by a start,
  a write has occured
  any write after read.
----------------------------------------------------------------------*/
always @(negedge sda or negedge i2c_resetS_n)
  if(!i2c_resetS_n)
    i2cS <=  1'b0;
  else
    if(scl) // start condition
      i2cS <=  1'b1;
    else
      i2cS <=  1'b0;

always @(posedge sda or negedge reset_n)
  if(!reset_n)
    i2cP <=  1'b0;
  else
    if(scl) // stop condition
      i2cP <=  1'b1;
    else
      i2cP <=  1'b0;

/*----------------------------------------------------------------------
 i2c state machine
----------------------------------------------------------------------*/
always @(negedge scl or negedge reset_n)
  if(!reset_n)
    begin
      i2c_state <=  I2C_IDLE;
      got_ack   <=  1'b0;
    end
  else
    begin
      i2c_state <=  i2c_nxt;
      got_ack   <=  got_ack_nxt;
    end

always @*
  begin
    my_addr = (srdata[7:1] == MYI2C_ADDR);
    case(i2c_state)
      I2C_IDLE:
        if (i2cS) 
          i2c_nxt = I2C_START;
        else 
          i2c_nxt = I2C_IDLE;

      I2C_START:
          i2c_nxt = I2C_ADDR;

      I2C_ADDR:
        if(!byte_done)
          i2c_nxt = I2C_ADDR;
        else
          if(my_addr)
            if(srdata[0]) // r/w_n
              i2c_nxt = I2C_READ;
            else
              i2c_nxt = I2C_LOGIC_ADDR;
          else // not my address
            i2c_nxt = I2C_IDLE;

      I2C_READ:
        if (i2cS) 
          i2c_nxt = I2C_START;
        else
        if(!byte_done)
          i2c_nxt = I2C_READ;
        else
          i2c_nxt = I2C_READ_ACK;

      I2C_READ_ACK:
        if(i2cS)
          i2c_nxt = I2C_START;
        else
          if(i2cP)
            i2c_nxt = I2C_IDLE;
          else
            i2c_nxt = I2C_READ;

      I2C_LOGIC_ADDR:
        if (i2cS) 
          i2c_nxt = I2C_START;
        else
        if(!byte_done)
          i2c_nxt = I2C_LOGIC_ADDR;
        else
          i2c_nxt = I2C_WRITE;

      I2C_WRITE:
        if (i2cS) 
          i2c_nxt = I2C_START;
        else
        if(!byte_done)
          i2c_nxt = I2C_WRITE;
        else
          i2c_nxt = I2C_WRITE; // TBD qlaify with multi bit
     default:
          i2c_nxt = I2C_IDLE;


    endcase

    // handle ack and data sending
    if( (((i2c_state == I2C_ADDR) && my_addr)
            || (i2c_state == I2C_LOGIC_ADDR)
            || (i2c_state == I2C_WRITE) )
         && bit_cnt7)
      sda_drv_lo = 1'b1; // send ack
    else
      if(((i2c_nxt == I2C_READ) || ((i2c_nxt == I2C_READ_ACK) && got_ack)) && (bit_cnt != 7))  // need to allow for ack in
        sda_drv_lo = ~read_data_rev[bit_cnt_nxt];
      else 
        sda_drv_lo = 1'b0;

    if((i2c_state == I2C_READ) && (bit_cnt7))
      got_ack_nxt = ~sda;
    else 
      got_ack_nxt = 1'b0;

  end
/*----------------------------------------------------------------------
 common shift clock domain clock generation, bit counter, shift logicister
  common address logicister
  loads with byte_done from I2C_LOGIC_ADDR
  Auto increments  I2C_READ && got_ack && !(S)top
  read and write generation

----------------------------------------------------------------------*/
// bit_cnt is on shift_clk (scl)
// sample (capture) clock
always @(negedge scl or negedge reset_n)
  if(!reset_n)
    begin
      addr        <=  6'b0;
      write_data  <=  8'b0;
      srdata      <=  8'b0;
      byte_done   <=  1'b0;
      write       <=  1'b0;
      bit_cnt     <=  3'b0;
      first_write_done <=  1'b0;
   end
  else 
    begin
      addr        <=  addr_nxt;
      write_data  <=  write_data_nxt;
      srdata      <=  srdata_nxt;
      byte_done   <=  byte_done_nxt;
      write       <=  write_nxt;
      bit_cnt     <=  bit_cnt_nxt;
      first_write_done <=  first_write_done_nxt;
    end

always @*
  begin
    if(
       ((i2c_state != I2C_IDLE) && bit_cnt !=7) 
      )
      srdata_nxt = {srdata[6:0], sda};
    else
      srdata_nxt = srdata;

    if( 
       ((i2c_state != I2C_IDLE) && bit_cnt != 7 && !byte_done & ! (i2c_state == I2C_START))
       )
      bit_cnt_nxt = bit_cnt + 1'b1;
    else 
      bit_cnt_nxt = 3'b0;

    if( 
         ((i2c_state != I2C_IDLE) && bit_cnt7)
       )
      byte_done_nxt = 1'b1;
    else 
      byte_done_nxt = 1'b0;

    if( 
        ((i2c_state==I2C_WRITE ) && byte_done_nxt)
      )
      write_nxt = 1'b1;
    else
      write_nxt = 1'b0;

//address load cases
    if( ((i2c_state==I2C_LOGIC_ADDR) && byte_done) )
	addr_nxt = srdata[5:0];
    else
//address increment cases
      if((
         ((i2c_state==I2C_READ) && got_ack_nxt && bit_cnt7 && !i2cS) //increment only on ack'd reads
        || (write_nxt && first_write_done))
	&& (addr[5:4] != 2'b11) // in this i2c addresses 3* will hold for reading the rams
     )
        addr_nxt = addr + 1'b1;
      else
        addr_nxt = addr;

    if(((i2c_state==I2C_LOGIC_ADDR) && byte_done) && (srdata[5:4] == 2'b11))
      trace_rd_reset = 1'b1;
    else
      trace_rd_reset = 1'b0;

    if((((i2c_state==I2C_READ) && got_ack_nxt && bit_cnt7 && !i2cS) //increment only on ack'd reads
        || (write_nxt && first_write_done))
	&& (addr[5:4] == 2'b11))
      rd_pop  = 1'b1;
    else
      rd_pop  = 1'b0;
      
// read and write are asyncronous here
    if( ((i2c_state==I2C_READ) || (i2c_nxt==I2C_READ) || (i2c_state==I2C_READ_ACK)) && !i2cP)
       i2c_read = 1'b1;
    else 
       i2c_read = 1'b0;
    if(
       i2c_read
      )
      read = 1'b1;
    else
      read = 1'b0;

    if( ((i2c_state==I2C_WRITE ) && byte_done_nxt) )
      write_data_nxt = srdata;
    else
      write_data_nxt = write_data;

    if( 
        (i2c_state==I2C_LOGIC_ADDR)
       ) // reset condition (either addr state)
      first_write_done_nxt = 1'b0;
    else
      if(write)
        first_write_done_nxt = 1'b1;
      else
        first_write_done_nxt = first_write_done;
  end
/*----------------------------------------------------------------------
  read data latch allows for  1/2 cycle of data time.
  I know I hate latches but is there a better soultion?

  latch is open when bit cnt is zero 
 ----------------------------------------------------------------------*/
/*
logic read_latch_open;
always @*
  begin
    if(bit_cnt==0)
      read_latch_open = 1'b1;
    else
      read_latch_open = 1'b0;
  end
always @(read_latch_open or read_data)
  if(read_latch_open)
    read_data_rev <=  read_data;
*/
always @(posedge scl or negedge reset_n)
  if(!reset_n)
    read_data_rev <= 8'h00;
  else
    if(bit_cnt==0) read_data_rev <= read_data;

endmodule // jml_i2c

