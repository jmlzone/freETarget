/*----------------------------------------------------------------------
 a simple I2c driver module for testing an i2c slave
----------------------------------------------------------------------*/
`timescale 1ns/1ns
module i2c_master (
  output logic scl,
  inout sda
  );
pullup(sda);
logic 	drv_sda;
initial  drv_sda = 1;

bufif0 sda_buf (sda,1'b0,drv_sda);

/* transfer buffer and config */
logic [6:0] dev_addr;
logic [7:0] reg_addr;
logic [7:0] data[0:31]; // read or write data for bursts
// timing paramaters
localparam DS = 100 ;
localparam SP = 100 ;
localparam ST = 100 ;
localparam CLKHI = 100;

task write(input int len);
int idx;
logic [6:0] cur_addr;
  begin
    send_byte({dev_addr,1'b0}, 1'b1, 1'b0); // start, i2c slave addr
    send_byte(reg_addr, 1'b0, 1'b0); // reg addr
    cur_addr = reg_addr;
    idx=0;
    while(idx < (len-1)) begin
      $display("sending burst write data[%0h] = %0h of max %0h\n", idx,data[idx], len);
      send_byte(data[idx], 1'b0, 1'b0); // data from 0 to data.size() -1
      cur_addr = cur_addr + 1;
      idx = idx + 1;
    end
    send_byte(data[idx], 1'b0, 1'b1); // last data, stop byte
    $display("sending last write data[%0h] = %0h of max %0h\n",idx, data[idx], len);
  end
endtask // write

task read(input int len);
int idx;
logic [6:0] cur_addr;
logic [7:0] rdata;
begin   
  send_byte({dev_addr,1'b0}, 1'b1, 1'b0); // start, I2C slave addr wr
  send_byte(reg_addr,1'b0, 1'b1); // send register addr, stop
  send_byte({dev_addr,1'b1}, 1'b1, 1'b0); // start, send i2c addr, read
  idx = 0;
  while(idx < (len-1)) begin
    read_byte(1'b0, 1, rdata); // read bytes
    data[idx] = rdata;
    cur_addr = cur_addr+1;
    idx=idx+1;
  end
  read_byte(1'b1, 0, rdata); // read last byte nack last byte
    data[idx] = rdata;
end
endtask // read

task read_byte(input logic send_stop, send_ack, output logic [7:0] read_data);
int i;
begin
  drv_sda = 1'b1; // high z since we read
  for(i=7; i>=0; i--) begin
    #DS ;
    scl = 1'b1;
    #CLKHI ;
    read_data[i] = sda;
    scl = 1'b0;
    #DS ;
  end
  // ACK
  if (send_ack) begin
    drv_sda = 0; // ack
  end else begin
    drv_sda = 1; // neg ack
  end
  #DS ;
  scl = 1'b1;
  #CLKHI ;
  scl = 1'b0;
  //#DS ;
  drv_sda = 1'b0;
  #DS ;   
  if (send_stop) begin
    scl = 1'b1;
    #SP ;
    drv_sda = 1'b1;
  end
  #DS ;
end
endtask // read_byte

task send_byte(input [7:0] data, input  send_start, send_stop);
int i;
  begin
    if (send_start) begin
      drv_sda = 1;
      #SP ;
      scl = 1;
      drv_sda = 1;
      #DS ;
      drv_sda =0;
      #ST ;
      scl = 0;
    end // if (send_start)
    for(i=7; i>=0; i--) begin
      #DS ;
      drv_sda =  data[i];
      #DS ;
      scl = 1'b1;
      #CLKHI ;
      scl = 1'b0;
    end
    // ACK
    #DS ;
    drv_sda = 1; // write
    #DS ;
    scl = 1'b1;
    #CLKHI ;
    scl = 1'b0;
    if (send_stop) begin
      if (drv_sda) begin
	drv_sda = 1'b0;
	#DS ;
      end
      #DS ;
      scl = 1'b1;
      #SP;
      drv_sda = 1'b1;
    end
    #DS ;
  end
endtask // send_byte

task display_read(input int len);
int idx;
begin
  for(idx=0; idx <len; idx++)
    $display(" read data [%2d], addr[%2d], got %2h" ,idx,(idx+reg_addr),data[idx]);
end
endtask // display_read


endmodule // i2cmaster
