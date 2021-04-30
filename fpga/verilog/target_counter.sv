module target_counter(
  input 	      clk,
  input 	      reset_n,
  input 	      start,
  input 	      stop,
  input 	      clear,
  input 	      quiet,
  output logic run,
  output logic [15:0] count
  );
always @(posedge clk or negedge reset_n)
  if(!reset_n) // hardware reset
    begin
      run <= 1'b0;
      count <= 16'b0;
    end
  else
  if(clear) // synchronous clear from i2c reg
    begin
      run <= 1'b0;
      count <= 16'b0;
    end
  else
    begin
     if (start)
       run <= 1'b1;
     else if(stop)
       run <= 1'b0;
    if(~quiet & run & count < 16'hffff)
      count <= count +1;
    end // else: !if(clear)

endmodule // target_counter

