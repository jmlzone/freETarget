module target_counter(
  input 	      clk,
  input 	      ares,
  input 	      start,
  input 	      stop,
  input 	      clear,
  input 	      quiet,
  output logic run,
  output logic [15:0] count
  );
logic clr;
assign clr = ares | clear;

// sr flop for mike trigger (converted to d)
 always @(posedge clk or posedge clr)
   if(clr)
     run <= 1'b0;
   else 
     if (~start) // active low
       run <= 1'b1;
     else if(~stop) // active low
       run <= 1'b0;
       
always @(posedge clk or posedge clr)
  if(clr)
    count <= 16'b0;
  else
    if(quiet & run & count < 16'hffff)
      count <= count +1;
endmodule // target_counter

