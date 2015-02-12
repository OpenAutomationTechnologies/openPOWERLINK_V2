module heart_beat(
	clk,
	led
);

parameter DUR_BITS = 26;

input  clk;
output led;


///////////////////////////////


reg [(DUR_BITS-1):0] cnt;
always @ (posedge clk)
begin
	cnt <= cnt + 1;
end

assign led = cnt[DUR_BITS-1];



endmodule
