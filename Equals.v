module Equals
(
	input [31:0] Register0,
	input [31:0] Register1,
	
	output reg Zero
);

always@(*) begin
	if(Register0 == Register1)
		Zero <= 1;
	else	
		Zero <= 0;
end

endmodule
