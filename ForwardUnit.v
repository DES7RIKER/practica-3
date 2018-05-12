module ForwardUnit
(
	input EX_MEM_RegWrite,
	input [4:0] EX_MEM_WriteRegister,
	input [4:0] ID_EX_RegisterRs,
	input [4:0] ID_EX_RegisterRt,
	input MEM_WB_RegWrite,
	input [4:0] MEM_WB_WriteRegister,

	output reg [1:0] ForwardA,
	output reg [1:0] ForwardB
	
);

always@(*) begin // Toma todas las entradas como lista de sensibilidad
	// EX Forward Unit
	if(EX_MEM_RegWrite == 1 && EX_MEM_WriteRegister != 0 && EX_MEM_WriteRegister == ID_EX_RegisterRs)
		ForwardA <= 2'b10;
		
	if(EX_MEM_RegWrite == 1 && EX_MEM_WriteRegister != 0 && EX_MEM_WriteRegister == ID_EX_RegisterRt)
		ForwardB <= 2'b10;
		
	// MEM Forward Unit
	if(MEM_WB_RegWrite == 1 && MEM_WB_WriteRegister != 0 && MEM_WB_WriteRegister == ID_EX_RegisterRs && EX_MEM_WriteRegister != ID_EX_RegisterRs)
		ForwardA <= 2'b01;
		
	if(MEM_WB_RegWrite == 1 && MEM_WB_WriteRegister != 0 && MEM_WB_WriteRegister == ID_EX_RegisterRt && EX_MEM_WriteRegister != ID_EX_RegisterRt)
		ForwardB <= 2'b01;
	

end

endmodule
