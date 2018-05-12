module HazardDetectionUnit
(
	input ID_EX_MemRead,
	input [4:0] ID_EX_WriteRegister,
	input [4:0] IF_ID_RegisterRs,
	input [4:0] IF_ID_RegisterRt,

	output reg EnablePC,
	output reg EnableRegister_IF_ID,
	output reg DisableControlSignals
	
);

always@(*) begin // Toma todas las entradas como lista de sensibilidad
	if(ID_EX_MemRead == 1 && ((ID_EX_WriteRegister == IF_ID_RegisterRs) || (ID_EX_WriteRegister == IF_ID_RegisterRt))) begin
		EnablePC <= 0;
		EnableRegister_IF_ID <= 0;
		DisableControlSignals <= 0;
	end
	else begin
		EnablePC <= 1;
		EnableRegister_IF_ID <= 1;
		DisableControlSignals <= 1;
	end


end

endmodule
