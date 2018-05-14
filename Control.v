/******************************************************************
* Description
*	This is control unit for the MIPS processor. The control unit is 
*	in charge of generation of the control signals. Its only input 
*	corresponds to opcode from the instruction.
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	01/03/2014
******************************************************************/
module Control
(
	input [5:0]OP,
	input [5:0] Funct,
	
	output Jal,
	output Jump,
	output reg Jr,
	output RegDst,
	output BranchEQ,
	output BranchNE,
	output MemRead,
	output MemtoReg,
	output MemWrite,
	output ALUSrc,
	output RegWrite,
	output [3:0]ALUOp
);

// Estos parámetros deben de tener el Opcode del greensheet en el mips
localparam R_Type = 0;
localparam I_Type_ADDI 	= 6'h08;
localparam I_Type_ORI 	= 6'h0d;
localparam I_Type_ANDI 	= 6'h0c;
localparam I_Type_LUI 	= 6'h0f;
localparam I_Type_LW 	= 6'h23;
localparam I_Type_SW 	= 6'h2b;
localparam I_Type_BEQ 	= 6'h04;
localparam I_Type_BNE 	= 6'h05;
localparam J_Type_JMP 	= 6'h02;
localparam J_Type_JAL 	= 6'h03;

//reg [10:0] ControlValues;
reg [14:0] ControlValues;

// Se expandieron los bits de ControlValues para agregar la bandera de jump y expandir ALUOP a 4 bits
// Se cambió el ALUOP por comodidad y comprensión
always@(OP) begin
	casex(OP)
												 //Banderas      ALUOP
		R_Type:       ControlValues= 14'b00_1_001_00_00_0111;
		I_Type_ADDI:  ControlValues= 14'b00_0_101_00_00_0000;
		I_Type_ORI:   ControlValues= 14'b00_0_101_00_00_0001;
		I_Type_ANDI:  ControlValues= 14'b00_0_101_00_00_0010;
		I_Type_LUI:   ControlValues= 14'b00_0_101_00_00_0011;
		I_Type_LW:	  ControlValues= 14'b00_0_111_10_00_0100;
		I_Type_SW:	  ControlValues= 14'b00_0_100_01_00_0101;
		I_Type_BEQ:	  ControlValues= 14'b00_0_000_00_01_1000;
		I_Type_BNE:	  ControlValues= 14'b00_0_000_00_10_1001;
		J_Type_JMP:	  ControlValues= 14'b01_0_000_00_00_0000; // No necesita hacer algo en ALU	
		J_Type_JAL:	  ControlValues= 14'b11_0_001_00_00_0000; // Activa RegWrite porque vas a guardar en $ra, activa bandera jmp para dejar pasar el jmp address

		
		default:
			ControlValues= 14'b00000000000000;
	endcase
	
	Jr <= (Funct == 6'b001000 && OP == 6'b000000) ? 1:0;
end	

assign Jal = ControlValues[13];
assign Jump = ControlValues[12];	
assign RegDst = ControlValues[11];
assign ALUSrc = ControlValues[10];
assign MemtoReg = ControlValues[9];
assign RegWrite = ControlValues[8];
assign MemRead = ControlValues[7];
assign MemWrite = ControlValues[6];
assign BranchNE = ControlValues[5];
assign BranchEQ = ControlValues[4];
assign ALUOp = ControlValues[3:0];	

endmodule


