/******************************************************************
* Description
*	This is an 32-bit arithetic logic unit that can execute the next set of operations:
*		add
*		sub
*		or
*		and
*		nor
* This ALU is written by using behavioral description.
* Version:
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	01/03/2014
******************************************************************/

module ALU 
(
	input [3:0] ALUOperation,
	input [31:0] A,
	input [31:0] B,
	input [4:0] shamt,			// Para el recorrimiento de bits
	output reg [31:0]ALUResult
);

localparam ADD = 4'b0000;
localparam SUB = 4'b0001;
localparam OR  = 4'b0010;
localparam AND = 4'b0011;
localparam NOR = 4'b0100;
localparam LUI = 4'b0101;
localparam SLL = 4'b0110;
localparam SRL = 4'b0111;
   
   always @ (A or B or ALUOperation or shamt)
     begin
		case (ALUOperation)
		  ADD: // add
			ALUResult=A + B;
		  SUB: // sub
			ALUResult=A - B;
		  OR:
			ALUResult=A | B;
		  AND:
			ALUResult=A & B;
		  NOR:
			ALUResult=~(A | B);
		  LUI:
			ALUResult= B << 16;
		  SLL:
			ALUResult= B << shamt;
		  SRL:
			ALUResult= B >> shamt;
		default:
			ALUResult= 0;
		endcase // case(control)
     end // always @ (A or B or control)
endmodule // ALU
