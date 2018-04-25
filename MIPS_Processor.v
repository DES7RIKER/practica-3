/******************************************************************
* Description
*	This is the top-level of a MIPS processor that can execute the next set of instructions:
*		add
*		addi
*		sub
*		ori
*		or
*		bne
*		beq
*		and
*		nor
* This processor is written Verilog-HDL. Also, it is synthesizable into hardware.
* Parameter MEMORY_DEPTH configures the program memory to allocate the program to
* be execute. If the size of the program changes, thus, MEMORY_DEPTH must change.
* This processor was made for computer organization class at ITESO.
* Version:
*	1.0
* Author:
*	Dr. José Luis Pizano Escalante
* email:
*	luispizano@iteso.mx
* Date:
*	12/06/2016
******************************************************************/


module MIPS_Processor
#(
	parameter MEMORY_DEPTH = 71,//32
	parameter DATA_WIDTH = 32,
	parameter RAM_DEPTH = 256
)

(
	// Inputs
	input clk,
	input reset,
	input [7:0] PortIn,
	// Output
	output [31:0] ALUResultOut,
	output [31:0] PortOut
);
//******************************************************************/
//******************************************************************/
assign  PortOut = 0;

//******************************************************************/
//******************************************************************/
// Data types to connect modules
wire BranchNE_wire;
wire BranchEQ_wire;
wire RegDst_wire;
wire NotZeroANDBrachNE;
wire ZeroANDBrachEQ;
wire ORForBranch;
wire ALUSrc_wire;
wire RegWrite_wire;
wire Zero_wire;
wire MemRead_wire;			// Habilitadores RAM
wire MemWrite_wire;			// Habilitadores RAM
wire MemtoReg_wire;			// Bandera para multiplexor de ALU-RAM
wire [3:0] ALUOp_wire;
wire [3:0] ALUOperation_wire;
wire [4:0] WriteRegister_wire;
wire [4:0] WriteRegisternojal_wire;
wire [31:0] MUX_PC_wire;
wire [31:0] PC_wire;
wire [31:0] RAM_DataOut;
wire [31:0] WriteData_wire;
wire [31:0] WriteDatanojal_wire;			
wire [31:0] Instruction_wire;
wire [31:0] ReadData1_wire;
wire [31:0] ReadData2_wire;
wire [31:0] InmmediateExtend_wire;
wire [31:0] ReadData2OrInmmediate_wire;
wire [31:0] ALUResult_wire;
wire [31:0] PC_4_wire;
wire [31:0] PC_8_wire;
wire [31:0] InmmediateExtendAnded_wire;
wire [31:0] PCtoBranch_wire;
wire [31:0] branchAddress_wire;
wire [31:0] jumpAddress_wire;
wire [31:0] newPC_wire;
wire [31:0] pcWithBranch_wire;
wire branchA_wire;
wire branchB_wire;
wire branch_wire;
wire jump_wire;
wire jal_wire;
wire secureRegWrite_wire;
wire jr_wire;
integer ALUStatus;

wire [31:0] ID_Instruction_wire;
wire [31:0] ID_PC_4_wire;

PipeRegister
#(
	.N(64)
)
Register
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	.DataInput({Instruction_wire, PC_4_wire}),
	
	.DataOutput({ID_Instruction_wire, ID_PC_4_wire})
);

//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
Control
ControlUnit
(
	.OP(ID_Instruction_wire[31:26]),
	.RegDst(RegDst_wire),
	.BranchNE(BranchNE_wire),
	.BranchEQ(BranchEQ_wire),
	.ALUOp(ALUOp_wire),			//Funct
	.ALUSrc(ALUSrc_wire),
	.RegWrite(RegWrite_wire),
	.MemRead(MemRead_wire),
	.MemWrite(MemWrite_wire),
	.MemtoReg(MemtoReg_wire),
	.Jump(jump_wire),
	.Jal(jal_wire)
);

PC_Register
#(
	.N(32)
)
program_counter
(
	.clk(clk),
	.reset(reset),
	.NewPC({10'b0000000001, newPC_wire[21:0]}), // Completar direcciones como en MARS
	.PCValue(PC_wire)
);

ProgramMemory
#(
	.MEMORY_DEPTH(MEMORY_DEPTH)
)
ROMProgramMemory
(
	.Address({10'b0000000000, PC_wire[21:0]}),
	.Instruction(Instruction_wire)
);

DataMemory
#(
	.DATA_WIDTH(DATA_WIDTH),
	.MEMORY_DEPTH(RAM_DEPTH)
)
RAMDataMemory
(
	.WriteData(ReadData2_wire),
	.Address(ALUResult_wire[9:2]),
	.MemWrite(MemWrite_wire),
	.MemRead(MemRead_wire), 
	.clk(clk),
	.ReadData(RAM_DataOut)
);

Adder32bits
PC_Plus_4
(
	.Data0(PC_wire),
	.Data1(4),
	
	.Result(PC_4_wire)
);

/*Adder32bits
PC_Plus_8
(
	.Data0(PC_4_wire),
	.Data1(4),
	
	.Result(PC_8_wire)
);
*/



//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
Multiplexer2to1
#(
	.NBits(5)
)
MUX_ForRTypeAndIType
(
	.Selector(RegDst_wire),
	.MUX_Data0(ID_Instruction_wire[20:16]),
	.MUX_Data1(ID_Instruction_wire[15:11]),
	
	.MUX_Output(WriteRegisternojal_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ToRegFromRAMALU
(
	.Selector(MemtoReg_wire),
	.MUX_Data0(ALUResult_wire),
	.MUX_Data1(RAM_DataOut),
	
	.MUX_Output(WriteDatanojal_wire)

);



Multiplexer2to1
#(
	.NBits(5)
)
MUX_Write_addRa
(
	.Selector(jal_wire),
	.MUX_Data0(WriteRegisternojal_wire),
	.MUX_Data1(5'b11111), //31 de $ra
	
	.MUX_Output(WriteRegister_wire)

);




Multiplexer2to1
#(
	.NBits(32)
)
MUX_Write_dataRa
(
	.Selector(jal_wire),
	.MUX_Data0(WriteDatanojal_wire), //lo usa el memtoreg
	//.MUX_Data1(PC_8_wire), //pc8
	.MUX_Data1(ID_PC_4_wire),
	.MUX_Output(WriteData_wire)

);






Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForBranchAddress
(
	.Selector(branch_wire),
	.MUX_Data0(PC_4_wire),
	.MUX_Data1(pcWithBranch_wire), 
	
	.MUX_Output(branchAddress_wire)

);

Adder32bits
AdderPCplus4AndBranchAddress
(
	.Data0({InmmediateExtend_wire[29:0], 2'b00}),	// Branch Address, despreciamos los bits más significativos porque se va a hacer un shifteo 2 bits a la izquierda
	.Data1(PC_4_wire),
	
	.Result(pcWithBranch_wire)
);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJumpAddress
(
	.Selector(jump_wire),
	.MUX_Data0(branchAddress_wire),
	.MUX_Data1({PC_4_wire[31:28], ID_Instruction_wire[25:0], 2'b00}), // Jump Address
	
	.MUX_Output(jumpAddress_wire)

);


RegisterFile
Register_File
(
	.clk(clk),
	.reset(reset),
	.RegWrite(secureRegWrite_wire),
	.WriteRegister(WriteRegister_wire),
	.ReadRegister1(ID_Instruction_wire[25:21]),
	.ReadRegister2(ID_Instruction_wire[20:16]),
	.WriteData(WriteData_wire),
	.ReadData1(ReadData1_wire),
	.ReadData2(ReadData2_wire)

);

SignExtend
SignExtendForConstants
(   
	.DataInput(ID_Instruction_wire[15:0]),
   .SignExtendOutput(InmmediateExtend_wire)
);



Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForReadDataAndInmediate
(
	.Selector(ALUSrc_wire),
	.MUX_Data0(ReadData2_wire),
	.MUX_Data1(InmmediateExtend_wire),
	
	.MUX_Output(ReadData2OrInmmediate_wire)

);


ALUControl
ArithmeticLogicUnitControl
(
	.ALUOp(ALUOp_wire),
	.ALUFunction(ID_Instruction_wire[5:0]),
	.ALUOperation(ALUOperation_wire),
	.Jr(jr_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForJrJump
(
	.Selector(jr_wire),
	.MUX_Data0(jumpAddress_wire),
	.MUX_Data1(ReadData1_wire),
	
	.MUX_Output(newPC_wire)

);


ALU
ArithmeticLogicUnit 
(
	.ALUOperation(ALUOperation_wire),
	.A(ReadData1_wire),
	.B(ReadData2OrInmmediate_wire),
	.shamt(ID_Instruction_wire[10:6]),
	.Zero(Zero_wire),
	.ALUResult(ALUResult_wire)
);

assign ALUResultOut = ALUResult_wire;


assign branchA_wire = BranchNE_wire && ~Zero_wire;
assign branchB_wire = BranchEQ_wire && Zero_wire;
assign branch_wire  = branchA_wire || branchB_wire;
assign secureRegWrite_wire = RegWrite_wire && ~jr_wire;

endmodule

