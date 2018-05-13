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
*************************** ***************************************/


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

wire EX_RegWrite_wire;
wire EX_RegDst_wire;
wire EX_jal_wire;
wire EX_ALUSrc_wire;
wire EX_MemtoReg_wire;
wire EX_BranchEQ_wire;
wire EX_BranchNE_wire;
wire EX_MemRead_wire;
wire EX_MemWrite_wire;
wire [31:0] EX_PC_4_wire;
wire [31:0] EX_ReadData1_wire;
wire [31:0] EX_ReadData2_wire;
wire [31:0] EX_InmmediateExtend_wire;
wire [4:0]	EX_RS;
wire [4:0]	EX_RT;
wire [4:0]	EX_RD;
wire [5:0]	EX_FUNCT;
wire [4:0]	EX_SHAMT;
wire [3:0]  EX_ALUOp_wire;

wire MEM_RegWrite_wire;
wire MEM_jal_wire;
wire MEM_MemtoReg_wire;
wire MEM_BranchEQ_wire;
wire MEM_BranchNE_wire;
wire MEM_MemRead_wire;
wire MEM_MemWrite_wire;
wire MEM_Zero_wire;
wire [31:0] MEM_pcWithBranch_wire;
wire [31:0] MEM_ALUResult_wire;
wire [31:0] MEM_ReadData2_wire;
wire [31:0] MEM_PC_4_wire;
wire [4:0]	MEM_WriteRegister_wire;

wire WB_RegWrite_wire;
wire WB_jal_wire;
wire WB_MemtoReg_wire;
wire [31:0] WB_ALUResult_wire;
wire [31:0] WB_RAM_DataOut;
wire [31:0] WB_PC_4_wire;
wire [4:0]	WB_WriteRegister_wire;

wire [1:0] forwardA_wire;
wire [31:0] multiplexerA1_wire;
wire [31:0] ALUOperand_A_wire;

wire [1:0] forwardB_wire;
wire [31:0] multiplexerB1_wire;
wire [31:0] ALUOperand_B_wire;

wire enablePC_wire;
wire enableRegister_IF_ID_wire;
wire disableControlSignals_wire;
wire [8:0] controlSignals_wire;

wire Zero_wire;
wire flush_wire;
wire jump_flags;

wire [4:0] WriteRegisterOrRa;

PipeRegister
#(
	.N(64)
)
IF_ID_Register
(
	.clk(clk),
	.reset(reset),
	.enable(enableRegister_IF_ID_wire),
	.flush(flush_wire),
	//.flush(0'b0),
	.DataInput({Instruction_wire, PC_4_wire}),
	
	.DataOutput({ID_Instruction_wire, ID_PC_4_wire})
);


PipeRegister
#(
	.N(167)
)
ID_EX_Register
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	//.flush(flush_wire),
	.flush(1'b0),
	.DataInput({controlSignals_wire,
					ID_PC_4_wire,
					ReadData1_wire,
					ReadData2_wire,
					InmmediateExtend_wire,
					ID_Instruction_wire[25:21],//RS
					ID_Instruction_wire[20:16],//RT
					ID_Instruction_wire[15:11],//RD
					ID_Instruction_wire[5:0],  //FUNCT
					ID_Instruction_wire[10:6], //SHAMT
					ALUOp_wire}),
	
	.DataOutput({EX_RegWrite_wire, 
					 EX_RegDst_wire, 
					 EX_jal_wire,
					 EX_ALUSrc_wire,
					 EX_MemtoReg_wire,
					 EX_BranchEQ_wire,
					 EX_BranchNE_wire,
					 EX_MemRead_wire,
					 EX_MemWrite_wire,
					 EX_PC_4_wire,
					 EX_ReadData1_wire,
					 EX_ReadData2_wire,
					 EX_InmmediateExtend_wire,
					 EX_RS,
					 EX_RT,
					 EX_RD,
					 EX_FUNCT,
					 EX_SHAMT,
					 EX_ALUOp_wire})
);

PipeRegister
#(
	.N(141)
)
EX_MEM_Register
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	//.flush(flush_wire),
	.flush(1'b0),
	.DataInput({EX_RegWrite_wire,  
					EX_jal_wire,
					EX_MemtoReg_wire,
					EX_BranchEQ_wire,
					EX_BranchNE_wire,
					EX_MemRead_wire,
					EX_MemWrite_wire,
					pcWithBranch_wire,
					Zero_wire,
					ALUResult_wire,
					EX_ReadData2_wire,
					EX_PC_4_wire,
					WriteRegister_wire}),
	
	.DataOutput({MEM_RegWrite_wire,  
					 MEM_jal_wire,
					 MEM_MemtoReg_wire,
					 MEM_BranchEQ_wire,
					 MEM_BranchNE_wire,
					 MEM_MemRead_wire,
					 MEM_MemWrite_wire,
					 MEM_pcWithBranch_wire,
					 MEM_Zero_wire,
					 MEM_ALUResult_wire,
					 MEM_ReadData2_wire,
					 MEM_PC_4_wire,
					 MEM_WriteRegister_wire})
);

PipeRegister
#(
	.N(104)
)
MEM_WB_Register
(
	.clk(clk),
	.reset(reset),
	.enable(1'b1),
	//.flush(flush_wire),
	.flush(1'b0),
	.DataInput({MEM_RegWrite_wire,  
					MEM_jal_wire,
					MEM_MemtoReg_wire,
					MEM_ALUResult_wire,
					MEM_PC_4_wire,
					RAM_DataOut,
					MEM_WriteRegister_wire}),
	
	.DataOutput({WB_RegWrite_wire,  
					 WB_jal_wire,
					 WB_MemtoReg_wire,
					 WB_ALUResult_wire,
					 WB_PC_4_wire,
					 WB_RAM_DataOut,
					 WB_WriteRegister_wire})
);

//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
ForwardUnit
ForwardUnit1
(
	.EX_MEM_RegWrite(MEM_RegWrite_wire),
	.EX_MEM_WriteRegister(MEM_WriteRegister_wire),
	.ID_EX_RegisterRs(EX_RS),
	.ID_EX_RegisterRt(EX_RT),
	.MEM_WB_RegWrite(WB_RegWrite_wire),
	.MEM_WB_WriteRegister(WB_WriteRegister_wire),

	.ForwardA(forwardA_wire),
	.ForwardB(forwardB_wire)
);

//A
Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForwardA1
(
	.Selector(forwardA_wire[1]),
	.MUX_Data0(EX_ReadData1_wire),
	.MUX_Data1(MEM_ALUResult_wire),
	
	.MUX_Output(multiplexerA1_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForwardA2
(
	.Selector(forwardA_wire[0]),
	.MUX_Data0(multiplexerA1_wire),
	.MUX_Data1(WriteData_wire),
	
	.MUX_Output(ALUOperand_A_wire)

);
// B
Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForwardB1
(
	.Selector(forwardB_wire[1]),
	.MUX_Data0(ReadData2OrInmmediate_wire), //Salida del multiplexor que elige registro o inmediato
	.MUX_Data1(MEM_ALUResult_wire),
	
	.MUX_Output(multiplexerB1_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ForwardB2
(
	.Selector(forwardB_wire[0]),
	.MUX_Data0(multiplexerB1_wire),
	.MUX_Data1(WriteData_wire),
	
	.MUX_Output(ALUOperand_B_wire)

);

//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
HazardDetectionUnit
HazardDetectionUnit1
(
	.ID_EX_MemRead(EX_MemRead_wire),
	.ID_EX_WriteRegister(WriteRegister_wire),
	.IF_ID_RegisterRs(ID_Instruction_wire[25:21]),
	.IF_ID_RegisterRt(ID_Instruction_wire[20:16]),

	.EnablePC(enablePC_wire),
	.EnableRegister_IF_ID(enableRegister_IF_ID_wire),
	.DisableControlSignals(disableControlSignals_wire)
	
);

Multiplexer2to1
#(
	.NBits(9)
)
MUX_StallForLW
(
	.Selector(disableControlSignals_wire),
	.MUX_Data0(9'b000000000),
	.MUX_Data1({RegWrite_wire, 
					RegDst_wire, 
					jal_wire,
					ALUSrc_wire,
					MemtoReg_wire,
					BranchEQ_wire,
					BranchNE_wire,
					MemRead_wire,
					MemWrite_wire}),
	
	.MUX_Output(controlSignals_wire)

);

//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
//******************************************************************/
Equals
EqualsRegs
(
	.Register0(ReadData1_wire),
	.Register1(ReadData2_wire),
	
	.Zero(Zero_wire)
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
	.Funct(ID_Instruction_wire[5:0]),
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
	.Jal(jal_wire),
	.Jr(jr_wire)
);

PC_Register
#(
	.N(32)
)
program_counter
(
	.clk(clk),
	.reset(reset),
	.enable(enablePC_wire),
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
	.WriteData(MEM_ReadData2_wire),
	.Address(MEM_ALUResult_wire[9:2]),
	.MemWrite(MEM_MemWrite_wire),
	.MemRead(MEM_MemRead_wire), 
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
	.Selector(EX_RegDst_wire),
	.MUX_Data0(EX_RT),//rt
	.MUX_Data1(EX_RD),//rd
	
	.MUX_Output(WriteRegister_wire)

);

Multiplexer2to1
#(
	.NBits(32)
)
MUX_ToRegFromRAMALU
(
	.Selector(WB_MemtoReg_wire),
	.MUX_Data0(WB_ALUResult_wire),
	.MUX_Data1(WB_RAM_DataOut),
	
	.MUX_Output(WriteDatanojal_wire)

);



Multiplexer2to1
#(
	.NBits(5)
)
MUX_Write_addRa
(
	.Selector(jal_wire),
	.MUX_Data0(WB_WriteRegister_wire),
	.MUX_Data1(5'b11111), //31 de $ra
	
	.MUX_Output(WriteRegisterOrRa)

);


Multiplexer2to1
#(
	.NBits(32)
)
MUX_Write_dataRa
(
	.Selector(jal_wire),
	.MUX_Data0(WriteDatanojal_wire), //lo usa el memtoreg
	.MUX_Data1(PC_4_wire),
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
	.Data1(ID_PC_4_wire),
	
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
	.MUX_Data1({ID_PC_4_wire[31:28], ID_Instruction_wire[25:0], 2'b00}), // Jump Address
	
	.MUX_Output(jumpAddress_wire)

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


RegisterFile
Register_File
(
	.clk(clk),
	.reset(reset),
	.RegWrite(secureRegWrite_wire),
	.WriteRegister(WriteRegisterOrRa),
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
	.Selector(EX_ALUSrc_wire),
	.MUX_Data0(EX_ReadData2_wire),
	.MUX_Data1(EX_InmmediateExtend_wire),
	
	.MUX_Output(ReadData2OrInmmediate_wire)

);


ALUControl
ArithmeticLogicUnitControl
(
	.ALUOp(EX_ALUOp_wire),
	.ALUFunction(EX_FUNCT),
	.ALUOperation(ALUOperation_wire)
	
);


ALU
ArithmeticLogicUnit 
(
	.ALUOperation(ALUOperation_wire),
	.A(ALUOperand_A_wire),
	.B(ALUOperand_B_wire),
	.shamt(EX_SHAMT),
	.ALUResult(ALUResult_wire)
);

assign ALUResultOut = ALUResult_wire;



assign branchA_wire = BranchNE_wire && ~Zero_wire;
assign branchB_wire = BranchEQ_wire && Zero_wire;
assign branch_wire  = branchA_wire || branchB_wire; // Para el multiplexor

//assign secureRegWrite_wire = WB_RegWrite_wire && ~jr_wire;
//assign secureRegWrite_wire = WB_RegWrite_wire && ~1'b0;

//assign jump_flags = branch_wire || jump_wire || jr_wire;
assign secureRegWrite_wire = RegWrite_wire && ~jr_wire;

assign flush_wire = (BranchEQ_wire && Zero_wire) || (BranchNE_wire && ~Zero_wire) || jump_wire || jr_wire;

endmodule
