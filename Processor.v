module PipelinedProcessor (
    input wire clk,
    input wire reset
);
    // Pipeline control signals
    wire PCWrite, IFIDWrite, ST;
    wire ALUSrc, MR, MW, MReg, EnRW;
    wire [1:0] ALUOp;
    wire [1:0] ForwardA, ForwardB;
    
    // IF stage wires
    wire [31:0] PC_Out, PC_Next, IF_Instruction;
    
    // ID stage wires
    wire [31:0] ID_PC, ID_Instruction;
    wire [31:0] ID_ReadData1, ID_ReadData2;
    wire [15:0] ID_Immediate;
    
    // EX stage wires
    wire EX_ALUSrc, EX_MR, EX_MW, EX_MReg, EX_EnRW;
    wire [1:0] EX_ALUOp;
    wire [31:0] EX_PC, EX_ReadData1, EX_ReadData2;
    wire [15:0] EX_Immediate;
    wire [3:0] EX_RegisterRs, EX_RegisterRt, EX_RegisterRd;
    wire [31:0] EX_ALU_A, EX_ALU_B, EX_ALUResult;
    
    // MEM stage wires
    wire MEM_MR, MEM_MW, MEM_MReg, MEM_EnRW;
    wire [31:0] MEM_ALUResult, MEM_WriteData, MEM_ReadData;
    wire [3:0] MEM_RegisterRd;
    
    // WB stage wires
    wire WB_MReg, WB_EnRW;
    wire [31:0] WB_ReadData, WB_ALUResult, WB_WriteData;
    wire [3:0] WB_RegisterRd;
    
    // PC incrementer
    assign PC_Next = PC_Out + 4;
    
    // Instantiate PC
    PC pc_module (
        .clk(clk),
        .reset(reset),
        .PCWrite(PCWrite),
        .PCNext(PC_Next),
        .PCOut(PC_Out)
    );
    
    // Instantiate Instruction Memory
    InstructionMemory imem (
        .clk(clk),
        .PCAddress(PC_Out),
        .EnIM(1'b1), // Always enabled
        .Instruction(IF_Instruction)
    );
    
    // Instantiate IF/ID Pipeline Register
    IF_ID_Register ifid (
        .clk(clk),
        .reset(reset),
        .IFIDWrite(IFIDWrite),
        .IF_PC(PC_Out),
        .IF_Instruction(IF_Instruction),
        .ID_PC(ID_PC),
        .ID_Instruction(ID_Instruction)
    );
    
    // Extract control and instruction fields
    wire [3:0] ID_Opcode = ID_Instruction[31:28];
    wire [3:0] ID_RegisterRd = ID_Instruction[27:24];
    wire [3:0] ID_RegisterRs = ID_Instruction[23:20];
    wire [3:0] ID_RegisterRt = ID_Instruction[19:16];
    assign ID_Immediate = ID_Instruction[15:0];
    
    // Instantiate Control Unit
    ControlUnit control (
        .Opcode(ID_Opcode),
        .ST(ST),
        .ALUSrc(ALUSrc),
        .ALUOp(ALUOp),
        .MR(MR),
        .MW(MW),
        .MReg(MReg),
        .EnRW(EnRW)
    );
    
    // Instantiate Register File
    RegisterFile regfile (
        .clk(clk),
        .EnRW(WB_EnRW),
        .RN1(ID_RegisterRs),
        .RN2(ID_RegisterRt),
        .WN(WB_RegisterRd),
        .WD(WB_WriteData),
        .RD1(ID_ReadData1),
        .RD2(ID_ReadData2)
    );
    
    // Instantiate Hazard Detection Unit
    HazardDetectionUnit hazard_detection (
        .ID_EX_RegisterRd(EX_RegisterRd),
        .ID_EX_MemRead(EX_MR),
        .IF_ID_RegisterRs(ID_RegisterRs),
        .IF_ID_RegisterRt(ID_RegisterRt),
        .PCWrite(PCWrite),
        .IFIDWrite(IFIDWrite),
        .ST(ST)
    );
    
    // Instantiate ID/EX Pipeline Register
    ID_EX_Register idex (
        .clk(clk),
        .reset(reset),
        .ID_ALUSrc(ALUSrc),
        .ID_ALUOp(ALUOp),
        .ID_MR(MR),
        .ID_MW(MW),
        .ID_MReg(MReg),
        .ID_EnRW(EnRW),
        .ID_PC(ID_PC),
        .ID_ReadData1(ID_ReadData1),
        .ID_ReadData2(ID_ReadData2),
        .ID_Immediate(ID_Immediate),
        .ID_RegisterRs(ID_RegisterRs),
        .ID_RegisterRt(ID_RegisterRt),
        .ID_RegisterRd(ID_RegisterRd),
        .EX_ALUSrc(EX_ALUSrc),
        .EX_ALUOp(EX_ALUOp),
        .EX_MR(EX_MR),
        .EX_MW(EX_MW),
        .EX_MReg(EX_MReg),
        .EX_EnRW(EX_EnRW),
        .EX_PC(EX_PC),
        .EX_ReadData1(EX_ReadData1),
        .EX_ReadData2(EX_ReadData2),
        .EX_Immediate(EX_Immediate),
        .EX_RegisterRs(EX_RegisterRs),
        .EX_RegisterRt(EX_RegisterRt),
        .EX_RegisterRd(EX_RegisterRd)
    );
    
    // Instantiate Forwarding Unit
    ForwardingUnit forwarding (
        .EX_MEM_RegisterRd(MEM_RegisterRd),
        .EX_MEM_RegWrite(MEM_EnRW),
        .MEM_WB_RegisterRd(WB_RegisterRd),
        .MEM_WB_RegWrite(WB_EnRW),
        .ID_EX_RegisterRs(EX_RegisterRs),
        .ID_EX_RegisterRt(EX_RegisterRt),
        .ForwardA(ForwardA),
        .ForwardB(ForwardB)
    );
    
    // ALU input multiplexers for forwarding
    wire [31:0] ALUInputA, ALUInputB;
    
// ForwardA Mux using continuous assignment
assign ALUInputA = (ForwardA == 2'b00) ? EX_ReadData1 :
                  (ForwardA == 2'b01) ? WB_WriteData :
                  (ForwardA == 2'b10) ? MEM_ALUResult : EX_ReadData1;
    
// ForwardB Mux using continuous assignment
assign ForwardB_Result = (ForwardB == 2'b00) ? EX_ReadData2 :
                        (ForwardB == 2'b01) ? WB_WriteData :
                        (ForwardB == 2'b10) ? MEM_ALUResult : EX_ReadData2;
    
    // ALUSrc Mux
    wire [31:0] Extended_Immediate = {{16{EX_Immediate[15]}}, EX_Immediate}; // Sign extend
    assign ALUInputB = (EX_ALUSrc) ? Extended_Immediate : ForwardB_Result;
    
    // Instantiate ALU
	ALU alu_unit (
        .A(ALUInputA),
        .B(ALUInputB),
        .ALUOp(EX_ALUOp),
        .Result(EX_ALUResult)
    );
    
    // Instantiate EX/MEM Pipeline Register
    EX_MEM_Register exmem (
        .clk(clk),
        .reset(reset),
        .EX_MR(EX_MR),
        .EX_MW(EX_MW),
        .EX_MReg(EX_MReg),
        .EX_EnRW(EX_EnRW),
        .EX_ALUResult(EX_ALUResult),
        .EX_WriteData(ForwardB_Result), // Use forwarded data
        .EX_RegisterRd(EX_RegisterRd),
        .MEM_MR(MEM_MR),
        .MEM_MW(MEM_MW),
        .MEM_MReg(MEM_MReg),
        .MEM_EnRW(MEM_EnRW),
        .MEM_ALUResult(MEM_ALUResult),
        .MEM_WriteData(MEM_WriteData),
        .MEM_RegisterRd(MEM_RegisterRd)
    );
    
    // Instantiate Data Memory
    DataMemory dmem (
        .clk(clk),
        .MR(MEM_MR),
        .MW(MEM_MW),
        .Address(MEM_ALUResult),
        .WriteData(MEM_WriteData),
        .ReadData(MEM_ReadData)
    );
    
    // Instantiate MEM/WB Pipeline Register
    MEM_WB_Register memwb (
        .clk(clk),
        .reset(reset),
        .MEM_MReg(MEM_MReg),
        .MEM_EnRW(MEM_EnRW),
        .MEM_ReadData(MEM_ReadData),
        .MEM_ALUResult(MEM_ALUResult),
        .MEM_RegisterRd(MEM_RegisterRd),
        .WB_MReg(WB_MReg),
        .WB_EnRW(WB_EnRW),
        .WB_ReadData(WB_ReadData),
        .WB_ALUResult(WB_ALUResult),
        .WB_RegisterRd(WB_RegisterRd)
    );
    
    // WB Mux
    assign WB_WriteData = WB_MReg ? WB_ReadData : WB_ALUResult;
    
endmodule