module PC (
    input wire clk,
    input wire reset,
    input wire PCWrite,
    input wire [31:0] PCNext,
    output reg [31:0] PCOut
);

    always @(posedge clk or posedge reset) begin
        if (reset)
            PCOut <= 32'h00000000;
        else if (PCWrite)
            PCOut <= PCNext;
    end
endmodule


module InstructionMemory (
    input wire [31:0] PCAddress,
    input wire EnIM,
    input wire clk,
    output reg [31:0] Instruction
);
    reg [7:0] memory [0:31]; // 32 bytes of memory
    
    // Initialize instruction memory with our program
    initial begin
        // SW reg1, 7(reg2) - 0000 0001 0010 0000 0000000000000111
        memory[0] = 8'h01;
        memory[1] = 8'h20;
        memory[2] = 8'h00;
        memory[3] = 8'h07;
        
        // NOR reg3, reg4, reg5 - 0001 0011 0100 0101 0000000000000000
        memory[4] = 8'h13;
        memory[5] = 8'h45;
        memory[6] = 8'h00;
        memory[7] = 8'h00;
        
        // ADDI reg6, reg3, 1078 - 0011 0110 0011 0000 0000010000110110
        memory[8] = 8'h36;
        memory[9] = 8'h30;
        memory[10] = 8'h04;
        memory[11] = 8'h36;
        
        // AND reg8, reg7, reg6 - 0111 1000 0111 0110 0000000000000000
        memory[12] = 8'h78;
        memory[13] = 8'h76;
        memory[14] = 8'h00;
        memory[15] = 8'h00;
        
        // OR reg9, reg8, reg3 - 1111 1001 1000 0011 0000000000000000
        memory[16] = 8'hF9;
        memory[17] = 8'h83;
        memory[18] = 8'h00;
        memory[19] = 8'h00;
    end
    
    always @(posedge clk) begin
    if (EnIM) begin
        // Use only the relevant bits of the address and ensure it's within range
        Instruction <= {
            memory[PCAddress[4:0]],         // Lowest 5 bits for 32 bytes
            memory[PCAddress[4:0] + 1],
            memory[PCAddress[4:0] + 2],
            memory[PCAddress[4:0] + 3]
        };
    end
end
endmodule


module RegisterFile (
    input wire clk,
    input wire EnRW,
    input wire [3:0] RN1,
    input wire [3:0] RN2,
    input wire [3:0] WN,
    input wire [31:0] WD,
    output wire [31:0] RD1,
    output wire [31:0] RD2
);
    reg [31:0] registers [0:15]; // 16 registers, each 32 bits
    
    // Initialize registers with given values
    initial begin
        registers[0] = 32'h00000000; // reg0 is hardwired to 0
        registers[1] = 32'h00016326; // reg1 = 90966 (decimal)
        registers[2] = 32'h00000005; // reg2 = 5
        registers[3] = 32'h00000000; // reg3 initially 0
        registers[4] = 32'h000FE331; // reg4 = FE331
        registers[5] = 32'h00045432; // reg5 = 45432
        registers[6] = 32'h00000000; // reg6 initially 0
        registers[7] = 32'h00023211; // reg7 = 23211
        registers[8] = 32'h00000000; // reg8 initially 0
        registers[9] = 32'h00000000; // reg9 initially 0
        registers[10] = 32'h00000000;
        registers[11] = 32'h00000000;
        registers[12] = 32'h00000000;
        registers[13] = 32'h00000000;
        registers[14] = 32'h00000000;
        registers[15] = 32'h00000000;
    end
    
    // Read ports (synchronous read)
    assign RD1 = registers[RN1];
    assign RD2 = registers[RN2];
    
    // Write port (falling edge)
    always @(negedge clk) begin
        if (EnRW && WN != 4'b0000) // reg0 cannot be written to
            registers[WN] <= WD;
    end
endmodule

module ALU (
    input wire [31:0] A,
    input wire [31:0] B,
    input wire [1:0] ALUOp,
    output reg [31:0] Result
);
    // ALU Operations
    parameter ADD = 2'b00;
    parameter AND = 2'b01;
    parameter OR  = 2'b10;
    parameter NOR = 2'b11;
    
    always @(*) begin
        case (ALUOp)
            ADD: Result = A + B;
            AND: Result = A & B;
            OR:  Result = A | B;
            NOR: Result = ~(A | B);
        endcase
    end
endmodule

module DataMemory (
    input wire clk,
    input wire MR,
    input wire MW,
    input wire [31:0] Address,
    input wire [31:0] WriteData,
    output reg [31:0] ReadData
);
    reg [31:0] memory [0:255]; // 1KB memory
    
    // Memory read
    always @(posedge clk) begin
        if (MR)
            ReadData <= memory[Address];
    end
    
    // Memory write
    always @(negedge clk) begin
        if (MW)
            memory[Address] <= WriteData;
    end
endmodule

module ControlUnit (
    input wire [3:0] Opcode,
    input wire ST,
    output reg ALUSrc,
    output reg [1:0] ALUOp,
    output reg MR,
    output reg MW,
    output reg MReg,
    output reg EnRW
);
    // Define opcodes
    parameter SW  = 4'b0000;
    parameter NOR = 4'b0001;
    parameter ADDI = 4'b0011;
    parameter AND = 4'b0111;
    parameter OR  = 4'b1111;
    
    always @(*) begin
        if (ST) begin
            // Set all control signals to 0 if stall
            ALUSrc = 1'b0;
            ALUOp = 2'b00;
            MR = 1'b0;
            MW = 1'b0;
            MReg = 1'b0;
            EnRW = 1'b0;
        end else begin
            case (Opcode)
                SW: begin
                    ALUSrc = 1'b1; // Use immediate
                    ALUOp = 2'b00; // ADD
                    MR = 1'b0;
                    MW = 1'b1; // Memory write
                    MReg = 1'b0;
                    EnRW = 1'b0; // No register write
                end
                NOR: begin
                    ALUSrc = 1'b0; // Use register
                    ALUOp = 2'b11; // NOR
                    MR = 1'b0;
                    MW = 1'b0;
                    MReg = 1'b0;
                    EnRW = 1'b1; // Register write
                end
                ADDI: begin
                    ALUSrc = 1'b1; // Use immediate
                    ALUOp = 2'b00; // ADD
                    MR = 1'b0;
                    MW = 1'b0;
                    MReg = 1'b0;
                    EnRW = 1'b1; // Register write
                end
                AND: begin
                    ALUSrc = 1'b0; // Use register
                    ALUOp = 2'b01; // AND
                    MR = 1'b0;
                    MW = 1'b0;
                    MReg = 1'b0;
                    EnRW = 1'b1; // Register write
                end
                OR: begin
                    ALUSrc = 1'b0; // Use register
                    ALUOp = 2'b10; // OR
                    MR = 1'b0;
                    MW = 1'b0;
                    MReg = 1'b0;
                    EnRW = 1'b1; // Register write
                end
                default: begin
                    ALUSrc = 1'b0;
                    ALUOp = 2'b00;
                    MR = 1'b0;
                    MW = 1'b0;
                    MReg = 1'b0;
                    EnRW = 1'b0;
                end
            endcase
        end
    end
endmodule


module HazardDetectionUnit (
    input wire [3:0] ID_EX_RegisterRd,  // Destination register of load instruction
    input wire ID_EX_MemRead,           // Whether it's a load instruction
    input wire [3:0] IF_ID_RegisterRs,  // Source register 1 of next instruction
    input wire [3:0] IF_ID_RegisterRt,  // Source register 2 of next instruction
    output reg PCWrite,
    output reg IFIDWrite,
    output reg ST
);
    always @(*) begin
        // Load-use hazard detection
        if (ID_EX_MemRead && 
            ((ID_EX_RegisterRd == IF_ID_RegisterRs) || 
             (ID_EX_RegisterRd == IF_ID_RegisterRt))) begin
            PCWrite = 1'b0;   // Stall PC
            IFIDWrite = 1'b0; // Stall IF/ID register
            ST = 1'b1;        // Insert bubble (NOP)
        end else begin
            PCWrite = 1'b1;
            IFIDWrite = 1'b1;
            ST = 1'b0;
        end
    end
endmodule


module ForwardingUnit (
    input wire [3:0] EX_MEM_RegisterRd,
    input wire EX_MEM_RegWrite,
    input wire [3:0] MEM_WB_RegisterRd,
    input wire MEM_WB_RegWrite,
    input wire [3:0] ID_EX_RegisterRs,
    input wire [3:0] ID_EX_RegisterRt,
    output reg [1:0] ForwardA,
    output reg [1:0] ForwardB
);
    always @(*) begin
        // Forward A logic
        if (EX_MEM_RegWrite && (EX_MEM_RegisterRd != 0) && 
            (EX_MEM_RegisterRd == ID_EX_RegisterRs))
            ForwardA = 2'b10; // Forward from EX/MEM
        else if (MEM_WB_RegWrite && (MEM_WB_RegisterRd != 0) && 
                 (MEM_WB_RegisterRd == ID_EX_RegisterRs))
            ForwardA = 2'b01; // Forward from MEM/WB
        else
            ForwardA = 2'b00; // No forwarding
        
        // Forward B logic
        if (EX_MEM_RegWrite && (EX_MEM_RegisterRd != 0) && 
            (EX_MEM_RegisterRd == ID_EX_RegisterRt))
            ForwardB = 2'b10; // Forward from EX/MEM
        else if (MEM_WB_RegWrite && (MEM_WB_RegisterRd != 0) && 
                 (MEM_WB_RegisterRd == ID_EX_RegisterRt))
            ForwardB = 2'b01; // Forward from MEM/WB
        else
            ForwardB = 2'b00; // No forwarding
    end
endmodule


// IF/ID Pipeline Register
module IF_ID_Register (
    input wire clk,
    input wire reset,
    input wire IFIDWrite,
    input wire [31:0] IF_PC,
    input wire [31:0] IF_Instruction,
    output reg [31:0] ID_PC,
    output reg [31:0] ID_Instruction
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ID_PC <= 32'h00000000;
            ID_Instruction <= 32'h00000000;
        end else if (IFIDWrite) begin
            ID_PC <= IF_PC;
            ID_Instruction <= IF_Instruction;
        end
    end
endmodule

// ID/EX Pipeline Register
module ID_EX_Register (
    input wire clk,
    input wire reset,
    // Control signals
    input wire ID_ALUSrc,
    input wire [1:0] ID_ALUOp,
    input wire ID_MR,
    input wire ID_MW,
    input wire ID_MReg,
    input wire ID_EnRW,
    // Data signals
    input wire [31:0] ID_PC,
    input wire [31:0] ID_ReadData1,
    input wire [31:0] ID_ReadData2,
    input wire [15:0] ID_Immediate,
    input wire [3:0] ID_RegisterRs,
    input wire [3:0] ID_RegisterRt,
    input wire [3:0] ID_RegisterRd,
    // Outputs
    output reg EX_ALUSrc,
    output reg [1:0] EX_ALUOp,
    output reg EX_MR,
    output reg EX_MW,
    output reg EX_MReg,
    output reg EX_EnRW,
    output reg [31:0] EX_PC,
    output reg [31:0] EX_ReadData1,
    output reg [31:0] EX_ReadData2,
    output reg [15:0] EX_Immediate,
    output reg [3:0] EX_RegisterRs,
    output reg [3:0] EX_RegisterRt,
    output reg [3:0] EX_RegisterRd
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all values
            EX_ALUSrc <= 1'b0;
            EX_ALUOp <= 2'b00;
            EX_MR <= 1'b0;
            EX_MW <= 1'b0;
            EX_MReg <= 1'b0;
            EX_EnRW <= 1'b0;
            EX_PC <= 32'h00000000;
            EX_ReadData1 <= 32'h00000000;
            EX_ReadData2 <= 32'h00000000;
            EX_Immediate <= 16'h0000;
            EX_RegisterRs <= 4'h0;
            EX_RegisterRt <= 4'h0;
            EX_RegisterRd <= 4'h0;
        end else begin
            // Pass values to next stage
            EX_ALUSrc <= ID_ALUSrc;
            EX_ALUOp <= ID_ALUOp;
            EX_MR <= ID_MR;
            EX_MW <= ID_MW;
            EX_MReg <= ID_MReg;
            EX_EnRW <= ID_EnRW;
            EX_PC <= ID_PC;
            EX_ReadData1 <= ID_ReadData1;
            EX_ReadData2 <= ID_ReadData2;
            EX_Immediate <= ID_Immediate;
            EX_RegisterRs <= ID_RegisterRs;
            EX_RegisterRt <= ID_RegisterRt;
            EX_RegisterRd <= ID_RegisterRd;
        end
    end
endmodule

// EX/MEM Pipeline Register
module EX_MEM_Register (
    input wire clk,
    input wire reset,
    // Control signals
    input wire EX_MR,
    input wire EX_MW,
    input wire EX_MReg,
    input wire EX_EnRW,
    // Data signals
    input wire [31:0] EX_ALUResult,
    input wire [31:0] EX_WriteData,
    input wire [3:0] EX_RegisterRd,
    // Outputs
    output reg MEM_MR,
    output reg MEM_MW,
    output reg MEM_MReg,
    output reg MEM_EnRW,
    output reg [31:0] MEM_ALUResult,
    output reg [31:0] MEM_WriteData,
    output reg [3:0] MEM_RegisterRd
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all values
            MEM_MR <= 1'b0;
            MEM_MW <= 1'b0;
            MEM_MReg <= 1'b0;
            MEM_EnRW <= 1'b0;
            MEM_ALUResult <= 32'h00000000;
            MEM_WriteData <= 32'h00000000;
            MEM_RegisterRd <= 4'h0;
        end else begin
            // Pass values to next stage
            MEM_MR <= EX_MR;
            MEM_MW <= EX_MW;
            MEM_MReg <= EX_MReg;
            MEM_EnRW <= EX_EnRW;
            MEM_ALUResult <= EX_ALUResult;
            MEM_WriteData <= EX_WriteData;
            MEM_RegisterRd <= EX_RegisterRd;
        end
    end
endmodule

// MEM/WB Pipeline Register
module MEM_WB_Register (
    input wire clk,
    input wire reset,
    // Control signals
    input wire MEM_MReg,
    input wire MEM_EnRW,
    // Data signals
    input wire [31:0] MEM_ReadData,
    input wire [31:0] MEM_ALUResult,
    input wire [3:0] MEM_RegisterRd,
    // Outputs
    output reg WB_MReg,
    output reg WB_EnRW,
    output reg [31:0] WB_ReadData,
    output reg [31:0] WB_ALUResult,
    output reg [3:0] WB_RegisterRd
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all values
            WB_MReg <= 1'b0;
            WB_EnRW <= 1'b0;
            WB_ReadData <= 32'h00000000;
            WB_ALUResult <= 32'h00000000;
            WB_RegisterRd <= 4'h0;
        end else begin
            // Pass values to next stage
            WB_MReg <= MEM_MReg;
            WB_EnRW <= MEM_EnRW;
            WB_ReadData <= MEM_ReadData;
            WB_ALUResult <= MEM_ALUResult;
            WB_RegisterRd <= MEM_RegisterRd;
        end
    end
endmodule


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


module PipelinedProcessor_tb;
    reg clk;
    reg reset;
    
    // Instantiate the processor
    PipelinedProcessor processor (
        .clk(clk),
        .reset(reset)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end
    
    // Test sequence
    initial begin
        // Initialize VCD file for GTKWave
        $dumpfile("processor_waveform.vcd");
        $dumpvars(0, PipelinedProcessor_tb);
        
        // Apply reset
        reset = 1;
        #15;
        reset = 0;
        
        // Run for enough cycles to execute all instructions
        #200;
        
        // End simulation
        $finish;
    end
    
    // Monitor register values
    initial begin
        $monitor("Time=%0t, PC=%h, Inst=%h, reg1=%h, reg2=%h, reg3=%h, reg4=%h, reg5=%h, reg6=%h, reg7=%h, reg8=%h, reg9=%h",
                 $time, processor.pc_module.PCOut, processor.ifid.ID_Instruction,
                 processor.regfile.registers[1], processor.regfile.registers[2],
                 processor.regfile.registers[3], processor.regfile.registers[4],
                 processor.regfile.registers[5], processor.regfile.registers[6],
                 processor.regfile.registers[7], processor.regfile.registers[8],
                 processor.regfile.registers[9]);
    end
    
endmodule