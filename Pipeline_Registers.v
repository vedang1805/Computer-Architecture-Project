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