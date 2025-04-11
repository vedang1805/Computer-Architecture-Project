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