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