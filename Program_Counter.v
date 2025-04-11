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
