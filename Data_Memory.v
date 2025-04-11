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