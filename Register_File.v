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
