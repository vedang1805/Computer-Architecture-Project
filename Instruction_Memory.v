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