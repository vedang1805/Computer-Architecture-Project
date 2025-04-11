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
