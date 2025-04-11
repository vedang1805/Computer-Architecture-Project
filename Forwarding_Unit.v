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