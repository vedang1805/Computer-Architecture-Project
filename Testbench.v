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