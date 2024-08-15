module cpu (
    input logic clk,
    input logic reset,
    input logic [18:0] instruction,  
    output logic [18:0] alu_out,     
    output logic [15:0] mem_addr,    
    output logic mem_write,          
    output logic [18:0] mem_data_in, 
    input logic [18:0] mem_data_out );

 
    logic [18:0] register_file[15:0];

    
    logic [15:0] pc;
    logic [15:0] sp;

    
    logic [18:0] mem_read_data;
    memory mem_inst (
        .clk(clk),
        .address(mem_addr),
        .mem_write(mem_write),
        .data_in(mem_data_in),
        .data_out(mem_read_data));

    
    typedef enum logic [4:0] {
        ADD = 5'b00000,
        SUB = 5'b00001,
        MUL = 5'b00010,
        DIV = 5'b00011,
        INC = 5'b00100,
        DEC = 5'b00101,
        AND = 5'b00110,
        OR  = 5'b00111,
        XOR = 5'b01000,
        NOT = 5'b01001,
        JMP = 5'b01010,
        BEQ = 5'b01011,
        BNE = 5'b01100,
        CALL = 5'b01101,
        RET = 5'b01110,
        LD  = 5'b01111,
        ST  = 5'b10000,
        FFT = 5'b10001,
        ENC = 5'b10010,
        DECC = 5'b10011} opcode_t;

    opcode_t opcode;
    logic [3:0] rd, rs1, rs2;
    logic [15:0] address;

    always_comb 
	begin
        opcode = opcode_t'( instruction[18:14]);
        rd = instruction[13:10];
        rs1 = instruction[9:6];
        rs2 = instruction[5:2];
        address = instruction[15:0];
    end

    // ALU Operations
    always_comb 
	begin
        case (opcode)
            ADD: alu_out = register_file[rs1] + register_file[rs2];
            SUB: alu_out = register_file[rs1] - register_file[rs2];
            MUL: alu_out = register_file[rs1] * register_file[rs2];
            DIV: alu_out = register_file[rs1] / register_file[rs2];
            INC: alu_out = register_file[rs1] + 19'b1;
            DEC: alu_out = register_file[rs1] - 19'b1;
            AND: alu_out = register_file[rs1] & register_file[rs2];
            OR:  alu_out = register_file[rs1] | register_file[rs2];
            XOR: alu_out = register_file[rs1] ^ register_file[rs2];
            NOT: alu_out = ~register_file[rs1];
            FFT: alu_out = fft_operation(register_file[rs1], register_file[rs2]);
            ENC: alu_out = encryption(register_file[rs1], register_file[rs2]);
            DECC: alu_out = decryption(register_file[rs1], register_file[rs2]);
            default: alu_out = 19'b0;
        endcase
    end

    // Memories Read/Write and Control Flow
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 16'h0000;
            sp <= 16'hFFFF;  // Initialize stock pointe
            mem_write <= 1'b0;
        end else begin
            case (opcode)
                LD: begin
                    mem_addr <= address;
                    register_file[rs1] <= mem_read_data;
                end
                ST: begin
                    mem_addr <= address;
                    mem_data_in <= register_file[rs1];
                    mem_write <= 1'b1;
                end
                JMP: begin
                    pc <= address;
                end
                BEQ: begin
                    if (register_file[rs1] == register_file[rs2])
                        pc <= address;
                    else
                        pc <= pc + 1;
                end
                BNE: begin
                    if (register_file[rs1] != register_file[rs2])
                        pc <= address;
                    else
                        pc <= pc + 1;
                end
                CALL: begin
                    sp <= sp - 1;
                    register_file[sp] <= pc + 1;
                    pc <= address;
                end
                RET: begin
                    sp <= sp + 1;
                    pc <= register_file[sp];
                end
                default: begin
                    pc <= pc + 1;
                    mem_write <= 1'b0; // Ensure memory write is dibled when not storing
                end
            endcase
        end
    end

    // Custum Operations
    function [18:0] fft_operation(input [18:0] src1, input [18:0] src2);
        
        reg [18:0] real_part, imag_part;
        real_part = (src1 + src2) >> 1;
        imag_part = (src1 - src2) >> 1;
        fft_operation = {real_part[9:0], imag_part[8:0]};
    endfunction

    function [18:0] encryption(input [18:0] data, input [18:0] key);
        encryption = data ^ key;
    endfunction

    function [18:0] decryption(input [18:0] data, input [18:0] key);
        decryption = data ^ key;
    endfunction
endmodule
