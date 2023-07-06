module veda_im(rst, mode, write_enable, address, data_in, data_out); //veda instruction memory

input [31:0] data_in;
input rst, mode, write_enable;
input [7:0] address;
output reg[31:0] data_out;

reg [7:0] addr;
reg [31:0] memory[127:0];

initial begin
    //bubble sort algorithm
    memory[0] = 32'b000000_00110_00011_00000_00010_000000;
    memory[1] = 32'b000000_00110_00000_00110_00000_100000;
    memory[2] = 32'b100011_00100_00110_00000_00000_000000;
    memory[3] = 32'b100011_00101_00110_00000_00000_000100;
    memory[4] = 32'b000000_00111_00100_00101_00000_101010;
    memory[5] = 32'b000101_00000_00111_01000_00000_001000;
    memory[6] = 32'b101011_00101_00110_00000_00000_000000;
    memory[7] = 32'b101011_00100_00110_00000_00000_000100;
    memory[8] = 32'b001000_00011_00011_00000_00000_000001;
    memory[9] = 32'b000000_01001_00010_00001_00000_100010;
    memory[10] = 32'b000101_00000_00011_01001_00000_000000;
    memory[11] = 32'b001000_00001_00001_00000_00000_000001;
    memory[12] = 32'b000000_00011_01000_01000_00000_100000;
    memory[13] = 32'b000101_00000_00001_00010_00000_000000;
end

always @(*) begin
    data_out <= memory[addr];
    if(rst) begin
        data_out <= 31'b0;
    end
    else if(mode) begin
        addr <= address;
    end
    else begin
        memory[address] <= data_in;
        addr <= address;
    end
    // data_out <= memory[addr];
end
endmodule

module add_inst(a,b,c); //module for addition

input [31:0] a,b;
output [31:0] c;

assign c = a + b;

endmodule

module sub_inst(a,b,c); //module for subtraction

input [31:0] a,b;
output [31:0] c;

assign c = a - b;

endmodule


module and_inst(a,b,c); //module for and

input [31:0] a,b;
output [31:0] c;

assign c = a & b;


endmodule

module or_inst(a,b,c); //module for or

input [31:0] a,b;
output [31:0] c;

assign c = a | b;

endmodule

module sll_inst(a,b,c); //module for left shift

input [31:0] a;
output [31:0]c;
input [4:0] b;

assign c = a << b;

endmodule

module srl_inst(a,b,c); //module for right shift

input [31:0] a;
output [31:0]c;
input [4:0] b;

assign c = a >> b;

endmodule

module decode(clk,out); //module for decoding the instruction (includes instruction fetch and ALU)
input clk;
reg [7:0] pc;

reg [5:0] Opcode;
reg [5:0] funct;

reg [4:0] rs;
reg [4:0] rd;
reg [4:0] rt;

reg [15:0] data;
reg [4:0] shamt;
reg [25:0] jump_loc;

integer i;

wire [31:0] instruction_register;
wire [31:0] data_register;

reg signed [31:0] veda_dm [63:0];

reg signed [31:0] registers [31:0];

wire [31:0] sumwire;
wire [31:0] subwire;
wire [31:0] andwire;
wire [31:0] orwire;
wire [31:0] sllwire;
wire [31:0] srlwire;

initial begin
    //initialising the data array (10 inputs)
    veda_dm[0] = 25;
    veda_dm[4] = -22;
    veda_dm[8] = 15;
    veda_dm[12] = 53;
    veda_dm[16] = 18;
    veda_dm[20] = -94;
    veda_dm[24] = 1822;
    veda_dm[28] = 6315;
    veda_dm[32] = 503;
    veda_dm[36] = 123;
    //display initial array
    $display("Input Array: %d %d %d %d %d %d %d %d %d %d", veda_dm[0], veda_dm[4], veda_dm[8], veda_dm[12], veda_dm[16], veda_dm[20], veda_dm[24], veda_dm[28], veda_dm[32], veda_dm[36]);
end

veda_im veda1(1'b0, 1'b1, 1'b0, pc, 32'b0, instruction_register); //fetching the instruction from veda


always @(*) begin
    if(pc>13) begin
        //display final sorted array
            $display("Sorted Array: %d %d %d %d %d %d %d %d %d %d", veda_dm[0], veda_dm[4], veda_dm[8], veda_dm[12], veda_dm[16], veda_dm[20], veda_dm[24], veda_dm[28], veda_dm[32], veda_dm[36]);
            $finish;
        end
end

initial begin
    pc = 0;
    registers[0] = 0; //$s7
    registers[1] = 0; //$s0
    registers[2] = 9; //$s6
    registers[3] = 0; //$s1
    registers[4] = 0; //$t0
    registers[5] = 0; //$t1
    registers[6] = 0; //$t7
    registers[7] = 0; //$t2
    registers[8] = 32'd0; //$zero
    registers[9] = 32'd0; //$s5

end
output reg [31:0] out;

//instantiating alu modules
add_inst uut1(registers[rs],registers[rt],sumwire);
sub_inst uut2(registers[rs],registers[rt],subwire);
and_inst uut3(registers[rs],registers[rt],andwire);
or_inst uut4(registers[rs],registers[rt],orwire);
sll_inst uut5(registers[rs],shamt,sllwire);
srl_inst uut6(registers[rs],shamt,srlwire);

always @(negedge clk)
begin
    //extracting instructions
    Opcode =  instruction_register[31:26];
    funct = instruction_register[5:0];
    rd = instruction_register[25:21];
    rs = instruction_register[20:16];
    rt = instruction_register[15:11];
    data =  instruction_register[15:0];
    shamt = instruction_register[10:6];
    jump_loc = instruction_register[25:0];
    
    case(Opcode) //decode and perform the instruction
        6'b000000: case(funct)

                     6'b100000: begin                               //add
                                
                                registers[rd] = sumwire;
                               out = sumwire;
                               pc = pc + 1;
                                end

                    6'b100010: begin                               //sub
                               
                               registers[rd] = subwire;
                               out = subwire;
                               pc = pc + 1;
                               end

                    6'b100001: begin                               //addu
                                
                                registers[rd] = sumwire;
                                pc = pc + 1;
                                end

                    6'b100011: begin                                //subu
                               
                               registers[rd] = subwire;
                               pc = pc + 1;
                               end

                    6'b100100: begin                                   //and
                               registers[rd] = registers[rs] & registers[rt];
                               pc = pc + 1;
                               out = registers[rd];

                               end

                    6'b100101: begin                                    //or
                               
                               registers[rd] = registers[rs] | registers[rt];
                               pc = pc + 1;
                               out = registers[rd];
                               end

                    6'b000000: begin                                    //sll
                               registers[rd] = sllwire;
                               pc = pc + 1;
                               out = registers[rd];
                               end

                    6'b000010: begin                                     //srl
                               
                               registers[rd] = srlwire;
                               pc = pc + 1;
                               out = registers[rd];
                               end

                    6'b001000: pc = registers[rs];                         //jr

                    6'b101010: begin                                     //slt

                               if(registers[rs] < registers[rt]) registers[rd] = 1;
                               else registers[rd] = 0;
                               pc = pc + 1;
                               end
                   endcase

        6'b001000 :     begin                                           //addi
                        registers[rd] = registers[rs] + data[15:0];
                        pc = pc + 1;
                        out = registers[rd];
                        end

        6'b001001 :     begin                                           //addiu
                        registers[rd] = registers[rs] + data[15:0];
                        pc = pc + 1;
                        out = registers[rd];
                        end

        6'b001100 :     begin                                           //andi
                        registers[rd] = registers[rs] & data[15:0];
                        pc = pc + 1;
                        out = registers[rd];
                        end

        6'b001101 :     begin                                           //ori
                        registers[rd] = registers[rs] | data[15:0];
                        pc = pc + 1;
                        out = registers[rd];
                        end

        6'b100011 :     begin                                               //lw
                        registers[rd] = veda_dm[registers[rs] +  data];
                        out = registers[rd];
                        pc = pc + 1;
                        end

        6'b101011 :     begin                                              //sw
                        veda_dm[registers[rs] +  data] = registers[rd];
                        out = registers[rd];
                        pc = pc + 1;
                        end

        6'b000100 :     if(registers[rt] == registers[rs]) begin            //beq
                        pc = pc + 1 + data[5:0]; 
                        out = pc;
                        end
        6'b000101 :     begin
            if(registers[rt] != registers[rs]) pc = data[9:0];              //bne
            else pc = pc + 1;
            out = pc;
        end

        6'b000111 :     begin                                                //bgt
            if(registers[rs] > registers[rt]) pc = pc + 1 + data[9:0];
        out = pc;
        end

        6'b001111 :     begin                                                //bgte
            if(registers[rs] >= registers[rt]) pc = pc + 1 + data[9:0];
        out = pc;
        end

        6'b000110 :     begin                                                //ble
        if(registers[rs] < registers[rt]) pc = pc + 1 + data[9:0];
        out = pc;
        end

        6'b011111 :     begin                                                //bleq
            if(registers[rs] <= registers[rt]) pc = pc + 1 + data[9:0];
        out = pc;
        end

        6'b000010 :     begin                                                //j
            pc = jump_loc[9:0];
        out = pc;
        end

        6'b000011 :     begin                                                //jal
                        registers[5'b11111] = pc + 1;
                        pc = jump_loc[9:0];
                        out = pc;
                        end

        6'b001010 :     begin
                        if(registers[rt] < data) registers[rs] = 1;         //slti
                        else registers[rs] = 0;
                        pc = pc + 1;
                        out = pc;
                        end


    endcase

end



endmodule





