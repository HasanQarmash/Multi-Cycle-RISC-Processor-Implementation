 /* 
* HASAN QARMASH    1210611  
* Instructors Names: Dr. Aziz Qaroush. & Dr. Ayman Hroub
 		   22/6/2024 6:00pm saturday
 */
 




parameter

// levels
    LOW = 1'b0,
    HIGH = 1'b1,




// instruction function code
// 5-bit instruction function code

    // R-Type Instructions Project
    AND = 4'b0000, // Reg(Rd) = Reg(Rs1) & Reg(Rs2) 
    ADD = 4'b0001, // Reg(Rd) = Reg(Rs1) + Reg(Rs2) 
    SUB = 4'b0010, // Reg(Rd) = Reg(Rs1) - Reg(Rs2) 
   // CMP = 5'b00011, // zero-signal = Reg(Rs) == Reg(Rs2), negative-signal = Reg(Rs) < Reg(Rs2)

    // I-Type Instructions
    ADDI = 4'b0011, // Reg(Rd) = Reg(Rs1) & Immediate5->16
    ANDI = 4'b0100, // Reg(Rd) = Reg(Rs1) + Immediate5->16
    LW   = 4'b0101, // Reg(Rd) = Mem(Reg(Rs1) + Imm_5)
	LBu  = 4'b0110, // Reg(Rd) = Mem(Reg(Rs1) + Imm_unsigned) 
	LBs  = 4'b0110, // Reg(Rd) = Mem(Reg(Rs1) + Imm_signed) 
    SW   = 4'b0111, // Mem(Reg(Rs1) + Imm_5) = Reg(Rd)	 
	
	BGT	 = 4'b1000,	 //if (Reg(Rd) > Reg(Rs1))
                     // Next PC = PC + sign_extended (Imm)
	                 // else PC = PC + 2 
		
	BGTZ = 4'b1000,	//if (Reg(Rd) > Reg(0))	
		            // Next PC = PC + sign_extended (Imm)
                    // Next PC = PC +2
	
	BLT	 = 4'b1001,	//if (Reg(Rd) < Reg(Rs1))
					//Next PC = PC + sign_extended (Imm)
					//else PC = PC + 2
    
	BLTZ = 4'b1001,	// if (Reg(Rd) < Reg(R0))
					//Next PC = PC + sign_extended (Imm)
					//else PC = PC + 2

    BEQ  = 4'b1010, //if (Reg(Rd) == Reg(Rs1))
					//Next PC = PC + sign_extended (Imm)
					//else PC = PC + 2
	BEQZ = 4'b1010,	 //if (Reg(Rd) == Reg(R0))
					 //Next PC = PC + sign_extended (Imm)
					 //else PC = PC + 2
	
	BNE	 = 4'b1011,	  //if (Reg(Rd) != Reg(Rs1))
					 //Next PC = PC + sign_extended (Imm)	 
	                 //else PC = PC + 2	
		
	BNEZ = 4'b1011, //if (Reg(Rd) != Reg(Rs1))
					//Next PC = PC + sign_extended (Imm)
					//else PC = PC + 2
	
	
    // J-Type Instructions
   JMP  = 4'b1100, // Next PC = {PC[15:12], Immediate}
   CALL = 4'b1101, // Next PC = {PC[15:12], Immediate} PC + 2 is saved on r15
   RET  = 4'b1110, //Next PC = r7	
   
    // S-Type Instructions
    Sv  = 4'b1111, // M[rs] = imm


// ALU function code signal
// 2-bit chip-select for ALU


  			  ALU_Add = 4'b0000, // used in ADD, ADDI, LW, SW, JAL
		      ALU_Sub = 4'b0001, // used in SUB
	          ALU_And = 4'b0010, // used in AND, ANDI 

              ALU_BGT  = 4'b0011,
              ALU_BGTZ = 4'b0100,
              ALU_BLT  = 4'b0101,
              ALU_BLTZ = 4'b0110,
              ALU_BEQ  = 4'b0111,
              ALU_BEQZ = 4'b1000,
              ALU_BNE = 4'b1001,
              ALU_BNEZ = 4'b1010, 
			  ALU_LBs= 4'b1011,

// PC source signal
// 2-bit source select for next PC value
  
    PC_Src_Dft = 2'b00, // PC = PC + 2
    PC_Src_Ra  = 2'b01, // return address from stack	
    PC_Src_Jmp = 2'b11, // jump address
    PC_Src_BTA = 2'b10, // branch target address

// ALU source signal
// 2-bit source select for next PC value

  //  ALU_Src_SAi = 2'b10, // If S-Type instruction && Function is (SLL || SLR), SA_5 is used as operand2 
    ALU_Src_Reg = 1'b0, // Else If S-Type || R-Type instruction, RB is used as operand2
    //ALU_Src_SIm = 2'b00, // Else If I-Type instruction && ! (ANDI), Signed Immediate_5 is used as operand2	 BRANCH INST
    ALU_Src_UIm = 1'b1, // Else If I-Type instruction && Unsigned Immediate14 is used as operand2

// 8 registers -16-bit
    R0 = 3'd0, // zero register
    R1 = 3'd1, // general purpose register
    R2 = 3'd2, // general purpose register
    R3 = 3'd3, // general purpose register
    R4 = 3'd4, // general purpose register
    R5 = 3'd5, // general purpose register
    R6 = 3'd6, // general purpose register
    R7 = 3'd7; // general purpose register
   

    
	
	
	
	
	
	
	
	
	
	

// outputs control signals and stage enable signals to the rest of the processor
// takes intruction-type, function-code, and flags as inputs   


module controlUnit(
    clock,

    // signal outputs
    sig_alu_op,	//here this signal to choice op code for instruction if and or add or sub or branch
    sig_pc_src, 
    sig_rb_src, // this signal to choice rd or rs2 in bus2	
	
	sig_rsORpc_src,//this signal to choice rs1 or pc value in I type
	//sig_r0ORrs_src, //this signal to choice 
	sig_rs1_src, //this signal to choice output depend on op code
	sig_rs2_src,
    sig_alu_src,  // this signal to choice bus2 or 4 type output extender
    sig_rf_enable_write, // this signal on rigster file to wite result
    sig_enable_data_memory_write, //data mem
    sig_enable_data_memory_read,  //data mem 
	//sig_enable_data_memory_read_byte, //data mem
    sig_write_back_data_select,	   // signal to choice between output alu or data memory
   	sig_address_src,
	sig_data_src,
	
    // stage enable outputs
    en_instruction_fetch,
    en_instruction_decode,
    en_execute,

    // inputs
  //   S ,R,J,I
    OpCode,//op code for our project 
    flag_zero, 
	flag_negative,
	flag_overflow,
	
	mood,
	sig_BZ
    
    );

    // ----------------- INPUTS -----------------

    // clock
    input wire clock;

    // instruction type [ R, S, I, J ]
   
	
	
    // op code
    input wire [3:0] OpCode;

    // zero flag
    input wire flag_zero,flag_negative,
	flag_overflow;
	
	//input mood for lws and branch
		input wire mood;

    // ----------------- OUTPUTS -----------------
    
    // Mux signals
    output reg [3:0] sig_alu_op;
    output reg [1:0] sig_pc_src = PC_Src_Dft;
    output reg sig_alu_src;
    
    output reg sig_write_back_data_select,
                sig_rb_src,sig_rsORpc_src,sig_rs1_src,sig_rs2_src,sig_address_src,sig_BZ,sig_data_src;

    // operation enable signals
    output reg sig_rf_enable_write = LOW,
                sig_enable_data_memory_write = LOW;
               
	output reg [1:0] sig_enable_data_memory_read = 2'b00;		
    
    // stage enable signals ( used as clock for each stage )
    output reg en_execute = LOW,
                en_instruction_fetch = LOW,
                en_instruction_decode = LOW;

    // ----------------- INTERNALS -----------------

  // binary codes for stages
    `define STG_FTCH 3'b000
    `define STG_DCDE 3'b001
    `define STG_EXEC 3'b010
    `define STG_MEM 3'b011
    `define STG_WRB 3'b100
    `define STG_INIT 3'b101	 // STAGE Initial
    
 
    reg [2:0] current_stage = `STG_INIT;
    reg [2:0] next_stage = `STG_FTCH; 
	
	    always@(posedge clock) begin // maybe use negative edge

        current_stage = next_stage;

    end
	
  ///////////////////////////////////////////////////////////////////
	
	    always@(posedge clock) begin // maybe use negative edge

        case (current_stage)

            `STG_INIT: begin                                    
                en_instruction_fetch = LOW; // fetch after finding PC src
                next_stage <= `STG_FTCH;
            end

            `STG_FTCH: begin 
                // disable previous stage
                en_instruction_decode = LOW;
                en_execute = LOW;

                // disable signals
                sig_rf_enable_write = LOW;
                sig_enable_data_memory_write = LOW;
                sig_enable_data_memory_read = 2'b00;
				

                // enable current stage
                en_instruction_fetch = HIGH; // fetch after finding PC src

                // next stage
                next_stage <= `STG_DCDE;
													
                // ---------------------------------------------
                // ------------ set control signals ------------
                // ---------------------------------------------

                // ---- PC Source ----

                // here the inputs are the ones from the previous stage
               if (OpCode == RET ) begin

                    sig_pc_src = PC_Src_Ra; // use return address as PC

             end  else if ( OpCode == JMP ||  OpCode == CALL ) begin

				   sig_pc_src = PC_Src_Jmp; // use JTA as PC	
				   			
				
     	end else if (OpCode == BEQZ && flag_zero == HIGH && mood ==HIGH) begin
        sig_pc_src = PC_Src_BTA;
        end else if (OpCode == BEQ && flag_zero == HIGH && mood ==LOW) begin
        sig_pc_src = PC_Src_BTA;								  
        end else if (OpCode == BNEZ && flag_zero == LOW && mood ==HIGH) begin
        sig_pc_src = PC_Src_BTA;
        end else if (OpCode == BNE && flag_zero == LOW && mood ==LOW) begin
        sig_pc_src = PC_Src_BTA;	 
        end else if (OpCode == BLT && flag_negative == HIGH && mood ==LOW) begin
        sig_pc_src = PC_Src_BTA;	
        end else if (OpCode == BLTZ && flag_negative == HIGH && mood ==HIGH) begin
        sig_pc_src = PC_Src_BTA;
        end else if (OpCode == BGT && flag_negative == LOW && flag_zero == LOW  && mood ==LOW) begin
        sig_pc_src = PC_Src_BTA;
        end else if (OpCode == BGTZ && flag_negative == LOW && flag_zero == LOW  && mood ==HIGH) begin
        sig_pc_src = PC_Src_BTA;
		end	else begin

                    sig_pc_src = PC_Src_Dft; // use next instruction address as PC

		     	end
			
            
            end
			
			

            `STG_DCDE: begin 
				
				
				      // disable previous stage
                en_instruction_fetch = LOW;

                // enable current stage
                en_instruction_decode = HIGH;
                
                 // next stage
                next_stage <= ( OpCode == JMP || OpCode == CALL || OpCode == RET ) ? `STG_FTCH : `STG_EXEC;
				
				
				

                // ---------------------------------------------		 
                // ------------ set control signals ------------
                // --------------------------------------------- 
				

                // ---- RS1 Source ----	
				
                sig_rs1_src = (OpCode > SUB) ? HIGH : LOW; 	//I-type 1 or R-type  0
				
				 // ---- RS2 Source ----
                sig_rs2_src = (OpCode > SUB && OpCode!=Sv ) ? HIGH : LOW; //rd or rs2
				
				 // ---- RD Source ----
                sig_rb_src = (OpCode > SUB) ? HIGH : LOW; //I-type-1 or R-type-0 
				sig_BZ=  ( (OpCode == BEQZ || OpCode == BNEZ  || 
				OpCode == BLTZ ||  OpCode == BGTZ ) && mood ===HIGH)   ?HIGH : LOW;	
				
				
				
				sig_rf_enable_write = (OpCode == JMP || OpCode == CALL) ? HIGH:LOW;
				
				// -----RS OR PC SOURSE	
				/*sig_rsORpc_src = (InstructionType == I_Type && 
                      (OpCode == BEQZ || OpCode == BEQ || OpCode == BNEZ || OpCode == BNE || 
                       OpCode == BLT || OpCode == BLTZ || OpCode == BGT || OpCode == BGTZ)) ? HIGH : LOW; */
			 
				
                // ---- ALU Source ----

                
          // ---- ALU Source ----
    if  (OpCode < SUB || OpCode == SUB || ( OpCode >SW && OpCode < JMP ) )  begin
        // R-Type that uses register 
        sig_alu_src = ALU_Src_Reg; // use Rb as operand
    end else if  (OpCode > SUB )  begin
        // ANDI instruction uses unsigned immediate
        sig_alu_src = ALU_Src_UIm; // use unsigned immediate as operand
    end //else if (InstructionType == I_Type && OpCode != ANDI) begin
        // I_Type instruction other than ANDI uses signed immediate
      //  sig_alu_src = ALU_Src_SIm; // use signed immediate as operand
    //end
end	  




            `STG_EXEC: begin 
				
				
	    // disable previous stage
                en_instruction_decode = LOW;

        // enable current stage
                en_execute = HIGH;

                // next stage
                if  ( 
                    ( OpCode inside { CALL,RET} ) || 
                    ( OpCode inside {LW, SW,LBs,LBu,Sv} ) ) begin 

                    next_stage <= `STG_MEM;

                end else if ( 
                    (OpCode inside {BEQZ,BEQ,BNEZ,BNE,BLT,BLTZ,BGT,BGTZ} )) begin 

                    next_stage <= `STG_FTCH;

                end else begin

                    next_stage <= `STG_WRB;

                end

                // ---------------------------------------------
                // ------------ set control signals ------------
                // ---------------------------------------------

                // ---- ALU Operation ----
				

			 if ( 
                    (  OpCode == ANDI ) || 
                    (  OpCode == AND ) ) begin 

                    sig_alu_op = ALU_And; // bitwise AND

                end else if ( OpCode == SUB ) begin
                    
                    sig_alu_op = ALU_Sub; // subtract

                end else    if (OpCode == ADD || OpCode == ADDI || OpCode == LW || OpCode == SW) begin
                    sig_alu_op = ALU_Add; // Set ALU operation to add
                   
				
				 ////////////////////////////
				 //mood	  Lood
			   end else  if (OpCode == LBs && mood==HIGH )begin 

                    sig_alu_op = ALU_LBs; //

                end		
			 else if  (OpCode == LBu && mood==LOW )begin 

                    sig_alu_op = ALU_Add; //

                end
			////////////////////////////////////   
			
			//Branch
				else if ( OpCode == BEQ && mood==LOW) begin

				sig_alu_op =ALU_BEQ ; // add	

                end	 
				
				else if ( OpCode == BEQZ && mood==HIGH) begin

				sig_alu_op =ALU_BEQZ ; // add	

                end
				
				else if ( OpCode == BGT && mood ==LOW ) begin

				sig_alu_op = ALU_BGT; // add	

                end	
				
				else if ( OpCode == BGTZ && mood ==HIGH ) begin

				sig_alu_op = ALU_BGTZ; // add	

                end	
				
				else if ( OpCode == BLT && mood ==LOW ) begin

				sig_alu_op = ALU_BLT; // add	

                end
				
				else if ( OpCode == BLTZ && mood ==HIGH ) begin

				sig_alu_op = ALU_BLTZ; 	

                end
				
				
		    	else if ( OpCode == BNEZ && mood ==HIGH ) begin

				sig_alu_op = ALU_BNEZ;	

                end
				
			  else if ( OpCode == BNE && mood ==LOW ) begin

				sig_alu_op = ALU_BNE; 	
			  end
			  
			  
			  sig_data_src= (OpCode==Sv)? HIGH:LOW;
			  sig_address_src= (OpCode == Sv) ? HIGH : LOW;



            end

            
			`STG_MEM: begin   	
				
				
	         // disable previous stage
                en_execute = LOW;
                
                 // next stage
                next_stage <= ( OpCode == LW || OpCode == LBs) ? `STG_WRB : `STG_FTCH;

                // ---------------------------------------------
                // ------------ set control signals ------------
                // ---------------------------------------------

                // ---- Memory Write ----

                sig_enable_data_memory_write = (OpCode == SW || OpCode == Sv ) ? HIGH : LOW;

                // ---- Memory Read ----
				
				if (OpCode == LW)begin
                sig_enable_data_memory_read = 2'b01;
			end else if  (OpCode == LBs && mood == HIGH)begin
			sig_enable_data_memory_read = 2'b11;
			
			end	else if  (OpCode == LBu && mood == LOW)begin 
				sig_enable_data_memory_read = 2'b10;
			end else begin
				  sig_enable_data_memory_read=2'b00;
				  
				  end
            end


            `STG_WRB: begin  
				
		    // disable previous stages
                sig_enable_data_memory_write = LOW;
                sig_enable_data_memory_read = 2'b00;
			//  sig_enable_data_memory_read_byte =LOW;
                en_execute = LOW;                
				
				
				
                 // next stage
                next_stage <= `STG_FTCH;

                // ---------------------------------------------
                // ------------ set control signals ------------
                // ---------------------------------------------
                
                // register file write enable
                sig_rf_enable_write = HIGH;

                // write back data source
                sig_write_back_data_select = ( OpCode == LW || OpCode == LBs) ? HIGH : LOW;

            end
      endcase

    end
	
	
	
endmodule  

 //////////////////////////////////////////////////////////////






   module controlUnitTestBench ();

    // ----------------- CLOCK -----------------
    
    // clock generator wires/registers
    reg clock;
    initial clock = 0;
    always #5 clock = ~clock; // generates clock square wave with 10ns period

    // ----------------- Inputs -----------------


    // op code
    reg [3:0] OpCode;

    // zero flag
    reg flag_zero,flag_overflow,flag_negative;

    // mood signal for lw and branch instructions
    reg mood;

    // ----------------- Signals -----------------

    // multi-bit mux control signals
    wire [3:0] sig_alu_op;
    wire [1:0] sig_pc_src;
    wire  sig_alu_src;

    // single bit mux control signals
    wire sig_write_back_data_select;
    wire sig_rb_src;
    wire sig_rsORpc_src;
    wire sig_rs1_src;
    wire sig_rs2_src;

    // operation enable signals
    wire sig_rf_enable_write;
    wire sig_enable_data_memory_write;
    wire [1:0] sig_enable_data_memory_read;

    // stage enable signals ( used as clock for each stage )
    wire en_instruction_fetch;
    wire en_instruction_decode;
    wire en_execute;

    controlUnit control_unit (
        .clock(clock),
        
        // signal outputs
        .sig_alu_op(sig_alu_op),
        .sig_pc_src(sig_pc_src),
        .sig_rb_src(sig_rb_src),
        .sig_rsORpc_src(sig_rsORpc_src),
        .sig_rs1_src(sig_rs1_src),
        .sig_rs2_src(sig_rs2_src),
        .sig_alu_src(sig_alu_src),
        .sig_rf_enable_write(sig_rf_enable_write),
        .sig_enable_data_memory_write(sig_enable_data_memory_write),
        .sig_enable_data_memory_read(sig_enable_data_memory_read),
        .sig_write_back_data_select(sig_write_back_data_select),
		.sig_address_src(sig_address_src),
		.sig_data_src(sig_data_src),
        // stage enable outputs
        .en_instruction_fetch(en_instruction_fetch),
        .en_instruction_decode(en_instruction_decode),
        .en_execute(en_execute),

        // inputs
        .OpCode(OpCode),
        .flag_zero(flag_zero),
		.flag_negative(flag_negative),
		.flag_overflow (flag_overflow ),
        .mood(mood),
		.sig_BZ(sig_BZ)
    );

    // ----------------- Simulation -----------------
    initial begin
        $dumpfile("controlUnitTestBench.vcd");
        $dumpvars(0, controlUnitTestBench);

        // Initialize inputs
        OpCode = 4'b0000;
        flag_zero = 0;
        mood = 0;

        // Test R-Type instruction (AND)
        #0 OpCode = 4'b0001; flag_zero = 0; mood = 0; // AND
        #40;

        // Test I-Type instruction (SUB)
        #0 OpCode = 4'b0010; flag_zero = 0; mood = 0; // ADDI
        #40;

        // Test I-Type instruction (ADDI)
        #0  OpCode = 4'b0011; flag_zero = 0; mood = 0; // ANDI
        #40;

        // Test I-Type instruction (ANDI)
        #0  OpCode = 4'b0100; flag_zero = 0; mood = 0; // LW
        #40;

        // Test I-Type instruction (LW)
        #0 OpCode = 4'b0101; flag_zero = 0; mood = 0; // SW
        #50;

        // Test J-Type instruction (LBu)
        #0  OpCode = 4'b0110;  mood = 0; // JMP
        #50; 
		  // Test J-Type instruction (LBs)
        #0 OpCode = 4'b0110;  mood = 1; // JMP
        #50;

        // Test Branch instruction (SW) 
        #0  OpCode = 4'b0111;  mood = 0; // BEQ not taken
        #40;

        // Test Branch instruction (BEQZ) taken
        #0  OpCode = 4'b1000; flag_zero = 1; mood = 0; // BEQ taken
        #30;

        // Test I-Type instruction () with mood HIGH
        #0  OpCode = 4'b1000; flag_zero = 0; mood = 1; // LBs
        #35;

        // Test I-Type instruction () with mood LOW
        #0  OpCode = 4'b1001; flag_zero = 0; mood = 0; // LBu
        #50;

        // Test Branch instruction (BNE) not taken
        #0  OpCode = 4'b1010; flag_zero = 1; mood = 0; // BNE not taken
        #30;

        // Test Branch instruction (BNE) taken
        #0  OpCode = 4'b1010; flag_zero = 0; mood = 0; // BNE taken
        #45;
		
		      // Test I-Type instruction (LBs) with mood HIGH
        #0 OpCode = 4'b1000; flag_zero = 0; mood = 1; // LBs
        #100;

        #10 $finish;
    end

endmodule







 //   ALU_Add = 3'b00, // used in ADD, ADDI, LW, SW, JAL
  //  ALU_Sub = 3'b01, // used in SUB, CMP, BEQ
   //	 ALU_And = 3'b10, // used in AND, ANDI






module ALU(
    input wire clock,
    input wire [3:0] sig_alu_op,
    input wire [15:0] A, B,
    output reg [15:0] Output,
    output reg flag_zero,
    output reg flag_negative,
    output reg flag_overflow
);		 


           assign flag_zero = (0 == Output);
	      assign flag_negative = (Output[15] == 1); // if 2s complement number is negative, MSB is 1

           //flag_zero <= (Output == 0);
         // flag_negative <= (Output[15] == 1);

    always @(posedge clock) begin
        case (sig_alu_op)
            ALU_Add, ALU_LBs: begin
                {flag_overflow, Output} <= {A[15], A} + {B[15], B};
            end
            ALU_Sub: begin
                {flag_overflow, Output} <= {A[15], A} - {B[15], B};
            end
            ALU_And: begin
                Output <= A & B;
                flag_overflow <= 0;
            end
            ALU_BGT, ALU_BGTZ, ALU_BLT, ALU_BLTZ, ALU_BEQ, ALU_BEQZ, ALU_BNEZ, ALU_BNE: begin
                {flag_overflow, Output} <= {B[15], B} - {A[15], A} ;
            end
            default: begin
                Output <= 0;
                flag_overflow <= 0;
				
            end
        endcase

        // Update flags immediately after computing the Output
      //  flag_zero <= (Output == 0);
     //   flag_negative <= (Output[15] == 1);
    end

endmodule







module ALUTestBench();

    // ----------------- CLOCK -----------------
    
    // Clock generator wires/registers
    reg clock;
    initial clock = 0;
    always #5 clock = ~clock; // Generates clock square wave with 10ns period

    // ----------------- ALU -----------------

    reg [15:0] A, B; // Operands
    wire [15:0] Output;
    wire flag_zero;
    wire flag_negative;
    wire flag_overflow;

    // Signals
    reg [3:0] sig_alu_op;

    // Define opcodes for ALU operations

    // Instantiate the ALU module
    ALU alu(
        .clock(clock),
        .A(A),
        .B(B),
        .Output(Output),
        .flag_zero(flag_zero),
        .flag_negative(flag_negative),
        .flag_overflow(flag_overflow),
        .sig_alu_op(sig_alu_op)
    );

    // ----------------- Simulation -----------------

    initial begin
        // Initial values
        A = 16'd0;
        B = 16'd0;
        sig_alu_op = ALU_Add;

        // Test addition
        #100;
        A = 16'd10;
        B = 16'd20;
        sig_alu_op = ALU_Add;
        #100;
        $display("(%0t) Addition: A=%0d + B=%0d => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, B, Output, flag_zero, flag_negative, flag_overflow);

        // Test subtraction
        #100;
        A = 16'd30;
        B = 16'd20;
        sig_alu_op = ALU_Sub;
        #100;
        $display("(%0t) Subtraction: A=%0d - B=%0d => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, B, Output, flag_zero, flag_negative, flag_overflow);

        // Test AND operation
        #100;
        A = 16'h0F0F;
        B = 16'h00FF;
        sig_alu_op = ALU_And;
        #100;
        $display("(%0t) AND: A=%0h & B=%0h => Output=%0h | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, B, Output, flag_zero, flag_negative, flag_overflow);

        // Test zero flag
        #100;
        A = 16'd50;
        B = 16'd50;
        sig_alu_op = ALU_Sub;
        #100;
        $display("(%0t) Zero Flag Test: A=%0d - B=%0d => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, B, Output, flag_zero, flag_negative, flag_overflow);

        // Test negative flag
        #100;
        A = 16'd50;
        B = 16'd100;
        sig_alu_op = ALU_Sub;
        #100;
        $display("(%0t) Negative Flag Test: A=%0d - B=%0d => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, B, Output, flag_zero, flag_negative, flag_overflow);

        // Test BGT operation (A - B > 0)
        #100;
        A = 16'd10;
        B = 16'd5;
        sig_alu_op = ALU_BGT;
        #100;
        $display("(%0t) BGT: A=%0d - B=%0d => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, B, Output, flag_zero, flag_negative, flag_overflow);

        // Test BGTZ operation (A - 0 > 0)
        #100;
        A = 16'd10;
        B = 16'd0;
        sig_alu_op = ALU_BGTZ;
        #100;
        $display("(%0t) BGTZ: A=%0d - 0 => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, Output, flag_zero, flag_negative, flag_overflow);

        // Test BLT operation (A - B < 0)
        #100;
        A = 16'd10;
        B = 16'd5;
        sig_alu_op = ALU_BLT;
        #100;
        $display("(%0t) BLT: A=%0d - B=%0d => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, B, Output, flag_zero, flag_negative, flag_overflow);

        // Test BLTZ operation (A < 0)
        #100;
        A = -16'd100;
        B = 16'd0;
        sig_alu_op = ALU_BLTZ;
        #100;
        $display("(%0t) BLTZ: A=%0d - 0 => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, Output, flag_zero, flag_negative, flag_overflow);

        // Test BEQ operation (A - B == 0)
        #100;
        A = 16'd10;
        B = 16'd10;
        sig_alu_op = ALU_BEQ;
        #100;
        $display("(%0t) BEQ: A=%0d - B=%0d => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, B, Output, flag_zero, flag_negative, flag_overflow);

        // Test BNEQ operation (A - B != 0)
        #100;
        A = 16'd10;
        B = 16'd5;
        sig_alu_op = ALU_BNE;
        #100;
        $display("(%0t) BNEQ: A=%0d - B=%0d => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, B, Output, flag_zero, flag_negative, flag_overflow); 

        // Test BEQZ operation (A == 0)
        #100;
        A = 16'd0;
        B = 16'd0;
        sig_alu_op = ALU_BEQZ;
        #100;
        $display("(%0t) BEQZ: A=%0d - 0 => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, Output, flag_zero, flag_negative, flag_overflow);

        // Test BNEZ operation (A != 0)
        #100;
        A = 16'd10;
        B = 16'd0;
        sig_alu_op = ALU_BNEZ;
        #100;
        $display("(%0t) BNEZ: A=%0d - 0 => Output=%0d | zero_flag=%0b | negative_flag=%0b | overflow_flag=%0b", $time, A, Output, flag_zero, flag_negative, flag_overflow);

        // Finish simulation
        #100;
        $finish;
    end

endmodule





 module dataMemory(
    input wire clock,
    input wire [15:0] AddressBus,
    input wire [15:0] InputBus,
    output reg [15:0] OutputBus,
    input wire sig_enable_write,
    input wire [1:0] sig_enable_read
);

    // ----------------- SIGNALS -----------------

    // memory
    reg [15:0] memory [0:255];

    // ----------------- LOGIC -----------------

    // Read/write instruction at positive edge of clock
    always @(posedge sig_enable_read or posedge sig_enable_write or clock) begin
        if (sig_enable_read == 2'b01) begin
            // Read word from memory
            OutputBus <= memory[AddressBus];
        end
        else if (sig_enable_read == 2'b10) begin
            // Read byte from memory and zero-extend
            OutputBus <= {8'b0, memory[AddressBus][7:0]};
        end
        else if (sig_enable_read == 2'b11) begin
            // Read byte from memory and sign-extend
            OutputBus <= {{8{memory[AddressBus][15]}}, memory[AddressBus][7:0]};
        end
        else if (sig_enable_write) begin
            // Write word to memory
            memory[AddressBus] <= InputBus;
        end
    end

    // ----------------- INITIALIZATION -----------------

    initial begin
        // Store some initial data
         memory[0] = 16'd9;
         memory[1] = 16'd4;
		 memory[2] = 16'd2;
		 memory[3] = 16'd3;
		 memory[5] = 16'd5;
		 memory[6]=	16'd6;
		 memory[30] = 16'd4;
		 memory[31] = 16'hF004;
    end

endmodule









  


module dataMemoryTestBench ();

    // ----------------- CLOCK -----------------
    
    // clock generator wires/registers
    reg clock;
    initial clock = 0;
    always #5 clock = ~clock; // generates clock square wave with 10ns period

    // ----------------- DATA MEMORY -----------------

    reg [15:0] AddressBus;
    reg [15:0] InputBus;
    wire [15:0] OutputBus;
    reg sig_enable_write;
    reg [1:0] sig_enable_read;
    //reg [1:0]sig_enable_read_byte; // New signal for byte read enable

    // Instantiate the data memory module
    dataMemory data_mem(
        .clock(clock),
        .AddressBus(AddressBus),
        .InputBus(InputBus),
        .OutputBus(OutputBus),
        .sig_enable_write(sig_enable_write),
        .sig_enable_read(sig_enable_read)
      //  .sig_enable_read_byte(sig_enable_read_byte) // Connect the new signal
    );

    // ----------------- Simulation -----------------

    initial begin
        // Initial values
        AddressBus = 16'b0;
        InputBus = 16'b0;
        sig_enable_write = 0;
        sig_enable_read = 0;
     //   sig_enable_read_byte = 2'b00; // Initialize byte read enable to 0

        // Write 9 to address 0
        #10;
        AddressBus = 16'd0;
        InputBus = 16'd9;
        sig_enable_write = 1;
        sig_enable_read = 0;
		//sig_enable_read_byte= 2'b00;
        #100;

        // Write 4 to address 1
        #10;
        AddressBus = 16'd1;
        InputBus = 16'h0FF4;
        sig_enable_write = 1;
        sig_enable_read = 0;
		//sig_enable_read_byte= 2'b00;
        #100;

        // Read from address 0
        #10;
        AddressBus = 16'd0;
        sig_enable_write = 0;
        sig_enable_read =  2'b01;
	//	sig_enable_read_byte= 2'b00;
        #100;
        $display("(%0t) Read from address 0: %0d", $time, OutputBus);

        // Read from address 1
        #10;
        AddressBus = 16'd1;
        sig_enable_write = 0;
        sig_enable_read =  2'b01;
		//sig_enable_read_byte= 2'b00;
        #100;
        $display("(%0t) Read from address 1: %0d", $time, OutputBus);

        // Read sign extention byte from address 0
        #10;
        AddressBus = 16'd0;
        sig_enable_write = 0;
        sig_enable_read =  2'b10;
     //   sig_enable_read_byte =  2'b11; // Enable byte read
        #100;
        $display("(%0t) Read sign extented byte from address 0: %0h", $time, OutputBus);

        // Read zero extention byte from address 1
        #100;
        AddressBus = 16'd1;
        sig_enable_write = 0;
        sig_enable_read =  2'b11;
      //  sig_enable_read_byte = 2'b11; // Enable byte read
        #100;
        $display("(%0t) Read zero extented byte from address 1: %0h", $time, OutputBus);

        // Finish simulation
        #10;
        $finish;
    end

endmodule 
	   







// register file (array of 16-bit 32 registers)
module registerFile (
    input wire clock,
    input wire sig_enable_write,
	input wire  sig_BZ,
    input wire [2:0] RA, RB, RW,
    output reg [15:0] BusA, BusB,
    input wire [15:0] BusW
);

    // ----------------- INTERNALS -----------------

    reg [15:0] registers_array [0:31];

    // read registers always
    always @(posedge clock) begin 
		if (sig_BZ==HIGH)begin
		BusA=registers_array[0];
		BusB = registers_array[RB];
		end 
		else begin
        BusA = registers_array[RA];
        BusB = registers_array[RB];	
		end 
    end

    // write to register on positive edge of sig_enable_write
    always @(posedge clock) begin
        if (sig_enable_write) begin
            if (RW != 3'b0) begin // write register is not R0
                registers_array[RW] = BusW;
            end
        end
    end

    // initialize registers to 0
    initial begin
        registers_array[0] <= 16'h0000;
        registers_array[1] <= 16'h0001;
        registers_array[2] <= 16'h0002;
        registers_array[3] <= 16'h0003;
        registers_array[4] <= 16'h0004;
        registers_array[5] <= 16'h0005;
        registers_array[6] <= 16'h0110;
        registers_array[7] <= -16'h000F;
		registers_array[8] <= 16'h0000;
		registers_array[9] <= 16'h0000;
		registers_array[10] <= 16'h0000;
		registers_array[11] <= 16'h0000;
		registers_array[12] <= 16'h0000;
		registers_array[13] <= 16'h0000;
		registers_array[14] <= 16'h0000;
		registers_array[15] <= 16'h0000;
		registers_array[16] <= 16'h0000;
		registers_array[17] <= 16'h0000;
		registers_array[18] <= 16'h0000;
		registers_array[19] <= 16'h0000;
		registers_array[20] <= 16'h0000;
		registers_array[21] <= 16'h0000;
		registers_array[22] <= 16'h0000;
		registers_array[23] <= 16'h0000;
		registers_array[24] <= 16'h0000;
		registers_array[25] <= 16'h0000;
		registers_array[26] <= 16'h0000;
		registers_array[27] <= 16'h0000;
		registers_array[28] <= 16'h0000;
		registers_array[29] <= 16'h0000;
		registers_array[30] <= 16'h0100;
		registers_array[31] <= 16'h0100;
	 

    end

endmodule  

module registerFileTestBench();

    // ----------------- CLOCK -----------------

    // Clock signal
    reg clock;

    // Generate clock signal
    initial begin
        clock = 0;
        forever #5 clock = ~clock; // Toggle clock every 5 time units
    end

    // ----------------- REGISTER FILE -----------------

    // Inputs
    reg sig_enable_write;
    reg [3:0] RA, RB, RW;
    reg [15:0] BusW;

    // Outputs
    wire [15:0] BusA, BusB;

    // Instantiate the register file
    registerFile uut (
        .clock(clock),
        .sig_enable_write(sig_enable_write), 
		.sig_BZ(sig_BZ),
        .RA(RA),
        .RB(RB),
        .RW(RW),
        .BusW(BusW),
        .BusA(BusA),
        .BusB(BusB)
    );

    // ----------------- TEST SEQUENCE -----------------

    initial begin
        // Initialize signals
        sig_enable_write = 0;
        RA = 3'd0;
        RB = 3'd0;
        RW = 3'd0;
        BusW = 16'd0;

        // Wait for reset
        #10;

        // Write 15 to register 1
        RW = 3'd1;
        BusW = 16'd15;
        sig_enable_write = 1;
        #10;
        sig_enable_write = 0;

        // Write 30 to register 2
        RW = 3'd2;
        BusW = 16'd30;
        sig_enable_write = 1;
        #10;
        sig_enable_write = 0;

        // Read from register 1 and 2
        RA = 3'd1;
        RB = 3'd2;
        #10;

        // Display the results
        $display("Register 1: %d", BusA);
        $display("Register 2: %d", BusB);

        // Write 45 to register 3
        RW = 3'd3;
        BusW = 16'd45;
        sig_enable_write = 1;
        #10;
        sig_enable_write = 0;

        // Read from register 3
        RA = 3'd3;
        #10;

        // Display the result
        $display("Register 3: %d", BusA);

        // End the simulation
        #10;
        $finish;
    end

endmodule	



// stores program instructions ( used 16 bit cells )



module instructionMemory(clock, AddressBus, InstructionReg);


    // clock
    input wire clock;

    // address bus
    input wire [15:0] AddressBus;


    // instruction register
    output reg [0:15] InstructionReg;


    // instruction memory
    reg [15:0] instruction_memory [0:127];


    // ----------------- LOGIC -----------------
					   
    assign InstructionReg = instruction_memory[AddressBus[15:0]]; 


    initial begin
        // load instructions from file
        // $readmemh("instructions.txt", instruction_memory);

        
        // instruction formts:

            // R-Type Instruction Format           
            //  Opcode4 Rd3 Rs13 Rs23 Unused3

            // I-Type Instruction Format            
            // Opcode4 m1 Rd3 Rs13 Immediate5


        // initial
        instruction_memory[0] = 16'b0;


        // ----------------- Addition Loop Program -----------------
	//	instruction_memory[1] = { ADDI, 1'b0,3'b001, 3'b010, 5'b00101 };  // Load immediate value 5 into R2	   
	 
	instruction_memory[2] = 16'b0;
	//{ ADD, R1, R0, R2,3'b001};  // Add R0 and R2, store result in R1
	//instruction_memory[3] = { ADDI,1'b1 ,R0, R0, 5'b00101 };  // Increment R0 by 1 
	
	
	
	
	
	//--------------------R-type -test --------------------------------//
		
		//instruction_memory[4] = { ADD, R1, R3, R2,3'b001};
		//instruction_memory[6] = { SUB, R4, R3, R2,3'b001};
		//instruction_memory[8] = { AND, R5, R3, R2,3'b001};

		
		
		
		
		
		
		////----------Test load and store  -----------------------------------------//
		
	
		


		
		instruction_memory[4] = { LW,1'b1 ,R1, R2, 5'b00001 };
		instruction_memory[6]=  { SW,1'b1 ,R7, R2, 5'b0000}; 
		instruction_memory[8]=  { LBs,1'b1 ,R1, R2, 5'b00010 };
		instruction_memory[6]=  { SW,1'b1 ,R0, R2, 5'b00010 }; 
		instruction_memory[10] = { LW,1'b1 ,R0, R1, 5'b00000 };	 
		
		
				
		
		// --------Branch Test-------------------//	
		
		
		//BGT,BGTZ  ZERO & NEGATVE =0
		//BLT,BLTZ NEGATIVE =1
		//BEQ,BEQZ ZERO =1
		//BNE.BNEZ ZERO=0
		/*instruction_memory[4] = { BLT,1'b0 ,R2, R3, 5'b00001 };//Negative flag = 1
		instruction_memory[5] = { BLTZ,1'b1 ,R7, R0, 5'b0001 };//Negative flag = 1
		instruction_memory[6] = { BGT,1'b0 ,R2, R1, 5'b00001 };	//Negative flag = 0 zero flag =0 
		instruction_memory[7] = { BGTZ,1'b1 ,R6, R0, 5'b00010 }; //Negative flag = 0 zero flag =0 
		instruction_memory[9] = { BEQ,1'b0 ,R1, R1, 5'b00011 }; 
		instruction_memory[12] = { BEQZ,1'b1 ,R5, R1, 5'b00001 };
		instruction_memory[13] = { BNE,1'b0 ,R1, R1, 5'b00001 };
		instruction_memory[15] = { BNEZ,1'b1 ,R1, R0, 5'b11110 };  */
		
	//-------------------------------------------------------------------------------------	
		
			  //-------------------S-type test----------------------------//
	
	//instruction_memory[4]=  { Sv ,R5, 9'b000000011};
	//instruction_memory[6] = { LW,1'b0 ,R1, R4, 5'b00001 }; // check	value BUSW Or value Data output must be 5
	
	
		
///----------------------------------------------------------------------------------------

// ------------------------j-type test-----------------------------------------//

/*instruction_memory[4]= { JMP ,12'b000001100};  
instruction_memory[12]= { CALL ,12'b000001111};	
instruction_memory[14] = { LW,1'b0 ,R1, R4, 5'b00001 };
instruction_memory[15]= { RET ,12'b000001111};
												 */
		
		

        // ----------------- Shift Loop Program -----------------

        
        // ----------------- Branch Program -----------------

        


    end


endmodule



// PC register and its logic

// PC src can be one of the following:
// 1- PC + 2
// 2- Branch target address (if instruction is I-Type & function is BEQor  and zero flag is 1)
// 3- Jump target address (if instruction is J-Type: J, JAL)
// 4- link return address (if stop bit is set)
// 5- PC = PC + sign_extended (Imm)

module pcModule(clock, PC, I_TypeImmediate, J_TypeImmediate, ReturnAddress,sig_pc_src);

    // ----------------- INPUTS -----------------

    // Clock signal
    input wire clock;
    
    // PC source control signal
    input wire [1:0] sig_pc_src; 

    // Return address from the stack
    input wire [15:0] ReturnAddress;

    // Immediate value for I-Type instructions
    input wire [15:0] I_TypeImmediate;

    // Immediate value for J-Type instructions
    input wire signed [11:0] J_TypeImmediate;

    // Immediate value to be sign-extended
   // input wire signed [15:0] Imm;

    // ----------------- OUTPUTS -----------------

    // Program Counter
    output reg [15:0] PC;

    // ----------------- INTERNALS -----------------

    // Incremented PC by 2
    wire [15:0] pc_plus_2;

    // Jump target address
    wire [15:0] jump_target_address;

    // Branch target address
    wire [15:0] brach_target_address;
    
    // Sign-extended immediate
    wire [15:0] sign_extended_imm;

    // Calculate PC + 2
    assign pc_plus_2 = PC + 16'd2;

    // Calculate jump target address
    assign jump_target_address = {{PC[15:12]},J_TypeImmediate};

    // Calculate branch target address
    assign brach_target_address =PC +I_TypeImmediate;

    // Calculate sign-extended immediate target address
    assign sign_extended_imm = PC + I_TypeImmediate;

    // Initialize PC to 0 at the beginning
    initial begin
        PC <= 16'd0;
    end

    // Update PC on the positive edge of the clock
    always @(posedge clock) begin
        case (sig_pc_src)
            2'b00: begin
                // Default: PC = PC + 2
                PC = pc_plus_2;      
            end  
            2'b01: begin
                // Return address: PC = ReturnAddress
                PC = ReturnAddress;  
            end  
            2'b10: begin
                // Branch target address: PC = PC + I_TypeImmediate
                PC = brach_target_address ;
            end  
            2'b11: begin
                // Jump target address: PC = PC + J_TypeImmediate
                PC = jump_target_address;
            end  
         //   3'b100: begin
                // Sign-extended immediate: PC = PC + Imm
           //     PC = sign_extended_imm;
           // end
        endcase
    end

endmodule 

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 module riscProcessor();

    initial begin		 
		#0
        $display("(%0t) > initializing processor ...", $time);

        #400 $finish;
    end

    // -----------------------------------------------------------------
    // ----------------------------- Wires -----------------------------
    // -----------------------------------------------------------------

    // clock generator wires/registers
	wire clock;

    // ----------------- Control Unit -----------------

    // multi-bit mux control signals
    wire [3:0] sig_alu_op;
    wire [1:0] sig_pc_src;
	//wire [1:0] sig_rs2_src;
    wire  sig_alu_src;

    // single bit mux control signals
    wire sig_write_back_data_select,
            sig_rb_src,sig_rs1_src, sig_rs2_src,sig_rsORpc_src,sig_BZ;

    // operation enable signals
    wire sig_rf_enable_write,
            sig_enable_data_memory_write;
    wire [1:0] sig_enable_data_memory_read;       

    // stage enable signals ( used as clock for each stage )
    wire en_instruction_fetch,
            en_instruction_decode,
            en_execute;

    // ----------------- Instrution Memory -----------------

    // instruction memory wires/registers		
    wire [15:0] PC; // output of PC Module input to instruction memory
    wire [15:0] InstructionReg; // output if instruction memory, input to other modules
	 // Immediate value to be sign-extended
     wire signed [4:0] Imm;
    // Instruction Parts
    wire [3:0] OpCode; // function code

    // R-Type
    wire [2:0] Rs1_Rtype, Rd_Rtype, Rs2; // register selection	
	//I-type
	wire [2:0] Rs1_Itype, Rd_Itype; // register selection
    wire [1:0] InstructionType; // instruction type [ R, S, I, J ]
    wire mood; // mood bit signed or unsignd extends	
	
	//Memory Write
	wire sig_address_src,
	     sig_data_src;

    // ----------------- Assignment -----------------

    // Function Code
    assign OpCode = InstructionReg[15:12];

    // R-Type
    assign Rs1_Rtype = InstructionReg[8:6];
    assign Rd_Rtype = InstructionReg[11:9];
    assign Rs2 = InstructionReg[5:3];
	
	
	    // I-Type
    assign Rs1_Itype = InstructionReg[7:5];
    assign Rd_Itype = InstructionReg[10:8];
	assign mood = InstructionReg[11]; 
	wire signed [4:0] I_TypeImmediate;
	assign I_TypeImmediate = InstructionReg[4:0]; 
    
    // J-Type
    wire signed [11:0] J_TypeImmediate;
    assign J_TypeImmediate = InstructionReg[11:0];
	
    wire signed [8:0] S_TypeImmediate;
    assign S_TypeImmediate = InstructionReg[8:0];


    // S-Type
    wire [0:4] SA;
	
   // assign SA = InstructionReg[20:24];

    // Instruction Type and Stop Bit
   // assign InstructionType = InstructionReg[29:30];
  //  assign StopBit = InstructionReg[31];


    // ----------------- PC Modules -----------------

    // register file wires/registers
    reg [15:0] ReturnAddress;  // input to PC Module from TODO
    wire signed [15:0] Sign_Extended_J_TypeImmediate, Sign_Extended_I_TypeImmediate ;  // input to PC Module from decode stage
    wire [15:0] Unsigned_Extended_I_TypeImmediate, Unsigned_Extended_SA,Unsigned_Extended_S_TypeImmediate; // input to ALU Module from decode stage

    // signed extender for J-Type instructions immediate ( 24 bit to 32 )
    assign Sign_Extended_J_TypeImmediate = { {8{J_TypeImmediate[0]}}, J_TypeImmediate }; 
	
	assign Unsigned_Extended_S_TypeImmediate = { {7{1'b0}}, S_TypeImmediate };

    // signed extender for I-Type instructions immediate ( 5 bit to 16 )
    assign Sign_Extended_I_TypeImmediate = { {11{I_TypeImmediate[4]}}, I_TypeImmediate };

    // unsigned extender for I-Type instructions immediate ( 5 bit to 16 )
    assign Unsigned_Extended_I_TypeImmediate = { {11{1'b0}}, I_TypeImmediate };

    // unsigned extender for S-Type instructions immediate ( 5 bit to 32 )
  //  assign Unsigned_Extended_SA = { {27{1'b0}}, SA };
	
	
    // ----------------- Register File -----------------
    
    reg [15:0] BusW; // TODO
    wire [15:0] BusA, BusB;

    wire [2:0] RA, RB, RW;
	////////////////////////
    assign RA = (sig_rs1_src==HIGH && OpCode ==RET) ? R7:  
	   (sig_rs1_src==LOW) ? Rs1_Rtype:Rs1_Itype;
    assign RB = (sig_rs2_src == LOW && OpCode == Sv) ? Rd_Rtype : 
	(sig_rs2_src == LOW) ? Rs2 : Rd_Itype; 
	
	
    assign RW = ( OpCode == CALL) ? R7:
	            (sig_rb_src==LOW) ? Rd_Rtype: Rd_Itype;
	
  //  assign BusW = (sig_rf_enable_write == HIGH && OpCode == CALL) ? (PC + 16'd2) : BusW ; // other value or previous value;

	////////////////////////

    // ----------------- ALU -----------------

    reg [15:0] ALU_A, ALU_B; // operands
    wire [15:0] ALU_Output;
    wire flag_zero;
    wire flag_negative;
	wire flag_overflow;

    assign ALU_A = BusA; 
	assign ALU_B =  (sig_alu_src==LOW) ? BusB:Unsigned_Extended_I_TypeImmediate;	
 
	

    // ----------------- Data Memory -----------------

    // data memory wires/registers/signals
    wire [15:0] DataMemoryAddressBus;
    wire [15:0] DataMemoryInputBus;

    wire [15:0] DataMemoryOutputBus;

    assign DataMemoryAddressBus = (sig_address_src==HIGH)? RB:ALU_Output;
    assign DataMemoryInputBus = (sig_data_src==HIGH)? Unsigned_Extended_S_TypeImmediate:BusB;

    // -----------------------------------------------------------------
    // ----------------------------- CLOCK -----------------------------
    // -----------------------------------------------------------------
    
    
	// generates clock square wave with 10ns period
	ClockGenerator clock_generator(clock);

    // -----------------------------------------------------------------
    // ----------------- Control Unit -----------------
    // -----------------------------------------------------------------
    
	
	  controlUnit control_unit (
        .clock(clock),
        
        // signal outputs
        .sig_alu_op(sig_alu_op),
        .sig_pc_src(sig_pc_src),
        .sig_rb_src(sig_rb_src),
        .sig_rsORpc_src(sig_rsORpc_src),
        .sig_rs1_src(sig_rs1_src),
        .sig_rs2_src(sig_rs2_src),
        .sig_alu_src(sig_alu_src),
        .sig_rf_enable_write(sig_rf_enable_write),
        .sig_enable_data_memory_write(sig_enable_data_memory_write),
        .sig_enable_data_memory_read(sig_enable_data_memory_read),
        .sig_write_back_data_select(sig_write_back_data_select),
		.sig_address_src(sig_address_src),
		.sig_data_src(sig_data_src),
		
        // stage enable outputs
        .en_instruction_fetch(en_instruction_fetch),
        .en_instruction_decode(en_instruction_decode),
        .en_execute(en_execute),

        // inputs
        .OpCode(OpCode),
        .flag_zero(flag_zero),
		.flag_negative(flag_negative),
		.flag_overflow (flag_overflow ),
        .mood(mood),	
		.sig_BZ(sig_BZ)
    );
			


    // -----------------------------------------------------------------
    // ----------------- Instruction Memory -----------------
    // -----------------------------------------------------------------
    
    // use en_instruction_fetch as clock for instruction memory
    instructionMemory instruction_memory(clock, PC, InstructionReg);

    // -----------------------------------------------------------------
    // ----------------- PC Module -----------------
    // -----------------------------------------------------------------
	//clock, PC, I_TypeImmediate, J_TypeImmediate, ReturnAddress, Imm, sig_pc_src	
	

     pcModule pc_module(en_instruction_fetch, PC, Sign_Extended_I_TypeImmediate,J_TypeImmediate, BusA,sig_pc_src);
	
    // -----------------------------------------------------------------
    // ----------------- Register File -----------------
    // -----------------------------------------------------------------


    // register file ( array of 16 registers )
    // use en_instruction_decode as clock for register file
    registerFile register_file(en_instruction_decode,sig_rf_enable_write,sig_BZ,RA, RB, RW, BusA, BusB,BusW);

    // -----------------------------------------------------------------
    // ----------------- ALU -----------------
    // -----------------------------------------------------------------

   
	ALU alu(en_execute,sig_alu_op ,ALU_A, ALU_B,ALU_Output,flag_zero,flag_negative,flag_overflow);

    // -----------------------------------------------------------------
    // ----------------- Data Memory -----------------
    // -----------------------------------------------------------------

    dataMemory data_memory(clock, DataMemoryAddressBus, DataMemoryInputBus, DataMemoryOutputBus, sig_enable_data_memory_write, sig_enable_data_memory_read);
	
    // -----------------------------------------------------------------
    // ----------------- Write Back -----------------
    // -----------------------------------------------------------------

    assign BusW = (sig_write_back_data_select == 0 && OpCode == CALL) ?(PC + 16'd2) : 
	     (sig_write_back_data_select == 0 ) ? ALU_Output : DataMemoryOutputBus;


endmodule		 



// generates clock square wave with 10ns period

module ClockGenerator (
    clock
);

initial begin
    $display("(%0t) > initializing clock generator ...", $time);
end

output reg clock=0; // starting LOW is important for first instruction fetch

always #5 begin
    clock=~clock;
end
					   

endmodule







