
-- Project: MIPS32 multi-cycle
-- Module: Top level

library ieee;
use ieee.std_logic_1164.all;

entity multicycle is
  port( CLK, reset_neg : in std_logic );
end multicycle;

architecture Behavioral of multicycle is

component ALU is
  port( -- input
        operand_1   : in std_logic_vector(31 downto 0);
        operand_2   : in std_logic_vector(31 downto 0);
        ALU_control : in std_logic_vector(3 downto 0);  -- 12 operations

        -- output
        result      : out std_logic_vector(31 downto 0);
        zero        : out std_logic );
end component;

component ALUControl is
  port( -- input
        ALUOp  : in std_logic_vector(1 downto 0);
        instr  : in std_logic_vector(5 downto 0);

        -- output
        result : out std_logic_vector(3 downto 0) );
end component;

component ControlUnit is
  port( -- input
        CLK         : in std_logic;
        Reset       : in std_logic;
        Op          : in std_logic_vector(5 downto 0);

        -- output (control signals)
        PCWriteCond : out std_logic;
        PCWrite     : out std_logic;
        IorD        : out std_logic;
        MemRead     : out std_logic;
        MemWrite    : out std_logic;
        MemToReg    : out std_logic;
        IRWrite     : out std_logic;
        PCSource    : out std_logic_vector(1 downto 0);
        ALUOp       : out std_logic_vector(1 downto 0);
        ALUSrcB     : out std_logic_vector(1 downto 0);
        ALUSrcA     : out std_logic;
        RegWrite    : out std_logic;
        RegDst      : out std_logic );
end component;

component InstructionRegister is
  port( -- input
        CLK             : in std_logic;
        reset_neg       : in std_logic;
        IRWrite         : in std_logic;
        in_instruction  : in std_logic_vector(31 downto 0);

        -- output
        out_instruction : out std_logic_vector(31 downto 0) );
end component;

component Memory is
  port( -- inputs
        CLK       : in std_logic;
        reset_neg : in std_logic;
        address   : in std_logic_vector(31 downto 0);
        MemWrite  : in std_logic;
        MemRead   : in std_logic;
        WriteData : in std_logic_vector(31 downto 0);

        -- output
        MemData   : out std_logic_vector(31 downto 0) );
end component;

component MemoryDataRegister is
  port( -- inputs
        CLK       : in std_logic;
        reset_neg : in std_logic;
        input     : in std_logic_vector(31 downto 0);    -- from MemDataOut

        -- output
        output    : out std_logic_vector(31 downto 0) ); -- to mux
end component;

component Mux2 is
  port( -- input
        input_1     : in std_logic_vector(31 downto 0);
        input_2     : in std_logic_vector(31 downto 0);
        mux_select  : in std_logic;

        -- output
        output      : out std_logic_vector(31 downto 0) );
end component;

component Mux2_5 is
  port( -- input
        input_1     : in std_logic_vector(4 downto 0);
        input_2     : in std_logic_vector(4 downto 0);
        mux_select  : in std_logic;

        -- output
        output      : out std_logic_vector(4 downto 0) );
end component;

component Mux3 is
  port( -- input
        input_1     : in std_logic_vector(31 downto 0);
        input_2     : in std_logic_vector(31 downto 0);
        input_3     : in std_logic_vector(31 downto 0);
        mux_select  : in std_logic_vector(1 downto 0);

        -- output
        output      : out std_logic_vector(31 downto 0) );
end component;

component Mux4 is
  port( -- input
        input_1     : in std_logic_vector(31 downto 0);
        input_2     : in std_logic_vector(31 downto 0);
        input_3     : in std_logic_vector(31 downto 0);
        input_4     : in std_logic_vector(31 downto 0);
        mux_select  : in std_logic_vector(1 downto 0);

        -- output
        output      : out std_logic_vector(31 downto 0) );
end component;

component ProgramCounter is
  port( -- input
        CLK       : in  std_logic;
        reset_neg : in  std_logic;
        input     : in  std_logic_vector(31 downto 0);
        PCcontrol : in  std_logic;

        -- output
        output    : out std_logic_vector(31 downto 0) );
end component;

component Registers is
  port( -- input
        CLK          : in std_logic;
        reset_neg    : in std_logic;
        address_in_1 : in std_logic_vector(4 downto 0);
        address_in_2 : in std_logic_vector(4 downto 0);
        write_reg    : in std_logic_vector(4 downto 0);

        write_data   : in std_logic_vector(31 downto 0);
        RegWrite     : in std_logic;  -- signal control

        -- output
        register_1   : out std_logic_vector(31 downto 0);
        register_2   : out std_logic_vector(31 downto 0) );
end component;

component ShiftLeft is
  port( -- input
        input  : in std_logic_vector(31 downto 0);

        -- output
        output : out std_logic_vector(31 downto 0) );
end component;

component ShiftLeft2 is
  port( -- input
        input  : in std_logic_vector(25 downto 0);

        -- output
        output : out std_logic_vector(27 downto 0) );
end component;

component SignExtend is
  port( -- input
        input  : in std_logic_vector(15 downto 0);

        -- output
        output : out std_logic_vector(31 downto 0) );
end component;

component TempRegisters is
  port( -- input
        CLK         : in std_logic;
        reset_neg   : in std_logic;
        in_reg_A    : in std_logic_vector (31 downto 0);
        in_reg_B    : in std_logic_vector (31 downto 0);
        in_ALU_out  : in std_logic_vector (31 downto 0);

        -- output
        out_reg_A   : out std_logic_vector(31 downto 0);
        out_reg_B   : out std_logic_vector(31 downto 0);
        out_ALU_out : out std_logic_vector(31 downto 0) );
end component;

  constant PC_increment : std_logic_vector(31 downto 0) := "00000000000000000000000000000100";

-- signals
  signal PC_out, MuxToAddress, MemDataOut, MemoryDataRegOut, InstructionRegOut, MuxToWriteData, ReadData1ToA, ReadData2ToB, RegAToMux, RegBOut, SignExtendOut, ShiftLeft1ToMux4, MuxToAlu, Mux4ToAlu, AluResultOut, AluOutToMux, JumpAddress, MuxToPC : std_logic_vector(31 downto 0);
  signal ZeroCarry_TL, ALUSrcA_TL, RegWrite_TL, RegDst_TL, PCWriteCond_TL, PCWrite_TL, IorD_TL, MemRead_TL, MemWrite_TL, MemToReg_TL, IRWrite_TL, ANDtoOR, ORtoPC : std_logic;
  signal MuxToWriteRegister : std_logic_vector(4 downto 0);
  signal ALUControltoALU : std_logic_vector(3 downto 0);
  signal PCsource_TL, ALUSrcB_TL, ALUOp_TL : std_logic_vector(1 downto 0);

begin

  ANDtoOR <= ZeroCarry_TL and PCWriteCond_TL;
  ORtoPC <= ANDtoOR or PCWrite_TL;
  JumpAddress(31 downto 28) <= PC_out(31 downto 28);

  
 A_Logic_Unit : ALU  port map(
							  operand_1 => MuxToAlu, 
							  operand_2 => Mux4ToALU,
							  ALU_control => ALUControltoALU,

							  result => AluResultOut,
							  zero => ZeroCarry_TL
							  );
  
	
 ALU_CONTROL : ALUControl port map(
 
								    ALUOp => ALUOp_TL,
								    instr => InstructionRegOut(5 downto 0),
								    result => ALUControltoALU
									);
 
  
 
CTRL_UNIT : ControlUnit port map(
								 CLK => CLK,
								 Reset => reset_neg, 
								 Op => InstructionRegOut(31 downto 26),

								 PCWriteCond => PCWriteCond_TL,
								 PCWrite => PCWrite_TL,
								 IorD => IorD_TL, 
								 MemRead => MemRead_TL,
								 MemWrite => MemWrite_TL,

								 MemToReg => MemToReg_TL,
								 IRWrite => IRWrite_TL,
								 PCSource => PCsource_TL,
								 ALUOp => ALUOp_TL,
								 ALUSrcB => ALUSrcB_TL,
								 ALUSrcA => ALUSrcA_TL, 
								 RegWrite => RegWrite_TL,
								 RegDst => RegDst_TL
								 );
	
 
 
INSTR_REG : InstructionRegister port map(

										 CLK => CLK, 
										 reset_neg => reset_neg, 
										 IRWrite => IRWrite_TL, 
										 in_instruction => MemDataOut,
										 out_instruction => InstructionRegOut
										 );
 
 
 
MEM : Memory port map(
					   
CLK => CLK,
					   reset_neg => reset_neg,
					   address => MuxToAddress,
					   MemWrite => MemWrite_TL,
					   MemRead => MemRead_TL,

					   WriteData => RegBOut,
					   MemData => MemDataOut
					   );
 
 
MEM_DATA_REG : MemoryDataRegister port map(
 
										    CLK => CLK, 
										    reset_neg => reset_neg,
										    input => MemDataOut, 
										    output => MemoryDataRegOut
											);
											
 MUX_1 : Mux2 port map(
       					input_1 => PC_out,
						input_2 => AluOutToMux,
						mux_select => IorD_TL,
						output => MuxToAddress
						);
  
 MUX_2 : Mux2_5 port map(
     					  input_1 => InstructionRegOut(20 downto 16),
						  input_2 => InstructionRegOut(15 downto 11),
						  mux_select => RegDst_TL,
						  output => MuxToWriteRegister
						  );
  
  MUX_3 : Mux2 port map(
  						input_1 => AluOutToMux,
						input_2 => MemoryDataRegOut,
						mux_select => MemToReg_TL,
						output => MuxToWriteData
						);
  
  MUX_4 : Mux2 port map(
						input_1 => PC_out,
						input_2 => RegAToMux,
						mux_select => ALUSrcA_TL,
						output => MuxToAlu
						);
     
  MUX_5 : Mux4 port map(
					  input_1 => RegBOut,
					  input_2 => PC_increment,
					  input_3 => SignExtendOut,
					  input_4 => ShiftLeft1ToMux4,
					  mux_select => ALUSrcB_TL,
					  output => Mux4ToAlu
					  );
					  
  MUX_6 : Mux3 port map(
						input_1 => AluResultOut,
						input_2 => AluOutToMux,
						input_3 => JumpAddress,
						mux_select => PCsource_TL,
						output => MuxToPC
						); 

  PC : ProgramCounter port map(	
							  CLK => CLK,
							  reset_neg => reset_neg,
							  input => MuxToPC,
							  PCcontrol => ORtoPC,
							  output => PC_out
							  );
							  
  REG : Registers port map(
						  CLK => CLK,
						  reset_neg => reset_neg,
						  address_in_1 => InstructionRegOut(25 downto 21),
						  address_in_2 => InstructionRegOut(20 downto 16),
						  write_reg => MuxToWriteRegister,
						  write_data => MuxToWriteData,
						  RegWrite => RegWrite_TL,
						  register_1 => ReadData1ToA,
						  register_2 => ReadData2ToB
						  );
  
  SE : SignExtend port map(
						   input => InstructionRegOut(15 downto 0),
						   output => SignExtendOut
						   );
 
  SLL1 : ShiftLeft port map(
							input => SignExtendOut, 
							output => ShiftLeft1ToMux4);

  SLL2 : ShiftLeft2 port map(
							  input => InstructionRegOut(25 downto 0),
							  output => JumpAddress(27 downto 0)
	  						    );
  
 TEMP_REG : TempRegisters port map(
									CLK => CLK,
								    reset_neg => reset_neg,
									in_reg_A => ReadData1ToA,
									in_reg_B => ReadData2ToB,
									in_ALU_out => AluResultOut,
									out_reg_A => RegAToMux,
									out_reg_B => RegBOut,
									out_ALU_out => AluOutToMux
									);
									
end Behavioral;
