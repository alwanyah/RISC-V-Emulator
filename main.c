#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <stdint.h>

enum opcode_decode
{
	R = 0x33,
	I = 0x13,
	S = 0x23,
	L = 0x03,
	B = 0x63,
	JALR = 0x67,
	JAL = 0x6F,
	AUIPC = 0x17,
	LUI = 0x37
};

typedef struct
{
	size_t data_mem_size_;
	uint32_t regfile_[32];
	uint32_t pc_;
	uint8_t *instr_mem_;
	uint8_t *data_mem_;
} CPU;

void CPU_open_instruction_mem(CPU *cpu, const char *filename);
void CPU_load_data_mem(CPU *cpu, const char *filename);

uint32_t ram_start = 0;

CPU *CPU_init(const char *path_to_inst_mem, const char *path_to_data_mem)
{
	CPU *cpu = (CPU *)malloc(sizeof(CPU));
	cpu->data_mem_size_ = 0x400000;
	cpu->pc_ = 0x0;
	CPU_open_instruction_mem(cpu, path_to_inst_mem);
	CPU_load_data_mem(cpu, path_to_data_mem);
	return cpu;
}

void CPU_open_instruction_mem(CPU *cpu, const char *filename)
{
	uint32_t instr_mem_size;
	FILE *input_file = fopen(filename, "r");
	if (!input_file)
	{
		printf("no input\n");
		exit(EXIT_FAILURE);
	}
	struct stat sb;
	if (stat(filename, &sb) == -1)
	{
		printf("error stat\n");
		perror("stat");
		exit(EXIT_FAILURE);
	}
	printf("size of instruction memory: %ld Byte\n\n", sb.st_size);
	instr_mem_size = sb.st_size;
	cpu->instr_mem_ = malloc(instr_mem_size);
	fread(cpu->instr_mem_, sb.st_size, 1, input_file);
	fclose(input_file);
	return;
}

void CPU_load_data_mem(CPU *cpu, const char *filename)
{
	FILE *input_file = fopen(filename, "r");
	if (!input_file)
	{
		printf("no input\n");
		exit(EXIT_FAILURE);
	}
	struct stat sb;
	if (stat(filename, &sb) == -1)
	{
		printf("error stat\n");
		perror("stat");
		exit(EXIT_FAILURE);
	}
	printf("read data for data memory: %ld Byte\n\n", sb.st_size);

	cpu->data_mem_ = malloc(cpu->data_mem_size_);
	fread(cpu->data_mem_, sb.st_size, 1, input_file);
	fclose(input_file);
	return;
}

/**
 * Instruction fetch Instruction decode, Execute, Memory access, Write back
 */

/*
* read the commands from the memory
*/

int target_read_u8(uint8_t *pval, uint32_t addr, CPU *cpu) // for lb & lbu
{

	addr -= ram_start;
	if (addr > cpu->data_mem_size_)  // check if addr is rigth
	{
		*pval = 0;
		printf("illegal read 8, PC: 0x%08x, address: 0x%08x\n", cpu->pc_,
			   addr + ram_start);
		return 1;
	}
	else
	{
		uint8_t *p = cpu->data_mem_ + addr;
		*pval = p[0];
	}
	return 0;
}

int target_read_u16(uint16_t *pval, uint32_t addr, CPU *cpu) // for lh & lhu
{

	addr -= ram_start;
	if (addr > cpu->data_mem_size_)  // check if addr is rigth
	{
		*pval = 0;
		printf("illegal read 16, PC: 0x%08x, address: 0x%08x\n", cpu->pc_,
			   addr + ram_start);
		return 1;
	}
	else
	{
		uint8_t *p = cpu->data_mem_ + addr;
		*pval = p[0] | (p[1] << 8);
	}
	return 0;
}

int target_read_u32(uint32_t *pval, uint32_t addr, CPU *cpu) // for lw
{

	addr -= ram_start;
	if (addr > cpu->data_mem_size_)  // check if addr is rigth
	{
		*pval = 0;
		printf("illegal read 32, PC: 0x%08x, address: 0x%08x\n", cpu->pc_,
			   addr + ram_start);
		return 1;
	}
	else
	{
		uint8_t *p = cpu->data_mem_ + addr;
		*pval = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
	}

	return 0;
}
#define UART_TX_ADDR 0x5000 // schnittstelle
#define XLEN 32

/*
* write the commands in the memory
*/

int target_write_u8(uint32_t addr, uint8_t val, CPU *cpu) // need it for sb
{
	if (addr == UART_TX_ADDR)
	{
		//printf("Output: \n");
		printf("%c", val);
	}
	else
	{
		addr -= ram_start; // ram_start always 0 (mabye useless?)
		if (addr > cpu->data_mem_size_ - 1) // check if addr is rigth
		{
			printf("illegal write 8, PC: 0x%08x, address: 0x%08x\n", cpu->pc_,
				   addr + ram_start);
			return 1;
		}
		else
		{
			uint8_t *p = cpu->data_mem_ + addr;
			p[0] = val & 0xff;
		}
	}
	return 0;
}

int target_write_u16(uint32_t addr, uint16_t val, CPU *cpu) // need it for sh
{

	addr -= ram_start;
	if (addr > cpu->data_mem_size_ - 2) // check if addr is rigth
	{
		printf("illegal write 16, PC: 0x%08x, address: 0x%08x\n", cpu->pc_,
			   addr + ram_start);
		return 1;
	}
	else
	{
		uint8_t *p = cpu->data_mem_ + addr;
		p[0] = val & 0xff;
		p[1] = (val >> 8) & 0xff;
	}
	return 0;
}

int target_write_u32(uint32_t addr, uint32_t val, CPU *cpu) // need it for sw
{

	addr -= ram_start;
	if (addr > cpu->data_mem_size_ - 4) // check if addr is rigth
	{
		return 1;
	}
	else
	{
		uint8_t *p = cpu->data_mem_ + addr;
		p[0] = val & 0xff;
		p[1] = (val >> 8) & 0xff;
		p[2] = (val >> 16) & 0xff;
		p[3] = (val >> 24) & 0xff;
	}

	return 0;
}

void CPU_execute(CPU *cpu, int i)
{
	uint32_t instruction = *(uint32_t *)(cpu->instr_mem_ + (cpu->pc_ & 0xFFFFF));
	uint32_t next_pc = cpu->pc_ + 4;

	uint32_t opcode, rd, rs1, rs2, funct3;
	// uint32_t cond, err;  // can remove
	uint32_t addr, val = 0, val2;
	
	uint32_t imm_i, imm_s, imm_b, imm_u, imm_j;
	imm_i = instruction >> 20;
	imm_s = ((instruction >> (8 - 1)) & 0x1f) | ((instruction >> (25 - 5)) & 0xfe0);
	imm_b = ((instruction >> (8 - 1)) & 0x1e) | ((instruction >> (25 - 5)) & 0x7e0) | ((instruction << (11 - 7)) & (1 << 11));
	imm_u = instruction & 0xfffff000;
	imm_j = ((instruction >> (21 - 1)) & 0x7fe) | ((instruction >> (20 - 11)) & (1 << 11)) | (instruction & 0xff000);
	
	// Replicate bit 31 if set
	if (instruction & (1 << 31)) {
	    imm_i |= 0xFFFFF800;
	    imm_s |= 0xFFFFF800;
	    imm_b |= 0xFFFFF000;
	    imm_j |= 0xFFF00000;
	}

	opcode = instruction & 0x7f;
	rd = (instruction >> 7) & 0x1f;
	rs1 = (instruction >> 15) & 0x1f;
	rs2 = (instruction >> 20) & 0x1f;
	funct3 = (instruction >> 12) & 0x7;
	
	//printf("pc=%08X instr: %08X opcode %02X rd=%02X r1=%02X r2=%02X funct3=%02X\n", cpu->pc_, instruction, opcode, rd, rs1, rs2, funct3);

	if (opcode == 0x37) // 0110111
	{ // lui
		if (rd != 0)
			cpu->regfile_[rd] = imm_u;
	}
	else if (opcode == 0x17) // 0010111
	{ // auipc
		if (rd != 0)
			cpu->regfile_[rd] = cpu->pc_ + imm_u;
	}
	else if (opcode == 0x6f) // 1101111
	{ // jal
		if (rd != 0)
			cpu->regfile_[rd] = cpu->pc_ + 4;

		next_pc = cpu->pc_ + (int32_t)imm_j;
	}
	else if (opcode == 0x67) // 1100111
	{ // jalr
		next_pc = cpu->regfile_[rs1] + (int32_t)imm_i;
		if (rd != 0)
			cpu->regfile_[rd] = cpu->pc_ + 4;
	}
	else if (opcode == 0x63)  // 1100011
	{ // B-type
		//printf("Branch: imm=%08X funct3=%01X\n", imm_b, funct3);

		switch (funct3)
		{
		case 0x0: // beq
			if (cpu->regfile_[rs1] == cpu->regfile_[rs2])
				next_pc = cpu->pc_ + (int32_t)imm_b;
			break;
		case 0x1: // bne
			if (cpu->regfile_[rs1] != cpu->regfile_[rs2])
				next_pc = cpu->pc_ + (int32_t)imm_b;
			break;
		case 0x4: // blt
			if ((int32_t)cpu->regfile_[rs1] < (int32_t)cpu->regfile_[rs2])
				next_pc = cpu->pc_ + (int32_t)imm_b;
			break;
		case 0x5: // bge
			if ((int32_t)cpu->regfile_[rs1] >= (int32_t)cpu->regfile_[rs2])
				next_pc = cpu->pc_ + (int32_t)imm_b;
			break;
		case 0x6: // bltu
			if (cpu->regfile_[rs1] < cpu->regfile_[rs2])
				next_pc = cpu->pc_ + (int32_t)imm_b;
			break;
		case 0x7: // bgeu
			if (cpu->regfile_[rs1] >= cpu->regfile_[rs2])
				next_pc = cpu->pc_ + (int32_t)imm_b;
			break;
		}
		
		//if (next_pc != cpu->pc_+4) {
		//    printf("Branch taken! next_pc=%08X imm=%d\n", next_pc, (int32_t)imm_b);
		//}
	}
	else if (opcode == 0x03) // 0000011
	{ // I-Type, L*
		addr = cpu->regfile_[rs1] + (int32_t)imm_i;

		switch (funct3)
		{
		case 0x0: // lb
		{
			uint8_t rval;
			target_read_u8(&rval, addr, cpu);
			val = (int8_t)rval;
		}
		break;

		case 0x1: // lH
		{
			uint16_t rval;
			target_read_u16(&rval, addr, cpu);
			val = (int16_t)rval;
		}
		break;

		case 0x2: // lW
		{   
			uint32_t rval;
			target_read_u32(&rval, addr, cpu);
			val = rval;
		}
		break;
		case 0x4: // lbu
		{
			uint8_t rval;
			target_read_u8(&rval, addr, cpu);
			val = rval;
		}
		break;
		case 0x5: // lhu
		{
			uint16_t rval;
			target_read_u16(&rval, addr, cpu);
			val = rval;
		}
		break;
		}
		
		//printf("L*: funct3=%01X imm=%d addr=%08X val=%08X\n", funct3, (int32_t)imm_i, addr, val);
		
		if (rd != 0)
			cpu->regfile_[rd] = val;
	}
	else if (opcode == 0x23) // 0100011
	{ // S-type
		addr = cpu->regfile_[rs1] + (int32_t)imm_s;
		val = cpu->regfile_[rs2];

		switch (funct3)
		{

		case 0x0: // sb
		    //printf("SB: addr=%08X val=%02X\n", addr, val);
			target_write_u8(addr, val, cpu);
			break;
		case 0x1: // sh
		    //printf("SH: addr=%08X val=%04X\n", addr, val);
			target_write_u16(addr, val, cpu);
			break;
		case 0x2: // sw
		    //printf("SW: addr=%08X val=%08X\n", addr, val);
			target_write_u32(addr, val, cpu);
			break;
		}
	}
	else if (opcode == 0x13) // 0010011
	{ // I-type, *I
		switch (funct3)
		{
		case 0x0: // addi
			val = cpu->regfile_[rs1] + imm_i;
			break;
		case 0x2: // slti
			val = (int32_t)cpu->regfile_[rs1] < (int32_t)imm_i;
			break;
		case 0x3: // sltiu
			val = cpu->regfile_[rs1] < imm_i;
			break;
		case 0x4: // xori
			val = cpu->regfile_[rs1] ^ imm_i;
			break;
		case 0x6: // ori
			val = cpu->regfile_[rs1] | imm_i;
			break;
		case 0x7: // andi
			val = cpu->regfile_[rs1] & imm_i;
			break;
		case 0x1: // slli
			val = cpu->regfile_[rs1] << rs2;
			break;
		case 0x5: // srli / srai
			if (imm_i & 0x400) // srai
			{
				val = (int32_t)cpu->regfile_[rs1] >> (int32_t)rs2;
			}
			else // srli
			{
				val = cpu->regfile_[rs1] >> rs2;
			}
			break;
		}

		if (rd != 0) {
		    //printf("I-type *I: val=%08X rd=%02X\n", val, rd);
			cpu->regfile_[rd] = val;
		}
	}
	else if (opcode == 0x33) // 0010011
	{ // R-type
		val = cpu->regfile_[rs1];
		val2 = cpu->regfile_[rs2];

		funct3 |= ((instruction >> (30 - 3)) & (1 << 3));

		switch (funct3)
		{
		case 0x0: // add
			val = val + val2;
			break;
		case 0x0 | 8: // sub
			val = val - val2;
			break;
		case 0x1: // sll
			val = val << val2;
			break;
		case 0x2: // slt
			val = (int32_t)val < (int32_t)val2;
			break;
		case 0x3: // sltu
			val = val < val2;
			break;
		case 0x4: // xor
			val = val ^ val2;
			break;
		case 0x5: // srl
			val = val >> val2;
			break;
		case 0x5 | 8: // sra
			val = (int32_t)val >> val2;
            break;
		case 0x6: // or
			val = val | val2;
			break;
		case 0x7: // and
			val = val & val2;
			break;
		}

		if (rd != 0) {
		    //printf("R-type: val=%08X rd=%02X\n", val, rd);
			cpu->regfile_[rd] = val;
		}
	}

	cpu->pc_ = next_pc; // pc +4
}

int main(int argc, char *argv[])
{

	printf("C Praktikum\nHU Risc-V  Emulator 2022\n");

	CPU *cpu_inst;

	cpu_inst = CPU_init(argv[1], argv[2]);
	for (uint32_t i = 0; i < 100000000; i++)
	{ // run 70000 cycles
		CPU_execute(cpu_inst, i);
	}

	printf(
		"\n-----------------------RISC-V program terminate------------------------\nRegfile values:\n");

	// output Regfile
	for (uint32_t i = 0; i <= 31; i++)
	{
		printf("%d: %X\n", i, cpu_inst->regfile_[i]);
	}
	fflush(stdout);

	return 0;
}
