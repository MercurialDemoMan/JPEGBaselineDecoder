#include "HuffmanStream.hpp"

#include <string>

void HuffmanStream::push_back(u8 value)
{
	stream.push_back(value);
}

u32 HuffmanStream::get_bits(u32 num_bits)
{
	u32 value = 0;
	
	while(num_bits--)
	{
		if(byte_offset >= stream.size())
		{
			throw std::runtime_error("[HuffmanStream] stream overflow");
		}
		
		u8 current_byte = stream[byte_offset];
		u8 current_bit  = 1 & (current_byte >> (7 - bit_offset));
		bit_offset++;
		
		value = (value << 1) | current_bit;
		
		if(bit_offset == 8)
		{
			byte_offset++;
			bit_offset = 0;
		}
	}
	
	return value;
}

u8 HuffmanStream::get_value(const HuffmanTable& table)
{	
	u32 code    = 0;
	u32 code_id = 0;
	
	//no code can be more than 16 bits wide
	for(u32 i = 0; i < 16; i++)
	{
		u32 res = get_bits(1);
		
		code = (code << 1) | res;
		for(u32 j = 0; j < table.code_sizes[i]; j++)
		{
			if(code == table.codes[code_id])
			{
				return table.values[code_id];
			}
			
			code_id++;
		}
	}
	
	throw std::runtime_error("[HuffmanStream] couldn't get next value");
	
	return 0;
}

