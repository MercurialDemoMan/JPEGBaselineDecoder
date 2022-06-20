#pragma once

#include "Utils.hpp"
#include <vector>
#include <array>

/**
 * Huffman Table Structure
 */
struct HuffmanTable
{
	std::array<u8, 16> code_sizes {};
	std::vector<u8>  values;
	std::vector<u16> codes;
};
	
/**
 * Stream For Parsing Huffman Compression
 */
struct HuffmanStream
{
	std::vector<u8> stream;
	u8 bit_offset   { 0 };
	u32 byte_offset { 0 };
	
	void push_back(u8 value);
	
	u32 get_bits(u32 num_bits);
	u8 get_value(const HuffmanTable& table);
};
