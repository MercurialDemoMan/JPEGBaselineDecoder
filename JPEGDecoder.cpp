#include "JPEGDecoder.hpp"


#define _USE_MATH_DEFINES
#include <math.h>
#include <omp.h>
#include <array>
#include <vector>
#include <chrono>
#include <string>
#include <memory>
#include <cassert>
#include <fstream>
#include <numeric>
#include <iostream>
#include <optional>
#include <iterator>
#include <algorithm>
#include <unordered_map>

#include "HuffmanStream.hpp"
#include "ByteStream.hpp"

namespace JPEGDecoder
{
	constexpr u32 BlockSize = 8;
	
	template<typename T = u32>
	using JPEGBlock = std::array<T, BlockSize * BlockSize>;
	
	struct Macroblock
	{
		union
		{
			struct
			{
				JPEGBlock<int> y;
				JPEGBlock<int> cb;
				JPEGBlock<int> cr;
			};
			
			JPEGBlock<int> components[3];
		};
	};
	
	static_assert(sizeof(Macroblock) == sizeof(JPEGBlock<int>) * 3);
	
	/**
	 * JPEG Markers Indicies For The Ease Of Comparison
	 */
	enum class JPEGSectionMarker
	{
		SOF0,  SOF1,  SOF2,  SOF3,
		SOF5,  SOF6,  SOF7,  JPG,
		SOF9,  SOF10, SOF11, SOF12,
		SOF13, SOF14, SOF15, SOI,
		EOI,   DQT,   DHT,   DRI, 
		SOS,   RST0,  RST1,  RST2,
		RST3,  RST4,  RST5,  RST6,
		RST7,  APP0,  APP1,  APP2,
		APP3,  APP4,  APP5,  APP6,  
		APP7,  APP8,  APP9,  APP10, 
		APP11, APP12, APP13, APP14,  
		APP15,  
	};
	
	/**
	 * JPEG Section Markers
	 */
	const std::unordered_map<u8, JPEGSectionMarker> Markers = 
	{
		{ 0xC0, JPEGSectionMarker::SOF0 },
		{ 0xC1, JPEGSectionMarker::SOF1 },
		{ 0xC2, JPEGSectionMarker::SOF2 },
		{ 0xC3, JPEGSectionMarker::SOF3 },
		{ 0xC5, JPEGSectionMarker::SOF5 },
		{ 0xC6, JPEGSectionMarker::SOF6 },
		{ 0xC7, JPEGSectionMarker::SOF7 },
		{ 0xC8, JPEGSectionMarker::JPG },
		{ 0xC9, JPEGSectionMarker::SOF9 },
		{ 0xCA, JPEGSectionMarker::SOF10 },
		{ 0xCB, JPEGSectionMarker::SOF11 },
		{ 0xCC, JPEGSectionMarker::SOF12 },
		{ 0xCD, JPEGSectionMarker::SOF13 },
		{ 0xCE, JPEGSectionMarker::SOF14 },
		{ 0xCF, JPEGSectionMarker::SOF15 },
		{ 0xD0, JPEGSectionMarker::RST0 },
		{ 0xD1, JPEGSectionMarker::RST1 },
		{ 0xD2, JPEGSectionMarker::RST2 },
		{ 0xD3, JPEGSectionMarker::RST3 },
		{ 0xD4, JPEGSectionMarker::RST4 },
		{ 0xD5, JPEGSectionMarker::RST5 },
		{ 0xD6, JPEGSectionMarker::RST6 },
		{ 0xD7, JPEGSectionMarker::RST7 },
		{ 0xD8, JPEGSectionMarker::SOI },
		{ 0xD9, JPEGSectionMarker::EOI },
		{ 0xDB, JPEGSectionMarker::DQT },
		{ 0xC4, JPEGSectionMarker::DHT },
		{ 0xDD, JPEGSectionMarker::DRI },
		{ 0xDA, JPEGSectionMarker::SOS },
		{ 0xE0, JPEGSectionMarker::APP0 },
		{ 0xE1, JPEGSectionMarker::APP1 },
		{ 0xE2, JPEGSectionMarker::APP2 },
		{ 0xE3, JPEGSectionMarker::APP3 },
		{ 0xE4, JPEGSectionMarker::APP4 },
		{ 0xE5, JPEGSectionMarker::APP5 },
		{ 0xE6, JPEGSectionMarker::APP6 },
		{ 0xE7, JPEGSectionMarker::APP7 },
		{ 0xE8, JPEGSectionMarker::APP8 },
		{ 0xE9, JPEGSectionMarker::APP9 },
		{ 0xEA, JPEGSectionMarker::APP10 },
		{ 0xEB, JPEGSectionMarker::APP11 },
		{ 0xEC, JPEGSectionMarker::APP12 },
		{ 0xED, JPEGSectionMarker::APP13 },
		{ 0xEE, JPEGSectionMarker::APP14 },
		{ 0xEF, JPEGSectionMarker::APP15 },
	};
	
	/**
	 * Nested Zig Zag Indicies
	 */
	const JPEGBlock<u8> NestedZigZagIndicies = 
	{
		 0,  1,  8, 16,  9,  2,  3, 10,
        17, 24, 32, 25, 18, 11,  4,  5,
        12, 19, 26, 33, 40, 48, 41, 34,
        27, 20, 13,  6,  7, 14, 21, 28,
        35, 42, 49, 56, 57, 50, 43, 36,
        29, 22, 15, 23, 30, 37, 44, 51,
        58, 59, 52, 45, 38, 31, 39, 46,
        53, 60, 61, 54, 47, 55, 62, 63
	};

	/**
	 * Component Description
	 */
	struct ComponentInfo
	{
		u8 id;
		u8 sample_factor_h;
		u8 sample_factor_v;
		u8 quantization_table_id;
		u8 ac_dest_id;
		u8 dc_dest_id;
	};
	
	/**
	 * State For Decoding JPEG Image
	 */
	struct JPEGState
	{
		enum class State
		{
			Uninitialized,
			ParsingHeader,
			ParsingHuffman,
			Decoding
		} state = State::Uninitialized;
		
		std::unordered_map<u32, JPEGBlock<>>  quantization_tables; // quantization tables
		std::unordered_map<u8, HuffmanTable>  ac_huffman_tables;   // huffman
		std::unordered_map<u8, HuffmanTable>  dc_huffman_tables;   // tables
		std::vector<ComponentInfo>            components_info;     // info about y, cr, cb components
		HuffmanStream                         huffman_stream;      // encoded macroblocks
		std::vector<Macroblock>               macroblocks;         // decoded macroblocks
		
		u32 reset_interval  { 0 }; // reset dc previous values
		s32 reset_counter   { 0 };
		u32 bit_depth       { 8 }; // pixel component width
		u32 width           { 0 }; // image width
		u32 height          { 0 }; // image height
		u32 block_h         { 0 }; // number of blocks in horizontal direction
		u32 block_v         { 0 }; // number of blocks in vertical direction
		u32 block_padded_h  { 0 }; 
		u32 block_padded_v  { 0 };
		u32 block_num       { 0 }; // number of blocks
		u8  sample_factor_h { 0 }; // biggest horizontal sample factor (usually luma)
		u8  sample_factor_v { 0 }; // biggest vertical sample factor   (usually luma)
		s32 previous_dc_values[3] = { 0, 0, 0 };
	};
	
	/**
	 * Create Image From Decoded Macroblocks
	 */
	static Utils::Image ConstructImage(JPEGState& state)
	{
		Utils::Image output;
		
		output.width  = state.width;
		output.height = state.height;
		output.pixels.resize(state.width * state.height);
		
		#pragma omp parallel for collapse(2) shared(state, output)
		for(u32 y = 0; y < state.height; y++)
		{
			for(u32 x = 0; x < state.width; x++)
			{
				auto& block = state.macroblocks[(y / BlockSize) * state.block_padded_h + (x / BlockSize)];
				
				Utils::Color color
				{ 
					{ block.y [(y & 0b111) * BlockSize + (x & 0b111)] },
					{ block.cb[(y & 0b111) * BlockSize + (x & 0b111)] },
					{ block.cr[(y & 0b111) * BlockSize + (x & 0b111)] }
				};
				
				output.pixels[y * output.width + x] = color;
			}
		}

		return output;
	}
	
	/**
	 * Convert Decoded Macroblocks From YCbCr To RGB
	 */
	static void ConvertToRGB(JPEGState& state)
	{
		#pragma omp parallel for collapse(2) shared(state)
		for(u32 v = 0; v < state.block_v; v += state.sample_factor_v)
		{
			for(u32 h = 0; h < state.block_h; h += state.sample_factor_h)
			{
				//get potentially down-sampled chroma components
				auto& chroma = state.macroblocks[v * state.block_padded_h + h];
						
				for(u8 y = 0; y < state.sample_factor_v; y++)
				{
					for(u8 x = 0; x < state.sample_factor_h; x++)
					{
						u8 sub_y = (state.sample_factor_v - 1 - y);
						u8 sub_x = (state.sample_factor_h - 1 - x);
						
						//get full resolution luma component
						auto& luma = state.macroblocks[(v + sub_y) * state.block_padded_h + (h + sub_x)];
						
						for(u8 i = 0; i < BlockSize; i++)
						{
							for(u8 j = 0; j < BlockSize; j++)
							{
								u8 pix_i = BlockSize - 1 - i;
								u8 pix_j = BlockSize - 1 - j;
								
								u32 luma_id   = pix_i * BlockSize + pix_j;
								u32 chroma_id = ((pix_i / state.sample_factor_v) + (4 * sub_y)) *
								                BlockSize +
												((pix_j / state.sample_factor_h) + (4 * sub_x));
								
								Utils::Color color = Utils::YCbCrToRGB
								({
									{ luma.y[luma_id]      },
									{ chroma.cb[chroma_id] },
									{ chroma.cr[chroma_id] }
								});
								
								luma.y [luma_id] = color.r;
								luma.cb[luma_id] = color.g;
								luma.cr[luma_id] = color.b;
							}
						}
					}
				}
			}
		}
	}
	
	/**
	 * Load Huffman Stream From SOS Section
	 */
	static void LoadHuffmanStream(JPEGState& state, ByteStream& stream)
	{
		u8 prev_byte;
		u8 current_byte = stream.get();
		
		while(true)
		{
			prev_byte = current_byte;
			current_byte = stream.get();
			
			if(prev_byte == 0xFF)
			{
				switch(current_byte)
				{
					// double marker skip
					case 0xFF: { continue; break; }
					// end of image -> end loading
					case 0xD9: { return; break; }
					// reset decoding
					case 0x00: 
					{
						current_byte = stream.get();
						state.huffman_stream.push_back(prev_byte);
						prev_byte = current_byte;
						continue;
						break;
					}
					// reset decoding
					case 0xD0 ... 0xD7:
					{
						state.huffman_stream.push_back(current_byte);
						prev_byte = current_byte;
						current_byte = stream.get();
						continue;
						break;
					}
					
					default:
					{
						throw std::runtime_error("[LoadHuffmanStream] SOS data contain unknown marker: " + std::to_string(current_byte));	
					}
				}
			}
			else
			{
				state.huffman_stream.push_back(prev_byte);
			}
		}
		
		throw std::runtime_error("[LoadHuffmanStream] Couldn't find EOI");
	}
	
	/**
	 * Decode Huffman Stream Containing Macroblocks
	 */
	static void DecodeMacroblocks(JPEGState& state, u32 h, u32 v)
	{
		//for every component
		for(u32 component_id = 0; component_id < state.components_info.size(); component_id++)
		{
			auto& component_info = state.components_info[component_id];
			auto& dc_table = state.dc_huffman_tables.find(component_info.dc_dest_id)->second;
			auto& ac_table = state.ac_huffman_tables.find(component_info.ac_dest_id)->second;
			
			//for every subsample
			for(u8 y = 0; y < component_info.sample_factor_v; y++)
			{
				for(u8 x = 0; x < component_info.sample_factor_h; x++)
				{
					auto& block = state.macroblocks[(v + y) * state.block_padded_h + (h + x)];
					
					u8 dc_length = state.huffman_stream.get_value(dc_table);
					
					//check MSB of diff, which signifies 1 = positive or 0 = negative sign
					s32 dc_diff = state.huffman_stream.get_bits(dc_length);
					if(dc_length != 0 && dc_diff < (1 << (dc_length - 1)))
					{
						dc_diff -= (1 << dc_length) - 1;
					}
					
					auto& current_component = block.components[component_id];
					state.previous_dc_values[component_id] += dc_diff;
					
					//first value in block is DC
					current_component[0] = state.previous_dc_values[component_id];
					
					//rest of the values in block are AC
					for(u32 j = 1; j < BlockSize * BlockSize; )
					{
						u8 ac_value = state.huffman_stream.get_value(ac_table);
						if(ac_value == 0)
						{
							break;
						}
						
						if(ac_value == 0xF0)
						{
							j += 16;
						}
						else
						{
							j += ac_value >> 4;
						}
		
						if(j >= BlockSize * BlockSize)
						{
							throw std::runtime_error("[BuildMacroblock] run length out of range: " + std::to_string(j));
						}
						
						u8 coeff_length = ac_value & 0x0F;
						if(coeff_length > 10)
						{
							throw std::runtime_error("[BuildMacroblock] coefficient length out of range: " + std::to_string(coeff_length));
						}
						
						if(coeff_length != 0)
						{
							s32 ac_coeff = state.huffman_stream.get_bits(coeff_length);
							if(ac_coeff < (1 << (coeff_length - 1)))
							{
								ac_coeff -= (1 << coeff_length) - 1;
							}
							
							current_component[NestedZigZagIndicies[j++]] = ac_coeff;
						}
					}
				}
			}
		}
	}
	
	/**
	 * Read And Parse Huffman Table From DHT JPEG Section
	 * TODO: this is the best conteder for performance issues
	 */
	static void DecodeHuffmanStream(JPEGState& state)
	{
		state.macroblocks.resize(state.block_num);
		
		for(u32 v = 0; v < state.block_padded_v / state.sample_factor_v; v++)
		{
			for(u32 h = 0; h < state.block_padded_h / state.sample_factor_h; h++)
			{
				u32 x = h * state.sample_factor_h;
				u32 y = v * state.sample_factor_v;
				
				DecodeMacroblocks(state, x, y);
				
				//check for dc restart interval
				if(state.reset_interval > 0)
				{
					if(--state.reset_counter == 0)
					{
						state.reset_counter = state.reset_interval;
						
						//reset dc values
						state.previous_dc_values[0] = 0;
						state.previous_dc_values[1] = 0;
						state.previous_dc_values[2] = 0;
						
						//reset bit stream boundary
						if(state.huffman_stream.byte_offset < state.huffman_stream.stream.size())
						{
							if(state.huffman_stream.bit_offset != 0)
							{
								state.huffman_stream.bit_offset = 0;
								state.huffman_stream.byte_offset++;
							}
							
							state.huffman_stream.byte_offset++;
						}
					}
				}
			}
		}
	}
	
	/**
	 * Inverse Discrete Cosine Transform For Block
	 */
	#pragma omp declare simd notinbranch
	static inline float DCTBase2D(int k, int j, int n, int m)
	{
		return (k == 0 ? 1.0 / std::sqrt(2.0) : 1.0) * 
			   (j == 0 ? 1.0 / std::sqrt(2.0) : 1.0) * 
			   std::cos((k * M_PI / BlockSize) * (n + 0.5)) * 
			   std::cos((j * M_PI / BlockSize) * (m + 0.5));
	}
	#pragma omp declare simd notinbranch
	static inline float DCTBase1D(int k, int n)
	{
		return (k == 0 ? 1.0 / std::sqrt(2.0) : 1.0) * 
		       std::sqrt(2.0 / BlockSize) * 
			   std::cos((k * M_PI / BlockSize) * (n + 0.5));
	};
	template<IDCTAlgorithm idct_algorithm = IDCTAlgorithm::CompoundLut>
	static void DoIDCTOnBlock(JPEGBlock<int>& input_block)
	{
		// Naive 2D IDCT
		// Reference Image: 10109x4542 (YCbCr, 8-bit)
		// Reference Time: 42272 ms
		if constexpr (idct_algorithm == IDCTAlgorithm::Reference)
		{
			JPEGBlock<int> block;
			
			//2D IDCT
			for(u32 y = 0; y < BlockSize; y++)
			{
				for(u32 x = 0; x < BlockSize; x++)
				{
					float result = 0.0;
					
					for(u32 v = 0; v < BlockSize; v++)
					{
						#pragma omp simd reduction(+:result)
						for(u32 u = 0; u < BlockSize; u++)
						{
							result += input_block[v * BlockSize + u] * DCTBase2D(u, v, x, y);
						}
					}
					
					block[y * BlockSize + x] = (2.0 / BlockSize) * result;
				}
			}

			input_block = std::move(block);
		}
		
		// Compound Of 1D IDCT's
		// Reference Image: 10109x4542 (YCbCr, 8-bit)
		// Reference Time: 5736 ms
		if constexpr (idct_algorithm == IDCTAlgorithm::Compound)
		{
			JPEGBlock<int> transposed;
			
			//1D IDCT
			for(u32 y = 0; y < BlockSize; y++)
			{
				for(u32 n = 0; n < BlockSize; n++)
				{
					float result = 0;
					
					#pragma omp simd reduction(+:result)
					for(u32 k = 0; k < BlockSize; k++)
					{
						result += input_block[y * BlockSize + k] * DCTBase1D(k, n);
					}
					
					// do transposition right here, which will cause cache misses,
					// but it's still faster than doing separate transposition
					transposed[n * BlockSize + y] = result;
				}
			}
			
			//1D IDCT
			for(u32 y = 0; y < BlockSize; y++)
			{
				for(u32 n = 0; n < BlockSize; n++)
				{
					float result = 0;
					
					#pragma omp simd reduction(+:result)
					for(u32 k = 0; k < BlockSize; k++)
					{
						result += transposed[y * BlockSize + k] * DCTBase1D(k, n);
					}
					
					//do transposition right here
					input_block[n * BlockSize + y] = result;
				}
			}
		}

		// Source: https://www.fit.vut.cz/research/publication-file/11440/preprint.pdf
		// Walsh-Hadamard IDCT Approximation
		// Reference Image: 10109x4542 (YCbCr, 8-bit)
		// Reference Time: 2939 ms
		// Less Precise = Lower Quality
		if constexpr (idct_algorithm == IDCTAlgorithm::WalshHadamard)
		{
			static const JPEGBlock<float> WalshHadamardBlock = 
			{
				+1, +1, +1, +1, +1, +1, +1, +1,
				+1, +1, +1, +1, -1, -1, -1, -1,
				+1, +1, -1, -1, +1, +1, -1, -1,
				+1, +1, -1, -1, -1, -1, +1, +1,
				+1, -1, +1, -1, +1, -1, +1, +1,
				+1, -1, +1, -1, -1, +1, -1, -1,
				+1, -1, -1, +1, +1, -1, -1, +1,
				+1, -1, -1, +1, -1, +1, +1, -1
			};
			
			JPEGBlock<int> block;
			
			for(u32 y = 0; y < BlockSize; y++)
			{
				for(u32 x = 0; x < BlockSize; x++)
				{
					float result = 0.0;
					
					#pragma omp simd collapse(2) reduction(+:result)
					for(u32 v = 0; v < BlockSize; v++)
					{
						for(u32 u = 0; u < BlockSize; u++)
						{
							result += input_block[v * BlockSize + u] * 
					                  WalshHadamardBlock[y * BlockSize + v] * 
							          WalshHadamardBlock[x * BlockSize + u];
						}
					}
					
					block[y * BlockSize + x] = (1.0 / BlockSize) * result;
				}
			}
			
			input_block = std::move(block);
		}
	
		// Naive 2D IDCT With Precomputed Values
		// Reference Image: 10109x4542 (YCbCr, 8-bit)
		// Reference Time: 368 ms
		if constexpr (idct_algorithm == IDCTAlgorithm::ReferenceLut)
		{
			static const float lut[BlockSize * BlockSize * BlockSize * BlockSize] =
			{
				#include "luts/IDCT_LUT_2D.txt"
			};
			
			JPEGBlock<int> block;
			
			//2D IDCT
			for(u32 y = 0; y < BlockSize; y++)
			{
				for(u32 x = 0; x < BlockSize; x++)
				{
					float result = 0.0;
					
					#pragma omp simd collapse(2) reduction(+:result)
					for(u32 v = 0; v < BlockSize; v++)
					{
						for(u32 u = 0; u < BlockSize; u++)
						{
							result += input_block[v * BlockSize + u] * lut[(y * BlockSize * BlockSize * BlockSize) + 
					                                                       (x * BlockSize * BlockSize) + 
															               (v * BlockSize) + 
															               (u)];
						}
					}
					
					block[y * BlockSize + x] = result;
				}
			}

			input_block = std::move(block);
		}
		
		// Compound Of 1D IDCT's With Precomputed Values
		// Reference Image: 10109x4542 (YCbCr, 8-bit)
		// Reference Time: 105 ms
		if constexpr (idct_algorithm == IDCTAlgorithm::CompoundLut)
		{
			static const float lut[BlockSize * BlockSize] =
			{
				#include "luts/IDCT_LUT_1D.txt"
			};
			
			JPEGBlock<int> transposed;
			
			//1D IDCT
			for(u32 y = 0; y < BlockSize; y++)
			{
				for(u32 n = 0; n < BlockSize; n++)
				{
					float result = 0;
					#pragma omp simd reduction(+:result)
					for(u32 k = 0; k < BlockSize; k++)
					{
						result += input_block[y * BlockSize + k] * lut[n * BlockSize + k];
					}
					
					transposed[n * BlockSize + y] = result;
				}
			}
						
			//1D IDCT
			for(u32 y = 0; y < BlockSize; y++)
			{
				for(u32 n = 0; n < BlockSize; n++)
				{
					float result = 0;
					#pragma omp simd reduction(+:result)
					for(u32 k = 0; k < BlockSize; k++)
					{
						result += transposed[y * BlockSize + k] * lut[n * BlockSize + k];
					}
					
					input_block[n * BlockSize + y] = result;
				}
			}
			
			/*
			// transposition
			for(u32 y = 0; y < BlockSize; y++)
			{
				#pragma omp simd
				for(u32 x = y + 1; x < BlockSize; x++)
				{
					std::swap(input_block[y * BlockSize + x], input_block[x * BlockSize + y]);
				}
			}
			*/
		}
	
		// Arai algoritm for 2D IDCT
		// source: Y. Arai, T. Agui, and M. Nakajima, "A Fast DCT-SQ Scheme for Images", Transactions of the IEICE E 71(11): 1095, November 1988.
		// Reference Image: 10109x4542 (YCbCr, 8-bit)
		// Reference Time: 58 ms
		// broken :/
		if constexpr (idct_algorithm == IDCTAlgorithm::Arai)
		{
			static float a0 = 1.0 / (2.0 * std::sqrt(2.0));
			static float a1 = std::cos(8.0 * M_PI / 16.0) / (2.0 * std::sin(3.0 * M_PI / 8.0) - std::sqrt(2.0));
			static float a2 = std::cos(M_PI / 8.0) / std::sqrt(2.0);
			static float a3 = std::cos(5.0 * M_PI / 16.0) / (std::sqrt(2.0) + 2.0 * std::cos(3.0 * M_PI / 8.0));
			static float a4 = 1.0 / (2.0 * sqrt(2.0));
			static float a5 = std::cos(3.0 * M_PI / 16.0) / (std::sqrt(2.0) - 2.0 * std::cos(3.0 * M_PI / 8.0));
			static float a6 = std::cos(3.0 * M_PI / 8.0) / std::sqrt(2.0);
			static float a7 = std::cos(M_PI / 16.0) / (std::sqrt(2.0) + 2.0 * std::sin(3.0 * M_PI / 8.0));
			
			static float c_pi_4   = std::cos(M_PI / 4);
			static float c_3_pi_8 = std::cos(3 * M_PI / 8);
			static float s_3_pi_8 = std::sin(3 * M_PI / 8);
			
			#pragma omp simd
			for(u32 i = 0; i < BlockSize; i++)
			{
				float b0 = input_block[0 * BlockSize + i] * a0;
				float b1 = input_block[1 * BlockSize + i] * a1;
				float b2 = input_block[2 * BlockSize + i] * a2;
				float b3 = input_block[3 * BlockSize + i] * a3;
				float b4 = input_block[4 * BlockSize + i] * a4;
				float b5 = input_block[5 * BlockSize + i] * a5;
				float b6 = input_block[6 * BlockSize + i] * a6;
				float b7 = input_block[7 * BlockSize + i] * a7;
				
				float c0 = b0 + b4;
				float c1 = b0 - b4;
				float c2 = b6 - b2;
				float c3 = (b6 + b2) * c_pi_4;
				float c4 = b1 + b7;
				float c5 = b3 - b5;
				float c6 = b3 + b5;
				float c7 = a1 - a7;
				
				float d0 = c0;
				float d1 = c1;
				float d2 = c3 - c2;
				float d3 = c3;
				float d4 = (c5 - c4) * c_pi_4;
				float d5 = c5 + c4;
				float d6 = c6 * c_3_pi_8 - c7 * s_3_pi_8;
				float d7 = c6 * s_3_pi_8 + c7 * c_3_pi_8;
				
				float e0 = d0;
				float e1 = d1;
				float e2 = d3 - d2;
				float e3 = d3;
				float e4 = d4 + d6;
				float e5 = d7 - d5;
				float e6 = d6;
				float e7 = d5 - d7;
				
				float f0 = e0 + e3;
				float f1 = e2 + e1;
				float f2 = e1 - e2;
				float f3 = e0 - e3;
				float f4 = e4;
				float f5 = e5;
				float f6 = e6;
				float f7 = e7;
				
				float g0 = f0 + f7;
				float g1 = f1 + f6;
				float g2 = f2 + f5;
				float g3 = f3 + f4;
				float g4 = f3 - f4;
				float g5 = f2 - f5;
				float g6 = f1 - f6;
				float g7 = f0 - f7;
				
				input_block[0 * BlockSize + i] = g0;
				input_block[1 * BlockSize + i] = g1;
				input_block[2 * BlockSize + i] = g2;
				input_block[3 * BlockSize + i] = g3;
				input_block[4 * BlockSize + i] = g4;
				input_block[5 * BlockSize + i] = g5;
				input_block[6 * BlockSize + i] = g6;
				input_block[7 * BlockSize + i] = g7;
			}
			
			#pragma omp simd
			for(u32 i = 0; i < BlockSize; i++)
			{
				float b0 = input_block[i * BlockSize + 0] * a0;
				float b1 = input_block[i * BlockSize + 1] * a1;
				float b2 = input_block[i * BlockSize + 2] * a2;
				float b3 = input_block[i * BlockSize + 3] * a3;
				float b4 = input_block[i * BlockSize + 4] * a4;
				float b5 = input_block[i * BlockSize + 5] * a5;
				float b6 = input_block[i * BlockSize + 6] * a6;
				float b7 = input_block[i * BlockSize + 7] * a7;
				
				float c0 = b0 + b4;
				float c1 = b0 - b4;
				float c2 = b6 - b2;
				float c3 = (b6 + b2) * c_pi_4;
				float c4 = b1 + b7;
				float c5 = b3 - b5;
				float c6 = b3 + b5;
				float c7 = a1 - a7;
				
				float d0 = c0;
				float d1 = c1;
				float d2 = c3 - c2;
				float d3 = c3;
				float d4 = (c5 - c4) * c_pi_4;
				float d5 = c5 + c4;
				float d6 = c6 * c_3_pi_8 - c7 * s_3_pi_8;
				float d7 = c6 * s_3_pi_8 + c7 * c_3_pi_8;
				
				float e0 = d0;
				float e1 = d1;
				float e2 = d3 - d2;
				float e3 = d3;
				float e4 = d4 + d6;
				float e5 = d7 - d5;
				float e6 = d6;
				float e7 = d5 - d7;
				
				float f0 = e0 + e3;
				float f1 = e2 + e1;
				float f2 = e1 - e2;
				float f3 = e0 - e3;
				float f4 = e4;
				float f5 = e5;
				float f6 = e6;
				float f7 = e7;
				
				float g0 = f0 + f7;
				float g1 = f1 + f6;
				float g2 = f2 + f5;
				float g3 = f3 + f4;
				float g4 = f3 - f4;
				float g5 = f2 - f5;
				float g6 = f1 - f6;
				float g7 = f0 - f7;
				
				input_block[i * BlockSize + 0] = g0;
				input_block[i * BlockSize + 1] = g1;
				input_block[i * BlockSize + 2] = g2;
				input_block[i * BlockSize + 3] = g3;
				input_block[i * BlockSize + 4] = g4;
				input_block[i * BlockSize + 5] = g5;
				input_block[i * BlockSize + 6] = g6;
				input_block[i * BlockSize + 7] = g7;
			}
			
		}
	}
	
	/**
	 * Dequantize And Perform Discrete Cosine Transform On Every Macroblock
	 */
	template<IDCTAlgorithm idct_algorithm>
	static void DequantizeAndPerformIDCT(JPEGState& state)
	{
		//for every block
		#pragma omp parallel for collapse(3) shared(state)
		for(u32 v = 0; v < state.block_v; v += state.sample_factor_v)
		{
			for(u32 h = 0; h < state.block_h; h += state.sample_factor_h)
			{
				//for every component in block
				for(u32 i = 0; i < state.components_info.size(); i++)
				{
					auto& component = state.components_info[i];
					auto& quantization_table = state.quantization_tables[component.quantization_table_id];
					
					//for every subsample
					for(u32 y = 0; y < component.sample_factor_v; y++)
					{
						for(u32 x = 0; x < component.sample_factor_h; x++)
						{
							auto& block = state.macroblocks[(v + y) * state.block_padded_h + (h + x)];
							auto& current_component = block.components[i];
							
							//dequantization
							#pragma omp simd  
							for(u32 k = 0; k < BlockSize * BlockSize; k++)
							{
								current_component[k] *= quantization_table[k];
							}
							
							//IDCT
							DoIDCTOnBlock<idct_algorithm>(current_component);
						}
					}
				}
			}
		}
	}
	
	/**
	 * Read And Parse SOF0 (Start Of Frame 0 - baseline DCT) JPEG Section
	 */
	static void HandleSOF0(JPEGState& state, ByteStream& stream)
	{
		u16 frame_length   = (stream.get() << 8) | (stream.get());
		u8  bit_depth      = stream.get();
		u16 height         = (stream.get() << 8) | (stream.get());
		u16 width          = (stream.get() << 8) | (stream.get());
		u8  num_components = stream.get();
		
		std::printf("    [SOF0] bpp (%hhu), width (%hu), height (%hu), components (%hhu)\n", bit_depth, width, height, num_components);
		
		if(bit_depth != 8)
		{
			throw std::runtime_error("[SOF0] unsupported bit depth: " + std::to_string(bit_depth));
		}
		if(num_components != 3)
		{
			throw std::runtime_error("[SOF0] unsupported number of components: " + std::to_string(num_components));
		}
		
		state.bit_depth      = bit_depth;
		state.width          = width;
		state.height         = height;
		state.block_h        = state.block_padded_h = std::ceil((double)width / BlockSize);
		state.block_v        = state.block_padded_v = std::ceil((double)height / BlockSize);
		state.block_num      = state.block_padded_h * state.block_padded_v;
		
		std::vector<ComponentInfo> components(num_components);
		
		//read and parse components
		for(u32 i = 0; i < num_components; i++)
		{
			u8 component_id          = stream.get();
			u8 byte                  = stream.get();
			u8 sample_factor_v       = byte & 0b1111;
			u8 sample_factor_h       = byte >> 4;
			u8 quantization_table_id = stream.get();
			components[i] = { component_id, sample_factor_h, sample_factor_v, quantization_table_id, 0, 0 };
			
			std::printf("    [SOF0] component (%hhu), sample factor h (%hhu), sample factor v (%hhu), quantization table id (%hhu)\n", component_id, sample_factor_h, sample_factor_v, quantization_table_id);
			
			// since luma tends to be the largest component
			// we will assume it is the 0th component and
			// recalculate the padding
			if(i == 0)
			{
				if((sample_factor_h == 2 || sample_factor_h == 1) &&
				   (sample_factor_v == 2 || sample_factor_v == 1))
				{
				    state.block_padded_h += sample_factor_h == 1 ? 0 : state.block_h % 2;
				    state.block_padded_v += sample_factor_v == 1 ? 0 : state.block_v % 2;
				    state.block_num = state.block_padded_h * state.block_padded_v;
				    
				    state.sample_factor_h = sample_factor_h;
				    state.sample_factor_v = sample_factor_v;
				}
				else
				{
					throw std::runtime_error("[SOF0] luma has unsupported sample factors: h " + std::to_string(sample_factor_h) + ", v " + std::to_string(sample_factor_v));
				}
			}
			else
			{
				if(sample_factor_h != 1 || sample_factor_v != 1)
				{
					throw std::runtime_error("[SOF0] chroma has unsupported sample factors: h " + std::to_string(sample_factor_h) + ", v " + std::to_string(sample_factor_v));
				}
			}
		}
		
		state.components_info = std::move(components);
		
		frame_length -= 8 + 3 * num_components;
		
		if(frame_length != 0)
		{
			throw std::runtime_error("[SOF0] frame is corrupted");
		}
	}
	
	/**
	 * Read And Parse DQT (Define Quantization Table(s)) JPEG Section
	 */
	static void HandleDQT(JPEGState& state, ByteStream& stream)
	{
		u16 frame_length = (stream.get() << 8) | (stream.get());
		
		if(frame_length < 2)
		{
			throw std::runtime_error("[DQT] frame is corrupted");
		}
		
		frame_length -= 2;
		
		while(frame_length > 0)
		{
			u8 byte = stream.get();
			
			u8 quantization_table_id = byte & 0b1111;
			u8 precision_index       = byte >> 4;
			u8 precision = precision_index == 0 ? 1 : 2;
			
			std::printf("    [DQT] table id (%hhu), precision (%i) bits\n", quantization_table_id, precision * 8);
		
			JPEGBlock<> quantization_table;
			
			for(u32 i = 0; i < BlockSize * BlockSize; i++)
			{
				if(precision == 1)
				{
					u8 coeff = stream.get();
				}
				else
				{
					u16 coeff = (stream.get() << 8) | stream.get();
				}
				
				quantization_table[NestedZigZagIndicies[i]] = coeff;
			}
			
			state.quantization_tables[quantization_table_id] = std::move(quantization_table);
			
			std::printf("        ");
			for(u32 i = 0; i < BlockSize * BlockSize; i++)
			{
				std::printf("%i, ", state.quantization_tables[quantization_table_id][i]);
				
				if((i + 1) % BlockSize == 0)
				{
					std::printf("\n        ");
				}
			}
			std::printf("\n");
			
			if(int(frame_length) - (1 + 64 * precision) < 0)
			{
				throw std::runtime_error("[DQT] frame is corrupted");
			}
			
			frame_length -= 1 + 64 * precision;
		}
	}
	
	/**
	 * Read And Parse DHT (Define Huffman Table(s)) JPEG Section
	 */
	static void HandleDHT(JPEGState& state, ByteStream& stream)
	{
		u16 frame_length = (stream.get() << 8) | (stream.get());
		
		if(frame_length < 2)
		{
			throw std::runtime_error("[DHT] frame is corrupted");
		}
		
		frame_length -= 2;
		
		while(frame_length > 0)
		{
			HuffmanTable table;
			
			u8 byte = stream.get();
			u8 type = byte >> 4;
			u8 dest_id = byte & 0b1111;
			
			std::printf("    [DHT] - create table type (%u), dest id (%u)\n", type, dest_id);
			
			u32 sizes_sum = 0;
			
			std::printf("        code sizes: ");
			
			//read code sizes
			for(u32 i = 0; i < 16; i++)
			{
				table.code_sizes[i] = stream.get();
				sizes_sum += table.code_sizes[i];
				std::printf("%hu, ", table.code_sizes[i]);
			}
			std::printf("\n        values:\n            ");
									
			//read values
			for(u32 i = 0; i < 16; i++)
			{
				u16 size = table.code_sizes[i];
				
				if(size == 0)
				{
					continue;
				}
				
				std::printf("(%u) - ", i);
				for(u32 j = 0; j < size; j++)
				{
					table.values.push_back(stream.get());
					std::printf("%hhu, ", table.values.back());
					
					if((j + 1) % BlockSize == 0)
					{
						std::printf("\n                   ");
					}
				}
				std::printf("\n            ");
			}
			std::printf("\n        codes:\n            ");
			
			//generate codes
			u32 code = 0;
			for(u32 i = 0; i < table.code_sizes.size(); i++)
			{
				u16 size = table.code_sizes[i];
				
				std::printf("(%u) - ", i);
				for(u32 j = 0; j < size; j++)
				{
					table.codes.push_back(code++);
					std::printf("0x%x, ", table.codes.back());
					
					if((j + 1) % BlockSize == 0)
					{
						std::printf("\n                   ");
					}
				}
				std::printf("\n            ");
				code <<= 1;
			}
			std::printf("\n");

			//set table
			if(type == 0)
			{
				state.dc_huffman_tables[dest_id] = table;
			}
			else
			{
				state.ac_huffman_tables[dest_id] = table;
			}
			
			if(int(frame_length) - (1 + 16 * sizes_sum) < 0)
			{
				throw std::runtime_error("[DHT] frame is corrupted");
			}
			
			frame_length -= 1 + 16 + sizes_sum;
		}
	}
	
	/**
	 * Read And Parse DRI (Define Restart Interval) JPEG Section
	 */
	static void HandleDRI(JPEGState& state, ByteStream& stream)
	{
		u16 frame_length   = (stream.get() << 8) | (stream.get());
		
		if(frame_length != 4)
		{
			throw std::runtime_error("[DRI] frame is corrupted");
		}
		
		u16 reset_interval = (stream.get() << 8) | (stream.get());
		
		std::printf("    [DRI] %hu\n", reset_interval);
		
		state.reset_interval = state.reset_counter = reset_interval;
	}
	
	/**
	 * Read And Parse SOS (Start Of Scan) JPEG Section
	 */
	static void HandleSOS(JPEGState& state, ByteStream& stream)
	{	
		u16 frame_length   = ((stream.get() << 8) | stream.get());
		
		if(frame_length < 2)
		{
			throw std::runtime_error("[SOS] frame is corrupted");
		}
		
		frame_length -= 2;
		
		u8 num_components = stream.get();
		
		std::printf("    [SOS] num of components (%hhu)\n", num_components);
		
		if(num_components != state.components_info.size())
		{
			throw std::runtime_error("[SOS] number of components doesn't match, read: " + std::to_string(num_components) + ", parsed: " + std::to_string(state.components_info.size()));
		}

		for(u32 i = 0; i < num_components; i++)
		{
			u8 component_id = stream.get();
			u8 byte         = stream.get();
			u8 dc_dest_id   = byte >> 4;
			u8 ac_dest_id   = byte & 0b1111;
			
			if(state.components_info[i].id != component_id)
			{
				throw std::runtime_error("[SOS] components are out of order");
			}
			
			state.components_info[i].dc_dest_id = dc_dest_id;
			state.components_info[i].ac_dest_id = ac_dest_id;
			
			std::printf("    [SOS] component id (%hhu), ac huffman id (%hhu), dc huffman id (%hhu)\n", component_id, ac_dest_id, dc_dest_id);
		}
		
		u8 spectral_selection_start  = stream.get();
		u8 spectral_selection_end    = stream.get();
		u8 spectral_selection_approx = stream.get();
	
		if(spectral_selection_start  != 0  ||
		   spectral_selection_end    != 63 ||
		   spectral_selection_approx != 0)
		{
			throw std::runtime_error("[SOS] unsupported spectral selection ss: " + 
			                          std::to_string(spectral_selection_start) + " se: " +
									  std::to_string(spectral_selection_end)   + " sa: " + 
									  std::to_string(spectral_selection_approx));
		}
	
		frame_length -= 4 + 2 * num_components;
		
		if(frame_length != 0)
		{
			throw std::runtime_error("[SOS] frame is corrupted");
		}
	}
	
	/**
	 * Skip Any Kind Of APP (Application Specific Data) JPEG Section
	 */
	static void HandleAPPN(JPEGState& state, ByteStream& stream)
	{
		u16 frame_length = (stream.get() << 8) | stream.get();
		
		if(frame_length < 2)
		{
			throw std::runtime_error("[APP] corrupted frame");
		}
		
		stream.skip(frame_length - 2);

		(void)state;
	}
	
	/**
	 * Decode JPEG Formatted Data
	 */
	std::optional<Utils::Image> Decode(const std::vector<u8>& data, IDCTAlgorithm idct_algorithm/* = IDCTAlgorithm::CompoundLut*/)
	{
		Utils::Image output;
		Utils::Timer timer;
		JPEGState    state;
		ByteStream   stream { data };

		u8   buffer;
		bool parsing = true;
		
		const char* idct_algo_map[] =
		{
			"reference",
			"compound",
			"walsh",
			"reference-lut",
			"compound-lut",
			"arai"
		};
		
		try
		{
			while(parsing)
			{
				// find marker
				if((buffer = stream.get()) != 0xFF)
				{
					continue;
				}
				if((buffer = stream.get()) == 0x00)
				{
					continue;
				}
				
				auto marker = Markers.find(buffer);
				if(marker == Markers.end())
				{
					std::printf("[Warning] unsupported marker: 0xFF%02x\n", marker->first);
					continue;
				}
				
				// parse marker
				switch(marker->second)
				{
					// handle start of image (change state of decoding)
					case JPEGSectionMarker::SOI:
					{
						std::printf("[Start Of Image]\n");
						if(state.state == JPEGState::State::Uninitialized)
						{
							state.state = JPEGState::State::ParsingHeader;
						}
						else
						{
							throw std::runtime_error("Encountered unexpected SOI frame");
						}
						break;
					}
				
					// handle end of image (this marker should be captured by LoadHuffmanStream)
					case JPEGSectionMarker::EOI:
					{
						throw std::runtime_error("Encoutered unexpected EOI frame");
						break;
					}
					
					// handle app markers (just read their size and skip them)
					case JPEGSectionMarker::APP0:					
					case JPEGSectionMarker::APP1:
					case JPEGSectionMarker::APP2:
					case JPEGSectionMarker::APP3:
					case JPEGSectionMarker::APP4:
					case JPEGSectionMarker::APP5:
					case JPEGSectionMarker::APP6:
					case JPEGSectionMarker::APP7:
					case JPEGSectionMarker::APP8:
					case JPEGSectionMarker::APP9:
					case JPEGSectionMarker::APP10:
					case JPEGSectionMarker::APP11:
					case JPEGSectionMarker::APP12:
					case JPEGSectionMarker::APP13:
					case JPEGSectionMarker::APP14:
					case JPEGSectionMarker::APP15:
					{
						std::printf("[Application Specific Data]\n");
						HandleAPPN(state, stream);
						std::printf("    [APP] type (0xff%02x)\n", marker->first);
						break;
					}
					
					// handle jpg baseline dct mode
					case JPEGSectionMarker::SOF0:
					case JPEGSectionMarker::SOF1:
					{
						std::printf("[Start Of Frame]\n");
						
						if(state.state != JPEGState::State::ParsingHeader)
						{
							throw std::runtime_error("Unexpected SOF frame");
						}
						
						HandleSOF0(state, stream);
						break;
					}
					
					// handle other jpg modes (unimplemented)
					case JPEGSectionMarker::SOF2:
					case JPEGSectionMarker::SOF3:
					case JPEGSectionMarker::SOF5:
					case JPEGSectionMarker::SOF6:
					case JPEGSectionMarker::SOF7:
					case JPEGSectionMarker::SOF9:
					case JPEGSectionMarker::SOF10:
					case JPEGSectionMarker::SOF11:
					case JPEGSectionMarker::SOF12:
					case JPEGSectionMarker::SOF13:
					case JPEGSectionMarker::SOF14:
					case JPEGSectionMarker::SOF15:
					{
						throw std::runtime_error("Unsupported SOF format: " + std::to_string(marker->first));
						break;
					}
					
					// handle definition of quantization table
					case JPEGSectionMarker::DQT:
					{
						std::printf("[Define Quantization Table]\n");
						
						if(state.state != JPEGState::State::ParsingHeader)
						{
							throw std::runtime_error("Unexpected SOF frame");
						}
						
						HandleDQT(state, stream);
						break;
					}
					
					// handle definition of huffman table
					case JPEGSectionMarker::DHT:
					{
						std::printf("[Define Huffman Table]\n");
						
						if(state.state != JPEGState::State::ParsingHeader)
						{
							throw std::runtime_error("Unexpected SOF frame");
						}
						
						HandleDHT(state, stream);
						break;
					}
					
					// handle definition of restart interval
					case JPEGSectionMarker::DRI:
					{
						std::printf("[Define Restart Interval]\n");
						
						if(state.state != JPEGState::State::ParsingHeader)
						{
							throw std::runtime_error("Unexpected SOF frame");
						}
						
						HandleDRI(state, stream);
						break;
					}
					
					// handle start of scan
					case JPEGSectionMarker::SOS:
					{
						std::printf("[Start Of Scan]\n");
						
						if(state.state != JPEGState::State::ParsingHeader)
						{
							throw std::runtime_error("Unexpected SOS frame");
						}
						
						std::printf("    [SOS-timer] algo: %s\n", idct_algo_map[u32(idct_algorithm)]);
						
						HandleSOS(state, stream);
						
						state.state = JPEGState::State::ParsingHuffman;
						
						std::printf("    [SOS-timer] Loading Huffman Stream:      ");
						timer.start();
						LoadHuffmanStream(state, stream);
						std::printf("%u ms\n", timer.end());
						
						std::printf("    [SOS-timer] Decoding Huffman Stream:     ");
						timer.start();
						DecodeHuffmanStream(state);
						std::printf("%u ms\n", timer.end());
						
						state.state = JPEGState::State::Decoding;
						
						std::printf("    [SOS-timer] Dequantize And Perform IDCT: ");
						timer.start();
						switch(idct_algorithm)
						{
							case IDCTAlgorithm::Reference:     { DequantizeAndPerformIDCT<IDCTAlgorithm::Reference>(state);     break; }
							case IDCTAlgorithm::Compound:      { DequantizeAndPerformIDCT<IDCTAlgorithm::Compound>(state);      break; }
							case IDCTAlgorithm::WalshHadamard: { DequantizeAndPerformIDCT<IDCTAlgorithm::WalshHadamard>(state); break; }
							case IDCTAlgorithm::ReferenceLut:  { DequantizeAndPerformIDCT<IDCTAlgorithm::ReferenceLut>(state);  break; }
							case IDCTAlgorithm::CompoundLut:   { DequantizeAndPerformIDCT<IDCTAlgorithm::CompoundLut>(state);   break; }
							case IDCTAlgorithm::Arai:          { DequantizeAndPerformIDCT<IDCTAlgorithm::Arai>(state);          break; }
						}
						std::printf("%u ms\n", timer.end());
						
						std::printf("    [SOS-timer] Convert YCbCr to RGB:        ");
						timer.start();
						ConvertToRGB(state);
						std::printf("%u ms\n", timer.end());
						
						std::printf("    [SOS-timer] Construct RGB image:         ");
						timer.start();
						output = ConstructImage(state);
						std::printf("%u ms\n", timer.end());
						std::printf("    [SOS-timer] Total time:                  %u ms\n", timer.total());
						parsing = false;
						break;
					}
					
					default:
					{
						std::printf("[Warning] Unsupported marker: 0xff%02x\n", buffer);
						break;
					}
				}
			}
		} 
		catch(std::out_of_range& e)
		{
			std::printf("JPEG data stream ended unexpectedly\n");
			std::printf("%s\n", e.what());
			return {};
		}
		catch(std::runtime_error& e)
		{
			std::printf("%s\n", e.what());
			return {};
		}
		
		return output;
	}
	
	/**
	 * Load JPEG Image From File
	 */
	std::optional<Utils::Image> DecodeFromFile(const std::string& file_path, IDCTAlgorithm idct_algorithm/* = IDCTAlgorithm::CompoundLut*/)
	{
		std::ifstream input(file_path, std::ios::binary);
		
		if(!input.is_open())
		{
			std::printf("Input file couldn't be opened\n");
			return {};
		}
		
		std::vector<u8> image_data(std::istreambuf_iterator<char>(input), {});
		return Decode(image_data, idct_algorithm);
	}
};