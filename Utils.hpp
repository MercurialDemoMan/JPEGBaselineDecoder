#pragma once

#include <cassert>
#include <cstdint>
#include <vector>
#include <chrono>
#include <stdexcept>

using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;

using s32 = int32_t;

namespace Utils
{
	/**
	 * RGB/YCbCr Color structure
	 */
	struct Color
	{
		union
		{
			int r;
			int y;
		};
		
		union
		{
			int g;
			int cb;
		};
		
		union
		{
			int b;
			int cr;
		};
	};
	
	static_assert(sizeof(Color) == sizeof(int) * 3);
	
	/**
	 * simple image structure
	 */
	struct Image
	{
		u32 width;
		u32 height;
		
		std::vector<Color> pixels;
	};
	
	/**
	 * Convert ycbcr to rgb color space
	 */
	Color YCbCrToRGB(const Color& color);
	
	/**
	 * Timer for measuring performance
	 */
	template<typename T = std::chrono::milliseconds>
	class Timer
	{
	public:
	
		Timer()
		{
			
		}
		
		void start()
		{
			m_start = std::chrono::high_resolution_clock::now();
		}
		
		u32 end()
		{
			auto result = std::chrono::duration_cast<T>(std::chrono::high_resolution_clock::now() - m_start).count();
			m_total += result;
			return result;
		}
		
		u32 total()
		{
			return m_total;
		}
		
	private:
		
		std::chrono::time_point<std::chrono::high_resolution_clock> m_start;
		u32  m_total { 0 };
	};
};