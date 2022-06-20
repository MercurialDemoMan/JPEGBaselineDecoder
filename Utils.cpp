#include "Utils.hpp"

#include <cmath>
#include <algorithm>

namespace Utils
{
	Color YCbCrToRGB(const Color& color)
	{
		int r = color.y + 1.402 * color.cr + 128;
		int g = color.y - 0.344 * color.cb - 0.714 * color.cr + 128;
		int b = color.y + 1.772 * color.cb + 128;
		
		return Color 
		{
			{ std::min(std::max(0, r), 255) },
			{ std::min(std::max(0, g), 255) },
			{ std::min(std::max(0, b), 255) }
		};
	}
};