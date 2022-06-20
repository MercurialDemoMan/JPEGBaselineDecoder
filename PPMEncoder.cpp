#include "PPMEncoder.hpp"

#include <iostream>
#include <fstream>

namespace PPMEncoder
{
	std::vector<u8> Encode(const Utils::Image& image)
	{
		std::vector<u8> ppm_data;
		ppm_data.push_back('P');
		ppm_data.push_back('6');
		ppm_data.push_back(' ');
		
		std::string width = std::to_string(image.width);
		for(u32 i = 0; i < width.size(); i++)
		{
			ppm_data.push_back(width[i]);
		}
		ppm_data.push_back(' ');
		
		std::string height = std::to_string(image.height);
		for(u32 i = 0; i < height.size(); i++)
		{
			ppm_data.push_back(height[i]);
		}
		ppm_data.push_back(' ');
		
		ppm_data.push_back('2');
		ppm_data.push_back('5');
		ppm_data.push_back('5');
		ppm_data.push_back(' ');
		
		// write pixel data
		for(unsigned int i = 0; i < image.width * image.height; i++)
		{
			ppm_data.push_back(image.pixels[i].r);
			ppm_data.push_back(image.pixels[i].g);
			ppm_data.push_back(image.pixels[i].b);
		}
		
		return ppm_data;
	}
	
	void EncodeToFile(const Utils::Image& image, const std::string& file_path)
	{
		std::ofstream output(file_path, std::ios::binary);
		
		auto ppm_data = Encode(image);
		
		output.write((const char*)(ppm_data.data()), ppm_data.size());
		
		output.close();
	}
}