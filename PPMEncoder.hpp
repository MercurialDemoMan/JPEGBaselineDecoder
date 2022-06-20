#pragma once

#include "Utils.hpp"
#include <vector>
#include <string>

/**
 * simple PPM image encoder
 */
namespace PPMEncoder
{
	std::vector<u8> Encode(const Utils::Image& image);
	void EncodeToFile(const Utils::Image& image, const std::string& file_path);
};