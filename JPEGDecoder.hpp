#pragma once

#include "Utils.hpp"

#include <string>
#include <vector>
#include <optional>

namespace JPEGDecoder
{
	/**
	 * algorithm for IDCT
	 */
	enum class IDCTAlgorithm
	{
		Reference,     // slowest
		Compound,      //
		WalshHadamard, // 
		ReferenceLut,  //
		CompoundLut,   // fastest
		
		Arai           // experimental
	};
	
	std::optional<Utils::Image> DecodeFromFile(const std::string& file_path, IDCTAlgorithm idct_algorithm = IDCTAlgorithm::CompoundLut);
	std::optional<Utils::Image> Decode(const std::vector<u8>& data, IDCTAlgorithm idct_algorithm = IDCTAlgorithm::CompoundLut);
};