#pragma once

#include "Utils.hpp"

#include <vector>

class ByteStream
{
public:
	ByteStream(const std::vector<u8>& data);

	u8 operator[](u32 index) const;
	
	u8 get();
	
	void skip(u32 bytes);
	void seek(u32 pos);
	
private:
	const std::vector<u8>& m_data;
	u32                    m_pos { 0 };
};