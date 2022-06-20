#include "ByteStream.hpp"



ByteStream::ByteStream(const std::vector<u8>& data) : m_data(data)
{
	
}
	
u8 ByteStream::operator[](u32 index) const
{
	return m_data[index];
}
		
u8 ByteStream::get()
{
	return m_data[m_pos++];
}

void ByteStream::skip(u32 bytes)
{
	m_pos += bytes;
}

void ByteStream::seek(u32 pos)
{
	m_pos = pos;
}