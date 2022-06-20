#include "Utils.hpp"
#include "JPEGDecoder.hpp"
#include "PPMEncoder.hpp"

#include "argparse.hpp"

int main(int argc, const char* argv[])
{
	//parse arguments
	ArgumentParser args;
	args.addArgument("-o", "--output", 1, true);
	args.addArgument("-a", "--algorithm", 1, true);
	args.addFinalArgument("input");
	try
	{
		args.parse(argc, argv);
	}
	catch(const std::runtime_error& e)
	{
		std::printf("%s\n", e.what());
		return 1;
	}
	
	//convert algorithm specification into enum
	JPEGDecoder::IDCTAlgorithm idct_type = JPEGDecoder::IDCTAlgorithm::CompoundLut;
	
	std::string idct_type_string = args.retrieve<std::string>("algorithm");
	
	if(!idct_type_string.empty())
	{
		if(idct_type_string == "ref")
		{
			idct_type = JPEGDecoder::IDCTAlgorithm::Reference;
		}
		else if(idct_type_string == "com")
		{
			idct_type = JPEGDecoder::IDCTAlgorithm::Compound;
		}
		else if(idct_type_string == "walsh")
		{
			idct_type = JPEGDecoder::IDCTAlgorithm::WalshHadamard;
		}
		else if(idct_type_string == "reflut")
		{
			idct_type = JPEGDecoder::IDCTAlgorithm::ReferenceLut;
		}
		else if(idct_type_string == "comlut")
		{
			idct_type = JPEGDecoder::IDCTAlgorithm::CompoundLut;
		}
		else if(idct_type_string == "arai")
		{
			idct_type = JPEGDecoder::IDCTAlgorithm::Arai;
		}
		else
		{
			std::printf("bad idct algorithm");
			return 1;
		}
	}
	
	std::string input_path = args.retrieve<std::string>("input");
	std::string output_path = args.retrieve<std::string>("output");
	
	std::optional<Utils::Image> decoded_image = JPEGDecoder::DecodeFromFile(input_path, idct_type);
	
	if(!output_path.empty())
	{
		if(decoded_image.has_value())
		{
			PPMEncoder::EncodeToFile(decoded_image.value(), output_path);
		}
		else
		{
			return 1;
		}
	}
	
	return 0;
}