#include <cstdio>

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

const int BlockSize = 8;

int main()
{
   auto DCTBase1D = [](int k, int n) -> double
	{
		double lambda_k = k == 0 ? 1.0 / std::sqrt(2.0) : 1.0;
		return lambda_k * std::sqrt(2.0 / BlockSize) * std::cos((k * M_PI / BlockSize) * (n + 0.5));
	};

    for(int n = 0; n < BlockSize; n++)
	{
		for(int k = 0; k < BlockSize; k++)
		{
			std::printf("%lf,\n", DCTBase1D(k, n));
		}
	}
}