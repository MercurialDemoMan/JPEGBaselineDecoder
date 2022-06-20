# JPEG to PPM converter

### Requirements

- GNU make 4.2.1

- clang 10.0.0

### Compilation

Compile using makefile: 

	make

Run using: 

	./jpg2ppm --output samples/sample0.ppm --algorithm comlut samples/sample0.jpg

Possible inverse discrite cosine transform algorithms:

- ***ref*** = naive O(n^4) 2D idct algorithm

- ***com*** = compound of O(n^3) 1D idct algorithms

- ***walsh*** = idct approximation using Walsh-Hadamard transform

- ***reflut*** = naive O(n^4) 2D idct algorithm with precomputed frequencies

- ***comlut*** = compound of O(n^3) 1D idct algorithms with precomputed frequencies *(default)*

- ***arai*** = fast implementation of idct using Arai, Agui, and Nakajima (AAN) algorithm *(broken)*
