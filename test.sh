#!/bin/bash

make -j

for i in samples/*.jpg; do
	echo "$i"
	./jpg2ppm.exe -a ref    -o "$i"ref.ppm "$i" | grep "timer"
	./jpg2ppm.exe -a com    -o "$i"com.ppm "$i" | grep "timer"
	./jpg2ppm.exe -a walsh  -o "$i"wal.ppm "$i" | grep "timer"
	./jpg2ppm.exe -a reflut -o "$i"rlt.ppm "$i" | grep "timer"
	./jpg2ppm.exe -a comlut -o "$i"clt.ppm "$i" | grep "timer"
	./jpg2ppm.exe -a arai   -o "$i"ara.ppm "$i" | grep "timer"
done