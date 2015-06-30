# OSM admin bounds extractor

This creates the grapml based input format for line simplification algorithm tests.

# Build requirements

* libosmium
* OSMPBF
* proj4

# Compiling

	mkdir build
	cd build
	cmake .. -DCMAKE_BUILD_TYPE=Release
	make -j
	./osm-admin-bounds input.osm.pbf

This will create a file "input.osm.txt" with an input format suitable for [my line simplifcation project](https://github.com/TheMarex/deberg).


