all:
	(cd testing; make all)
#	(cd lib; make all)

testing:
	(cd testing; make all)

lib:
	(cd lib; make all)

opencv:
	(cd opencv; git fetch --all --tags --prune; git checkout tags/3.2.0; mkdir build; cd build; cmake -DCMAKE_BUILD_TYPE=Release ..; make -j7)

clean:
	(cd testing; make clean)
	(cd lib; make clean)
