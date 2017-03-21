all:
	(cd testing; make all)
#	(cd lib; make all)

testing:
	(cd testing; make all)

lib:
	(cd lib; make all)

opencv:
	(unzip opencv-3.2.0; cd opencv-3.2.0;mkdir build; cd build; cmake -DCMAKE_BUILD_TYPE=Release ..; make -j7)

clean:
	(cd testing; make clean)
	(cd lib; make clean)
