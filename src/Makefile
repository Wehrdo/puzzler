all:
	(cd testing; make all)
#	(cd lib; make all)

testing:
	(cd testing; make all)

lib:
	(cd lib; make all)

opencv:
	(cd opencv; git fetch --all --tags --prune; git checkout tags/3.2.0; cd ..)
	(cd opencv_contrib; git fetch --all --tags --prune; git checkout tags/3.2.0; cd ..)
	(cd opencv; mkdir build; cd build; cmake -DCMAKE_BUILD_TYPE=Release -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DBUILD_opencv_dnn=OFF -DBUILD_opencv_cnn_3dobj=OFF ..; make -j7)

clean:
	(cd testing; make clean)
	(cd lib; make clean)
	(cd opencv; rm -rf build)
