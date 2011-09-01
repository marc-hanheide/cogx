Environment Variables

You have to set the following environment variables a good place is the .profiles file

#Aldebaran
export AL_DIR="$HOME/opt/naoqi"
export PYTHONPATH="$PYTHONPATH:${AL_DIR}/lib"

#Nao2matlab project
export NAO_DIR="$HOME/......../nao2matlab"


Matlab add the libs to your search path LD_LIBRARY_PATH in matlab
on my matlab 7.9.0 under Ubuntu I added at line 1336
LD_LIBRARY_PATH=$AL_DIR/lib:$NAO_DIR/lib:$LD_LIBRARY_PATH
or start matlab from a bash where you set the the LD_LIBRARY_PATH or use the in .bashrc
export LD_LIBRARY_PATH=$AL_DIR/lib:$NAO_DIR/lib:$LD_LIBRARY_PATH



	
Problems:
Matlab has problems with glnx86/libstdc++.so.6 matlab GLIBCXX_3.4.11
- Have a look on http://www.mathworks.de/matlabcentral/newsreader/view_thread/162466
- Or 
	cd matlab/sys/os/glnx86/
	mv libgcc_s.so.1 libgcc_s.so.1.back
	ln -s /lib/libgcc_s.so.1 libgcc_s.so.1
	rm libstdc++.so.6 (this was already a symbolic link)
	ln -s /usr/lib/libstdc++.so.6 libstdc++.so.6
Ubuntu > 9.x 
- Ubuntu removed for security reasons the possibility to set the LD_LIBRARY_PATH via .profile or /etc/environemt
  you have to set it in your .bashrc or /etc/ld..... somewhere or in the Matlab start script

Author: Markus Bader 2010
email: markus.bader@tuwien.ac.at

