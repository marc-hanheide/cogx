Problems:

32Bit System:
Matlab has problems with glnx86/libstdc++.so.6 matlab GLIBCXX_3.4.11
- Have a look on http://www.mathworks.de/matlabcentral/newsreader/view_thread/162466
- Or 
	cd matlab/sys/os/glnx86/
	mv libgcc_s.so.1 libgcc_s.so.1.back
	ln -s /lib/libgcc_s.so.1 libgcc_s.so.1
	rm libstdc++.so.6
	ln -s /usr/lib/libstdc++.so.6 libstdc++.so.6


64Bit System:
Matlab has problems with glnx64/libstdc++.so.6 matlab GLIBCXX_3.4.11
- Have a look on http://www.mathworks.de/matlabcentral/newsreader/view_thread/162466
- Or 
	cd matlab/sys/os/glnxa64/
	mv libgcc_s.so.1 libgcc_s.so.1.back
	ln -s /lib/libgcc_s.so.1 libgcc_s.so.1
	rm libstdc++.so.6
	ln -s /usr/lib/libstdc++.so.6 libstdc++.so.6

After you have done this, you have to recompile the mex files and restart matlab!!
