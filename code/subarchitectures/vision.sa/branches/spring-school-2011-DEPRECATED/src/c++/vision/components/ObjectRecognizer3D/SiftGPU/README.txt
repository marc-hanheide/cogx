A GPU implementation of David Lowe's Scale Invariant Feature Transform

Changchang wu

http://cs.unc.edu/~ccwu

University of North Carolina at Chapel Hill




1. SIFT 

	SIFTGPU is an implementation of SIFT for GPU. SiftGPU uses GPU to process pixels and features 
	parallely in Gaussian pyramid construction, DoG keypoint detection and descriptor generation 
	for SIFT. Compact feature list is efficiently build through a GPU/CPU mixed reduction.  

	SIFTGPU is inspired by Andrea Vedaldi's sift++ and Sudipta N Sinha et al's GPU-SIFT. Many 
	parameters of sift++ ( for example, number of octaves,number of DOG levels, edge threshold,
	etc) are available in SiftGPU. 
	

	SIFTGPU also includes a GPU exhaustive/guided sift matcher SiftMatchGPU. It basically multiplies 
	the descriptor matrix on GPU and find closest feature matches on GPU.  GLSL/CUDA/CG implementations
	are all provided. 
    
    NEW: The latest SIFTGPU also enables you to use Multi-GPUs and GPUS on different computers.
	Check doc/manual.pdf for more information. You can modify some marcros definition in 
	SimpleSIFT.cpp and speed.cpp to enable the testing of the new functions. 
    

2. Requirements

	The default implemntation uses GLSL, and it requires a GPU that has large memory and supports
	dynamic branching. For nVidia graphic cards, you can optionally use CG(require fp40) and the 
	CUDA (experimental) implementation. Note that, I haven't tested on any ATI card yet. 

	For the CUDA version, it is not compiled by default. You need to compile the package with macro 
	CUDA_SIFTGPU_ENABLED to	enable the functionality. You should also try different implementations and
	select the fastest one for different image sizes and parameters. 

	SiftGPU uses CG, DevIl Image library, GLEW and GLUT. You'll need to make sure your system has
	all the dependening libraries. SiftGPU should be able to run on any operation system that supports 
	the above libraries

	For windows system visual studio solution are provided as VC/SiftGPU.dsw and VC/SiftGPU.sln.
	Linux makefile is in folder Linux of the package, and xcode project is in the Mac folder. 


3. Helps 

	Use -help to get parameter information. Check /doc/manual.pdf for samples and explanations. 
	In the vc workspace, there is a project called SimpleSIF that gives an example of simple 
	SiftGPU usage. There are more examples of different ways of using SiftGPU in manual.pdf 


	Check /doc/manual.pdf for help on the viewer. 

