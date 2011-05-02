#ifndef NEURONICS_VERSION_H_
#define NEURONICS_VERSION_H_

//#include <string>

class NeuronicsVersion
{
public:

private:

	std::string mName;
	std::string mVersion;
	std::string mBuildDate;
	std::string mBuildUser;
	std::string mCompiler;

public:
	
	NeuronicsVersion ()
	{
		mName       = "NEURONICS_NAME";
   		mVersion    = "NEURONICS_VERSION";
   		mBuildDate  = "NEURONICS_BUILD_DATE";
   		mBuildUser  = "NEURONICS_BUILD_USER";
   		mCompiler   = "NEURONICS_COMPILER";
	}

	std::string getVersion ()
	{
		return "module name:\t" + mName      + "\n" +
    		   "version:\t"     + mVersion   + "\n" +
    		   "build date:\t"  + mBuildDate + "\n" +
    		   "build user:\t"  + mBuildUser + "\n" +
    		   "compiler:\t"    + mCompiler;
	}
};


#endif /*NEURONICS_VERSION_H_*/
