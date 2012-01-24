/*
* This program match the number of support vector from two different files.
* The file could be either .dat file or .modle file
*
* Author:
*   Roger
*   jiel@nada.kth.se
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFFERSIZE 10000000
#define MAXPATH 1024

bool flagSaveResult = false;
bool flagMap = false;

/**Check if the two vectors containing in the input lines match*/
bool vectorMatch(char* line1, char* line2)
{
	char COLON[]=":";
	char SPACE[]=" ";
   char END[]="\n";
	char *pdest1,*pdest2;
	char *StringPtr1 = &line1[0];
	char *StringPtr2 = &line2[0];
	
	/**Find the start point of the vector*/ 
	int indexCOLON,indexSPACE;
   /**Find the end point of the vector*/
   int indexEND1,indexEND2,cmpN;

	/**Line 1*/
	indexCOLON = strcspn(StringPtr1,COLON);
	indexSPACE = strcspn(StringPtr1,SPACE);
	while ( indexCOLON > indexSPACE )
	{
		pdest1 = strstr(StringPtr1,SPACE);
		StringPtr1 = pdest1+1;
		indexCOLON = strcspn(StringPtr1,COLON);
		indexSPACE = strcspn(StringPtr1,SPACE);
	}
	indexEND1 = strcspn(StringPtr1,END);

	/**Line 2*/
	indexCOLON = strcspn(StringPtr2,COLON);
	indexSPACE = strcspn(StringPtr2,SPACE);
	while ( indexCOLON > indexSPACE )
	{
		pdest2 = strstr(StringPtr2,SPACE);
		StringPtr2 = pdest2+1;
		indexCOLON = strcspn(StringPtr2,COLON);
		indexSPACE = strcspn(StringPtr2,SPACE);
	}
	indexEND2 = strcspn(StringPtr2,END);

   if( (indexEND1-indexEND2)>2 || (indexEND1-indexEND2)<-2 )	
   	return false;
   else
   {
      if( indexEND1>= indexEND2 )
        cmpN = indexEND2 - 1;
      else
        cmpN = indexEND1 - 1;
   }

	
	if( strncmp(StringPtr1,StringPtr2,cmpN) == 0 )
		return true;
	else
		return false; 
}

/**Check the string is a vector or head informaiton of model file*/
bool vectorCheck(char* line)
{
	char COLON = ':';
	char *StringPtr = &line[0];
	if( strchr(StringPtr,COLON) != NULL )
		return true;
	else
		return false;
}

void exitWithHelp()
{
  printf("usage: svm-match <parameters> model_file model_or_data_file\n"
         "options:\n"
         "-s <match_result> : save the match result to the specified filename\n"
         "-m <map_to_instance>: Give an instance list file of the corresponding data.\n"
	 "                      The program will output a list of the mapped instances.\n"
	 "                      Use to track the where the SVs come from.\n"
	 "                      Requirement: The instance list should have the same order\n"
	 "                                   as the data file.\n"
       );
  exit(1);
}

void parseCommandLine(int argc, char** argv, char* modelFileName, char* dataFileName, char* outputFileName, char* listFileName)
{
	int i;

	/**parse options*/
	for(i=1; i<argc; i++)
	{
		if(argv[i][0] != '-') break;
		++i;
		switch(argv[i-1][1])
		{
		    case 's':
			flagSaveResult = true;
			strcpy(outputFileName, argv[i]);
			break;
		    case 'm':
			flagMap = true;
			strcpy(listFileName, argv[i]);
			break;
		    default:
			fprintf(stderr,"unknown option\n");
			exitWithHelp();
		}
	}


	/**Determine Operation File Name*/
	if(i+2!=argc)
		exitWithHelp();

	strcpy(modelFileName,argv[i]);
	strcpy(dataFileName,argv[i+1]);
}
int main(int argc, char **argv)
{
	FILE *stream1,*stream2;
	FILE *output=0;
	FILE *list=0;

	char matchFileName1[MAXPATH];
	char matchFileName2[MAXPATH];
	char outputFileName[MAXPATH];
	char listFileName[MAXPATH];
	
	parseCommandLine(argc,argv,matchFileName1,matchFileName2,outputFileName,listFileName);

	if( matchFileName1 == NULL || matchFileName2 == NULL)
		exitWithHelp();
	if( flagSaveResult == true && outputFileName == NULL)
		exitWithHelp();

	/**Read Input Model and Data File*/
	printf("Reading file 1: %s\n",matchFileName1);
	stream1 = fopen(matchFileName1,"r");
	if( stream1==NULL )
	{
		fprintf(stderr,"Can't not open target model or data file: %s \n",matchFileName1);
		exit(1);
	}
	printf("Reading file 2: %s\n",matchFileName2);
	stream2	= fopen(matchFileName2,"r");
	if( stream2==NULL )
	{
		fprintf(stderr,"Can't not open target model or data file: %s \n",matchFileName2);
		exit(1);
	}

	/**Create Output Result File*/	
	if(flagSaveResult == true)
	{
		output = fopen(outputFileName,"w");
		if(output == NULL)
		{
			fprintf(stderr,"Can't not create output file: %s \n",outputFileName);
			exit(1);
		}
	}

	/**Read List File*/	
	if(flagMap == true)
	{	
		list = fopen(listFileName,"r");
		if(list == NULL)
		{
			fprintf(stderr,"Can't not open list file: %s \n",listFileName);
			exit(1);
		} 
	}

	int counter = 0;
	int svFile1=0,svFile2=0;
	bool flagMatch;	

	char *StringBuffer1 = new char[BUFFERSIZE];
	char *StringBuffer2 = new char[BUFFERSIZE];
	char *VectorBuffer = new char[MAXPATH];

	fgets(StringBuffer1,BUFFERSIZE,stream1);
	/**Find the first line contain vector*/
	while( !vectorCheck(StringBuffer1) )
		fgets(StringBuffer1,BUFFERSIZE,stream1);
	
	printf("Matching...\n");

	if(flagMap == true)
	{
		printf("Images Which Become A Suppor Vector:\n"); 
		if(flagSaveResult == true)
			fprintf(output,"Images Which Become A Suppor Vector:\n");
	}

	while(!feof(stream1))
	{
		svFile1++;

		rewind(stream2);
		fgets(StringBuffer2,BUFFERSIZE,stream2);
		/**Find the first line contain vector*/
		while( !vectorCheck(StringBuffer2) )
			fgets(StringBuffer2,BUFFERSIZE,stream2);
		
		if(flagMap == true)
		{
			rewind(list);
			fgets(VectorBuffer,MAXPATH,list);	
		}

		flagMatch = false;
		
		int j=0;	
		
		while( !feof(stream2) && !flagMatch )
		{
			j++;
			if( vectorMatch(StringBuffer1,StringBuffer2) )
			{
				counter ++;
				flagMatch = true;
				if(flagMap == true)
				{
					printf("vector:%s",VectorBuffer);
					if(flagSaveResult == true)
						fprintf(output,"%s",VectorBuffer);
				}
				
			}
			else
			{
				fgets(StringBuffer2,BUFFERSIZE,stream2);
				if(flagMap == true)
				{
					fgets(VectorBuffer,MAXPATH,list);	
				}
			}
		}
		
		if( svFile2 < j)
			svFile2 = j; 

		fgets(StringBuffer1,BUFFERSIZE,stream1);
	}
	
	/**Print results*/
	printf("\n======== Matching Result ========");
	printf("\nNumber of vectors in file 1: %d",svFile1);
	printf("\nNumber of vectors in file 2: %d",svFile2);
	printf("\nVectors matched: %d\n",counter);
	if( flagSaveResult == true )
	{
		fprintf(output,"\n======== Matching Result ========");
		fprintf(output,"\nNumber of vectors in file 1: %d",svFile1);
		fprintf(output,"\nNumber of vectors in file 2: %d",svFile2);
		fprintf(output,"\nVectors matched: %d\n",counter);
	}

	fclose(stream1);
	fclose(stream2);
	
	if( flagSaveResult == true )
	{	
		fclose(output);
	}
	return 0;
}
