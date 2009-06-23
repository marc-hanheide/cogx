#include "data_handling.h"

#define FEATUREVECTOR_SIZE 6

// function that prints the passed argument
template <typename T>
void print_item (const T& elem) {
    cout << elem << " ";
}

template <typename T>
void print_featvector (const FeatureVector& v) {

	for_each (v.begin(), v.end(), print_item<T>);

}

template <typename T>
void print_sequence (const Sequence& s) {
	typename Sequence::const_iterator i;
	cout << "{";
	for (i=s.begin(); i != s.end(); i++) {
		cout << "[";
		print_featvector<T> (*i);
		if (i+1 == s.end())
			cout << "]";
		else
			cout << "], ";
	}
	cout << "}" << endl;
}

template <typename T>
void print_dataset (const DataSet& d) {
	for_each (d.begin(), d.end(), print_sequence<T>);
}

void generate_rand_sequences (DataSet& data, long numSeq, long seqSize) {
	// initialize random seed:
	srand ( time(NULL) );
	
	// generate random number:
	double randNr;

	for (int s=0; s<numSeq; s++) {
		Sequence &currentSequence = *(new Sequence);
		for (int v=0; v<seqSize; v++) {

			FeatureVector &currentVector = *(new FeatureVector);
		
			for (int n=0; n< FEATUREVECTOR_SIZE; n++) {
				randNr = (rand() % 10 + 1) / 10.0;
				currentVector.push_back (randNr);			
			}

			
			currentSequence.push_back (currentVector);	
		}
		data.push_back (currentSequence);
	}

}

bool write_dataset (string fileName, const DataSet& data) {
	ofstream writeFile(fileName.c_str(), ios::out | ios::binary);
	if (!writeFile)
		return false;

	bool featvectorSizeSaved = false;
	long numSeqs = data.size();
	writeFile.write ((const char*)&numSeqs, sizeof(numSeqs));
// 	cout << numSeqs << endl;
	DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {
		long seqSize = (*s).size();
		writeFile.write ((const char*)&seqSize, sizeof (seqSize));
// 		cout << "\t" << seqSize << endl;
		Sequence::const_iterator v = (*s).begin();
		if (!featvectorSizeSaved) {
			long featvectorSize = (*v).size();
			writeFile.write ((const char*)&featvectorSize, sizeof (featvectorSize));
			featvectorSizeSaved = true;
// 			cout << "\t\t" << featvectorSize << endl;
		}

		for (v=(*s).begin(); v!= (*s).end(); v++) {
			FeatureVector::const_iterator n;
			for (n=(*v).begin(); n!= (*v).end(); n++) {
				writeFile.write ((const char* )&(*n), sizeof (*n));
			}
		}
	}
	
	writeFile.close();
	return true;
}

bool read_dataset (string fileName, DataSet& data) {
	ifstream readFile(fileName.c_str(), ios::in | ios::binary);
	if (!readFile)
		return false;

	bool featvectorSizeGot = false;
	long featvectorSize;
	long numSeq;
	readFile.read ((char* )&numSeq, sizeof(numSeq));
// 	cout << numSeq << endl;
	for (int s=0; s<numSeq; s++) {
		Sequence &currentSequence = *(new Sequence);
		long seqSize;
		readFile.read((char *)&seqSize, sizeof(seqSize));
// 		cout << "\t" << seqSize << endl;
		for (int v=0; v<seqSize; v++) {
			FeatureVector &currentVector = *(new FeatureVector);
			if (!featvectorSizeGot) {
				readFile.read ((char *)&featvectorSize, sizeof(featvectorSize));
				featvectorSizeGot = true;
// 				cout << "\t\t" << featvectorSize << endl;
			}
			for (int n=0; n<featvectorSize; n++) {
				double value;
				readFile.read ((char* )&value, sizeof(value));
				currentVector.push_back (value);
			}
			currentSequence.push_back (currentVector);
		}
		data.push_back(currentSequence);
	}	
	
	readFile.close();
	return true;



	
}

int main(int argc, char * argv[]) {
	if (argc < 3) {
		cerr << argv[0] << " [nr. of sequences] [sequences size]" << endl;
		exit (0);
	}
	DataSet data;

	//Generate artificial random sequences
	int numSeq = atoi(argv[1]);
	int seqSize = atoi(argv[2]);
	generate_rand_sequences (data, numSeq, seqSize);
	cout << "printing generated data: " << endl;
	print_dataset<double> (data);
	write_dataset ("training.dat", data);
	cout << "printing stored data: " << endl;
	DataSet savedData;
	read_dataset ("training.dat", savedData);
	print_dataset<double> (savedData);

	
}
