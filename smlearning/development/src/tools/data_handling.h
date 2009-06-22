#include <cstdlib>
#include <ctime>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>

using namespace std;

typedef vector<double> FeatureVector;
typedef vector<FeatureVector> Sequence;
typedef vector<Sequence> DataSet;


// function that prints the passed argument
template <typename T>
void print_item (const T& elem);

template <typename T>
void print_featvector (const FeatureVector& v);

template <typename T>
void print_sequence (const Sequence& s);

template <typename T>
void print_database (const DataSet& d);

void generate_rand_sequences (DataSet& data, long numSeq, long seqSize);

bool write_database (string fileName, const DataSet& data);

bool read_database (string fileName, DataSet& data);
