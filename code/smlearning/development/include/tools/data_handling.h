#include <cstdlib>
#include <ctime>
#include <vector>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>

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
void print_dataset (const DataSet& d);

void generate_rand_sequences (DataSet& data, long numSeq, long seqSize);

bool write_dataset (string fileName, const DataSet& data);

bool read_dataset (string fileName, DataSet& data);

bool write_cdl_file_padding (string fileName, const DataSet& data);

template <class Function>
bool write_n_fold_cross_valid_sets (string baseFileName, int n, Function f);
