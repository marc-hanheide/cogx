/** @file data_structs.cpp
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2010      Sergio Roa
 
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

 
 */

#include <tools/data_structs.h>


namespace smlearning {


///
///Write DataSet vector to a file
///
bool LearningData::write_dataset (string fileName, const LearningData::DataSet& data) {
	fileName += ".seq2";
	ofstream writeFile(fileName.c_str(), ios::out | ios::binary);
	if (!writeFile)
		return false;

	long numSeqs = data.size();
	writeFile.write ((const char*)&numSeqs, sizeof(numSeqs));
	cout << numSeqs << endl;
	LearningData::DataSet::const_iterator d_iter;
	for (d_iter=data.begin(); d_iter!=data.end(); d_iter++) {
		LearningData::MotorCommand mC = d_iter->first;
		LearningData::Chunk::Seq seq = d_iter->second;
		writeFile.write((const char*)&mC, sizeof(mC));
		long seqSize = seq.size();
		writeFile.write ((const char*)&seqSize, sizeof (seqSize));
  		cout << "\t" << seqSize << endl;
		LearningData::Chunk::Seq::const_iterator s_iter;
		for (s_iter=seq.begin(); s_iter != seq.end(); s_iter++) {
			writeFile.write ((const char*)&(*s_iter), sizeof(*s_iter));
		}

		
	}
	return true;
}


///
///Read DataSet vector from a file
///
bool LearningData::read_dataset (string fileName, LearningData::DataSet& data) {
	fileName  += ".seq2";
	ifstream readFile(fileName.c_str(), ios::in | ios::binary);
	if (!readFile)
		return false;

	long numSeq;
	readFile.read ((char*)&numSeq, sizeof(numSeq));
// 	cout << "Nr. of seq.: " << numSeq << endl;

	for (int s=0; s<numSeq; s++) {
		LearningData::Sequence currentSequence;
		LearningData::Chunk::Seq currentChunkSeq;
		LearningData::MotorCommand currentMotorCommand;

		readFile.read((char *)&currentMotorCommand, sizeof (currentMotorCommand));
		long seqSize;
		readFile.read((char *)&seqSize, sizeof(seqSize));
// 		cout << "  Seq. size: " << seqSize << endl;
//  		print_motorCommand (currentMotorCommand);
		for (int c=0; c<seqSize; c++) {
			LearningData::Chunk currentChunk;
			readFile.read((char *)&currentChunk, sizeof(currentChunk));
// 			print_Chunk (currentChunk);
			currentChunkSeq.push_back (currentChunk);
		}
		currentSequence = make_pair (currentMotorCommand, currentChunkSeq);
		data.push_back (currentSequence);
	}

	return true;	

}

///
///print a motor command struct
///
void LearningData::print_motorCommand (const LearningData::MotorCommand& mC) {
	cout << "\tmC=(";
	cout << " V=[" << mC.initEfPosition.v1 << " "
	     << mC.initEfPosition.v2 << " " <<  mC.initEfPosition.v3<< "],";
	cout << " s=" << mC.speed << ",";
	cout << " hA=" << mC.horizontalAngle << ")" << endl;
}

///
///print a chunk
///
void LearningData::print_Chunk (const LearningData::Chunk& c) {
	cout << "\tC=[";
	cout << " eP=" << c.effectorPose.p.v1 << " "
	     << c.effectorPose.p.v2 << " " << c.effectorPose.p.v2<< ",";
	cout << " oP=" << c.effectorPose.p.v1 << " "
	     << c.effectorPose.p.v2 << " " << c.effectorPose.p.v2<< ",";
	cout << " eO=" << c.efRoll << " " << c.efPitch << " " << c.efYaw << ",";
	cout << " oO=" << c.obRoll << " " << c.obPitch << " " << c.obYaw << " ]" << endl;
}

///
///print a DataSet vector
///
void LearningData::print_dataset (const LearningData::DataSet &d) {
	cout << "Dataset size: " << d.size() << endl;
	
	LearningData::DataSet::const_iterator d_iter;

	for (d_iter = d.begin(); d_iter != d.end(); d_iter++) {
		LearningData::MotorCommand mC = d_iter->first;
		cout << "{";

		LearningData::Chunk::Seq seq = d_iter->second;

		print_motorCommand (mC);
		cout << "\tSeq (size=" << seq.size() << ")" << "=(\n";

		LearningData::Chunk::Seq::const_iterator s_iter;
		for (s_iter=seq.begin(); s_iter != seq.end(); s_iter++) {
			print_Chunk (*s_iter);
		}
		cout << "\t)\n} " << endl;

		
	}
}


};  /* smlearning namespace */
