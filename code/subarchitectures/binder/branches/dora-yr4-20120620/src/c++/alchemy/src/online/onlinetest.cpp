/*
 * All of the documentation and software included in the
 * Alchemy Software is copyrighted by Stanley Kok, Parag
 * Singla, Matthew Richardson, Pedro Domingos, Marc
 * Sumner, Hoifung Poon, and Daniel Lowd.
 * 
 * Copyright [2004-07] Stanley Kok, Parag Singla, Matthew
 * Richardson, Pedro Domingos, Marc Sumner, Hoifung
 * Poon, and Daniel Lowd. All rights reserved.
 * 
 * Contact: Pedro Domingos, University of Washington
 * (pedrod@cs.washington.edu).
 * 
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the
 * following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the
 * above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 * 
 * 3. All advertising materials mentioning features or use
 * of this software must display the following
 * acknowledgment: "This product includes software
 * developed by Stanley Kok, Parag Singla, Matthew
 * Richardson, Pedro Domingos, Marc Sumner, Hoifung
 * Poon, and Daniel Lowd in the Department of Computer Science and
 * Engineering at the University of Washington".
 * 
 * 4. Your publications acknowledge the use or
 * contribution made by the Software to your research
 * using the following citation(s): 
 * Stanley Kok, Parag Singla, Matthew Richardson and
 * Pedro Domingos (2005). "The Alchemy System for
 * Statistical Relational AI", Technical Report,
 * Department of Computer Science and Engineering,
 * University of Washington, Seattle, WA.
 * http://www.cs.washington.edu/ai/alchemy.
 * 
 * 5. Neither the name of the University of Washington nor
 * the names of its contributors may be used to endorse or
 * promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF WASHINGTON
 * AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY
 * OF WASHINGTON OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include "arguments.h"
#include "onlineengine.h"

int main(int argc, char* argv[])
{
    // Example inference string needed to construct on OnlineEngine
  string inferenceString =
    "-ms -i univ-out.mln -e univ-test.db -q advisedBy,student -lazy true";
  OnlineEngine* oe = new OnlineEngine(inferenceString);
  oe->init();

  vector<string> trueEvidence;
  vector<string> falseEvidence;
  vector<string> oldEvidence;

  vector<string> nonZeroAtoms;
  vector<float> probs;
  
    // Time step 1
  oe->infer(nonZeroAtoms, probs);
    // Print true atoms
  cout << "Time step 1 non-zero atoms: " << endl;
  vector<string>::const_iterator it = nonZeroAtoms.begin();
  vector<float>::const_iterator probIt = probs.begin();
  for (; it != nonZeroAtoms.end(); it++)
  {
    cout << (*it) << " " << (*probIt) << endl;
    probIt++;
  }

    // Change evidence / query
  trueEvidence.push_back("inPhase(Gail, Pre_Quals)");
//  trueEvidence.push_back("student(Gail)");
//  falseEvidence.push_back("professor(Gail)");
  oldEvidence.push_back("hasPosition(Gail, Faculty)");
  oe->addTrueEvidence(trueEvidence);
  oe->addFalseEvidence(falseEvidence);
  oe->removeEvidence(oldEvidence);

    // Time step 2
    // Reduce the number of inference steps
  oe->setMaxInferenceSteps(100);
  oe->infer(nonZeroAtoms, probs);

    // Print true atoms
  cout << "Time step 2 true atoms: " << endl;
  it = nonZeroAtoms.begin();
  probIt = probs.begin();
  for (; it != nonZeroAtoms.end(); it++)
  {
    cout << (*it) << " " << (*probIt) << endl;
    probIt++;
  }

    // No change in evidence / query
    // Time step 3
  oe->infer(nonZeroAtoms, probs);
    // Print true atoms
  cout << "Time step 3 true atoms: " << endl;
  it = nonZeroAtoms.begin();
  probIt = probs.begin();
  for (; it != nonZeroAtoms.end(); it++)
  {
    cout << (*it) << " " << (*probIt) << endl;
    probIt++;
  }

  delete oe;
}

