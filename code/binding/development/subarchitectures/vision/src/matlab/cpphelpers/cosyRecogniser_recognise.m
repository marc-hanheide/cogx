%// **orig**: this->idl2mat("f", roi->m_features);
%// **orig**: this->eval("global mAV mDA mFS");
%// **orig**: this->eval("global currMode");
%// **orig**: this->eval("readConstants;");
%// **orig**: this->eval("answ = KDBFrec(f, mAV, mFS);");
%// **orig**: this->eval("ansQl = qnt2ql(answ, currMode.THRs);");
%// **orig**: this->eval("ansYes = lf2sfa(ansQl, ANSyes)");
%// **orig**: this->eval("ansPy = lf2sfa(ansQl, ANSpy)");
function [ansYes, ansPy] = cosyRecogniser_recognise(f)
   readConstants;
   global mAV mDA mFS
   global currMode
   global ANSyes ANSpy
   global avAcronyms

   answ = KDBFrec(f, mAV, mFS);
   ansQl = qnt2ql(answ, currMode.THRs);
   ansYes = lf2sfa(ansQl, ANSyes);
   ansPy = lf2sfa(ansQl, ANSpy);
   disp(['Recognised: ',idx2name(ansYes,avAcronyms)]);
end
