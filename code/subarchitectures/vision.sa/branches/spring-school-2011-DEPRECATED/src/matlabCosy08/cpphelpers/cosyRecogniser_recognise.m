%// **orig**: this->idl2mat("f", roi->m_features);
%// **orig**: this->eval("global mAV mDA mFS");
%// **orig**: this->eval("global currMode");
%// **orig**: this->eval("readConstants;");
%// **orig**: this->eval("answ = KDBFrec(f, mAV, mFS);");
%// **orig**: this->eval("ansQl = qnt2ql(answ, currMode.THRs);");
%// **orig**: this->eval("ansYes = lf2sfa(ansQl, ANSyes)");
%// **orig**: this->eval("ansPy = lf2sfa(ansQl, ANSpy)");
function [ansYes, ansPy, answ] = cosyRecogniser_recognise(f)
   readConstants;
   global mAV mDA mFS
   global currMode
   global ANSyes ANSpy
   global avAcronyms

   answ = KDBFrec(f, mAV, mFS);
   if currMode.qnt2qlD==0
      ansQl = qnt2ql(answ, currMode.THRs);
   else
      ansQl = qnt2qlD(answ, currMode.THRs, currMode.CTT);
   end

   ansYes = lf2sfa(ansQl, ANSyes);
   ansPy = lf2sfa(ansQl, ANSpy);
   disp(['Recognised: ',idx2name(ansYes,avAcronyms)]);
   showRec(ansYes,ansPy,answ,f);
end
