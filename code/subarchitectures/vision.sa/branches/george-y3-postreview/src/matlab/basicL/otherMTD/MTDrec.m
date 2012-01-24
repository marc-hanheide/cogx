function rQnt=MTDrec(f,mC,mFS)

global MTD;

switch MTD
   case 1
      rQnt=MVBFrec(f,mC,mFS);
   case 2
      rQnt=KDBFrec(f,mC,mFS);
   case 3
      rQnt=MKDErec(f,mC);
   case 4
      rQnt=ABrec(f,mC);
   case 5
      rQnt=MKDBFrec(f,mC);
   case 6
      rQnt=ODKDErec(f,mC);
end

  

