function rCqnt=KDBFrec(F,mC,mFS)
%rCqnt=KDBFrec(F,mC,mFS)
%Recognize attribute values (quantitative)
%F: input feature vectors
%mC: models of concepts
%mFS: feature statistics
%rCqnt: recognized quantitative uncertanties for each concept in long unordered format
%     Note: only already learned Cs are considered and their numbers (sequence) do not
%     neccesarilly correspond to their names (numbers) in general


MINCONF=2;
MINCONF=6;

global Params
if ~isempty(Params)
   MINCONF=Params.MINCONF;
end;


numC=size(mC,2);
if isempty(mC(1).name)
   numC=0; 
end;
N=size(F,2);

rCqnt=zeros(numC,1+N);
for j=1:numC
   rCqnt(j,1)=mC(j).name;
   for i=1:N
      if mC(j).conf>=MINCONF
%         rC = 1-getBel( mC(j), F(mC(j).Fb,i) );
         rC = getBel( mC(j), F(mC(j).Fb,i) );
         rCqnt(j,i+1)=rC;
      else  %if conf==1 only one sample has been observed => no model yet
         rCqnt(j,i+1)=Inf;
      end
   end;
end;

%rCqnt
