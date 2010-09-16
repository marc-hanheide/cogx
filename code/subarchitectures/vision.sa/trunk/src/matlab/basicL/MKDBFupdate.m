function [mC]=MKDBFupdate(F,C,mC)
%[mC]=MKDBFupdate(F,C,mC)
%MKDBF incremental learning - one update step.
%F: input feature vectors
%C: concept labels for these samples
%mC: current models of concepts


%parameters
ING=2; %Initial Gaussians
CCT=0.3;  %.5; %CompressionClusterThreshold
global Params
if ~isempty(Params)
   ING=Params.ING;
end;

%MDF={1,2,3,1:2,2:3,[1 3],1:3,4,5,6,4:5,5:6,[4,6],4:6};
%MDF={1,[1 3],1:3,4:6};
%MDF={1:3,4:6};
global currMode Coma
if ~isempty(currMode)
   MDF=Params.MDF;
   CM=Coma.SCC;
end
 
if size(F,2)>1   %several feature vectors given
   for i=1:size(F,2) %proces one by one
      [mC]=MKDBFupdate(F(:,i),lf2sf(C(:,i)),mC);
   end;
elseif ~isempty(C) %at least one concept given
   
   if size(C,2)==1 %add 1 weights if not provided
      C=[C ones(size(C,1),1)];
   end;
   
   %numF=size(F,1);
   numC=length(mC);
   if isempty(mC(1).name)
      numC=0;
   end;
   
   %UPDATE MKDE MODELS
   mC(1).Nall=mC(1).Nall+mean(C(:,2));
   oldCs=[mC.name];
   for i=1:size(C,1) %for all currently observed concepts
      
      %check if the concept already exists
      [isOld,idx]=ismember(C(i,1),oldCs);
      if  ~isOld %C does not exist yet => add new C
         numC=numC+1;
         idx=numC;
         mC(idx).name=C(i,1);
         mC(idx).conf=0;
      end;
      mC(idx).conf=mC(idx).conf+C(i,2);
      
      %update MKDE model
      if mC(idx).conf<ING %collect initial samples
         mC(idx).x_init=[mC(idx).x_init F];
      elseif mC(idx).conf==ING %initialize KDE
         mC(idx).kde= executeOperatorIKDE( [], 'input_data', mC(idx).x_init, 'add_input', 'compressionClusterThresh', CCT   );      
      else %update KDE
         mC(idx).kde= executeOperatorIKDE( mC(idx).kde, 'input_data', F, 'add_input' );
         if mC(idx).conf> 10
            %mC(i).kde= executeOperatorIKDE( mC(idx).kde, 'compress_pdf' );
         end
      end
   end;
   
   
%FEATURE SELECTION
[Fbs, M_distances]=selectFeatures(mC,CM,MDF);
   
   for i=1:numC
      oldFb=mC(i).Fb;
      newFb=MDF{Fbs(i)};
      mC(i).Fb=newFb;
      if ~isequal(newFb,oldFb) %new best feature
         disp(['SW-UPD: i=' num2str(i) ' C=' num2str(mC(i).name) ' Fb:' num2str(oldFb) ' -> ' num2str(newFb) ' (conf=' num2str(mC(i).conf) ')']);
      end
   end
   
  
   
end;


