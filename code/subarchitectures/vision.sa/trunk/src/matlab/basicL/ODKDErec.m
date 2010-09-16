function pcx=ODKDErec(F,mC)

%MINCONF = 3; %minimum number of previously observed objects of the particular category

numSC=getc(mC,'numSC');

for sc=1:numSC
   rslt = executeOperatorIKDEClsfr( mC{sc}, 'input_data', F, 'classifyData', 'use_unknown_model',1   ) ;         
   pcx{sc}=[[getc(mC,sc,0,'name')';0] rslt.P];
end

return;
   
%    
%    
%    input_data={};
%    for c=1:numC
%       idxs=find(C{sc}(c,:)==1);
%       indat=[] ;
%       indat.data=F(:,idxs);
%       indat.class=c;
%       input_data=horzcat(input_data,indat);
%    end;
%    
%    mC{sc}=executeOperatorIKDEClsfr( mC{sc}, 'input_data', input_data, 'add_input' ) ;
%    th_ref=mC{sc}.compressionClusterThresh;   
%    mC{sc}=executeOperatorIKDEClsfr( mC{sc},'compress_pdf',  'compressionClusterThresh', costThreshold);
%    mC{sc}=executeOperatorIKDEClsfr( mC{sc}, 'compressionClusterThresh', th_ref);
%    mC{sc}=executeOperatorIKDEClsfr( mC{sc}, 'make_simple_feature_selection') ;
%    
% end
% 
% 
% 
% minThUnk = 0.1 ;
% global currMode
% if ~isempty(currMode)
%    MDF=currMode.MDF;
%    CM=currMode.CTT;
% end
% 
% if isempty(mC(1).name)
%    pcx=0;pxc=0;pc=0;
%    pxc=[];pcx=[];pc=[];
%    return;
% end
% 
% numC=size(mC,2);
% if isempty(mC(1).name)
%    numC=0; 
% end;
% N=size(F,2);
% 
% Nall=sum([mC.conf])/3;
% 
% names=[mC.name];
% confs=[mC.conf];
% nCG=max(CM(:,2));
% sumCG=zeros(nCG,1);
% for i=1:nCG
%    cs=find(CM(:,2)==i);
%    ics=find(ismember(names,cs));
%    sumCG(i)=sum(confs(ics));
% end;
% 
% pxc=zeros(numC,1+N);
% pc=zeros(numC,1+N);
% pcx=zeros(numC,1+N);
% 
% %pxc, pc
% pci=zeros(numC,1);
% for j=1:numC
%    cj=mC(j).name;
%    pxc(j,1)=cj;
%    pc(j,1)=cj;
%    pci(j)=mC(j).conf/sumCG(CM(find(CM(:,1)==cj),2));
%    for i=1:N
%    if mC(j).conf>=MINCONF
%          rC = executeOperatorIKDE( mC(j).kde, 'input_data', F(:,i), 'evalPdfOnData', 'selectSubDimensions', mC(j).Fb ) ;
%          pxc(j,i+1)=rC.evalpdf;
%       else  %if conf<MINCONF only a few samples have been observed => no model yet
%          pxc(j,i+1)=0;
%       end
%    end;
% end;
% pc(:,2:N+1)=repmat(pci,1,N);
% 
% clasInd = {} ;
% sumpxcpc=zeros(nCG,N+1);
% for i=1:nCG
%    cs=find(CM(:,2)==i);
%    ics=find(ismember(names,cs));
%    clasInd = horzcat(clasInd, ics) ;
%    sumpxcpc(i,:)=sum(pxc(ics,:).*pc(ics,:));
% end;
% 
% 
% %pcx with unknown model
% Ckprior = zeros(1,numC) ;
% for i=1:numC
%     cg=CM(names(i),2);
% %     pUn=11/(sumCG(cg)+10);
% %     delt = mC(1).Nall - sumCG(cg) ;
%    % pUn = max(exp(-0.5*(sumCG(cg)/(0.5*(10+delt))).^2),minThUnk) ;
%     Ncthis = length(find(CM(:,2)==cg)) ;
%     pUn = exp(-0.5*(sumCG(cg)/20)^2) ; %1/(sumCG(cg)+1); %(1/(Ncthis+1))*exp(-sumCG(cg)/10) ;
%     pKn=1-pUn;
%     Ckprior(i) = pKn ;
% %      pcx(i,2:end)=pcx(i,2:end)*pKn;
% end; 
% 
% %pcx withouth unknown model
% normcs = zeros(1,numC) ;
% for j=1:numC
%    ccg=CM(find(CM(:,1)==mC(j).name),2);
%    pcx(j,1)=mC(j).name;
%    for i=1:N
%       if mC(j).conf>= MINCONF          
%          ncon = (sumpxcpc(CM(find(CM(:,1)==mC(j).name),2),i+1))*Ckprior(j) + (1-Ckprior(j)) ;
%          normcs(j) = ncon ; 
%          pcx(j,i+1)=pxc(j,i+1)*pc(j,i+1)*Ckprior(j) / ncon;%+1/1*1/Nall);
%       else  %if conf<MINCONF only a few samples has been observed => no model yet
%          pcx(j,i+1)=0;
%       end
%    end;
% end;
%  
% 
% % %pcx with unknown model
% % for i=1:numC
% %     cg=CM(names(i),2);
% % %     pUn=11/(sumCG(cg)+10);
% % %     delt = mC(1).Nall - sumCG(cg) ;
% %    % pUn = max(exp(-0.5*(sumCG(cg)/(0.5*(10+delt))).^2),minThUnk) ;
% %     Ncthis = length(find(CM(:,2)==cg)) ;
% %     pUn = (1/(Ncthis+1))*exp(-sumCG(cg)/100) ;
% %     pKn=1-pUn;
% %     pcx(i,2:end)=pcx(i,2:end)*pKn;
% % end;    
%    
%     
% %     
% % pUn=11/(Nall+10);
% % pKn=1-pUn;
% % pcx(:,2:end)=pcx(:,2:end)*pKn;
% 
% 
% 
% 
% 
