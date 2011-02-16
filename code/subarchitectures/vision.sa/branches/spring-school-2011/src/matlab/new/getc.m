function val=getc(varargin)
%function val=getc(mC,sc,c,param)

nargin=length(varargin);

if nargin==0
   disp('Available parameters: name pdf numComp conf Fb numSC numC');
   return;
end

mC=varargin{1};

if nargin>2
   sc=varargin{2};
end;
if nargin>3
   c=varargin{3};
   if c==0
      numC=getc(mC,sc,'numC');
      c=1:numC;
   end   
end;
param=varargin{nargin};



switch param
   case 'name'
      numc=length(c);
      val=zeros(1,numc);
      for i=1:numc
         val(i)=str2num(mC{sc}.class_labels_names{c(i)});
      end
   case 'pdf'
      val=mC{sc}.kde_cl{c}.pdf;
   case 'numComp'
      if exist('sc')
         if exist('c')
            numc=length(c);
            val=zeros(1,numc);
            for i=1:numc
               val(i)=length(mC{sc}.kde_cl{c(i)}.pdf.w);
            end
         else
            val=mean(getc(mC,sc,0,'numComp'));
         end
      else
         numSC=getc(mC,'numSC');
         valall=[];
         for sc=1:numSC
            valall=[valall getc(mC,sc,0,'numComp')];
         end
         val=mean(valall);
      end
   case 'conf'
      if exist('sc')
         if exist('c')
            numc=length(c);
            val=zeros(1,numc);
            for i=1:numc
               val(i)=mC{sc}.kde_cl{c(i)}.ikdeParams.N_eff;
            end
         else
            val=sum(getc(mC,sc,0,'conf'));
         end
      else
         numSC=getc(mC,'numSC');
         valall=[];
         for sc=1:numSC
            valall=[valall getc(mC,sc,0,'conf')];
         end
         val=sum(valall);
      end
      %val=mC{sc}.kde_cl{c}.ikdeParams.N_eff;
   case 'Fb'
      if sc>0
         val=mC{sc}.sub_selected_features;
         if isempty(val)
            val=1:getc(mC,'numF');
         end
      else
         numSC=getc(mC,'numSC');
         for sc=1:numSC
            val{sc}=getc(mC,sc,'Fb');
         end
      end
   case 'info'
      numSC=getc(mC,'numSC');
      fprintf('numSC: %d,   numComp: %d,    conf: %d\n',getc(mC,'numSC'),getc(mC,'numComp'),getc(mC,'conf'));
      for sc=1:numSC
%          fprintf('*** SC%d: ***\n',sc);
%          fprintf('numC: %d,   numComp: %d,    conf: %d\n',getc(mC,sc,'numC'),getc(mC,sc,'numComp'),getc(mC,sc,'conf'));
         fprintf('SC%d ( numC: %d,   numComp: %d,    conf: %d )\n',sc,getc(mC,sc,'numC'),getc(mC,sc,'numComp'),getc(mC,sc,'conf'));
         fprintf('concepts: '); fprintf('%d ',getc(mC,sc,0,'name')); fprintf('\n');
         fprintf('numComp : '); fprintf('%2.1f ',getc(mC,sc,0,'numComp')); fprintf('\n');
         fprintf('conf    : '); fprintf('%d ',getc(mC,sc,0,'conf')); fprintf('\n');
         fprintf('Fb      : '); fprintf('%d ',getc(mC,sc,0,'Fb')); fprintf('\n');
      end;
         
   case 'numSC'
      val=length(mC);
    case 'numC'
       if exist('sc')
          val=length(mC{sc}.class_labels);
       else
          val=0;
          for sc=1:getc(mC,'numSC')
             val=val+getc(mC,sc,'numC');
          end
       end
   case 'numF'
      val=length(mC{1}.cummulative_feat_costs);
   case 'gains'
       val=mC{sc}.class_gains;
   otherwise
      val='';
      disp('Unknown parameter');
end
