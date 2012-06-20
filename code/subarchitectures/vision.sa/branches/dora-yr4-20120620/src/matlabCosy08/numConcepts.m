function [nlc,nec,LC,EC,confs]=numConcepts(mAV)

MINCONF=2;

EC=[mAV.name];
nec=length(EC);

LC=[];
for j=1:nec 
   if mAV(j).conf>=MINCONF
      LC=[LC mAV(j).name];
   end
end;   
nlc=length(LC);

confs=[mAV.conf];
