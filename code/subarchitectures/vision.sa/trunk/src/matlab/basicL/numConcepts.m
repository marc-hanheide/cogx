function [nlc,nec,LC,EC,confs]=numConcepts(mC)

MINCONF=2;

EC=[mC.name];
nec=length(EC);

LC=[];
for j=1:nec 
   if mC(j).conf>=MINCONF
      LC=[LC mC(j).name];
   end
end;   
nlc=length(LC);

confs=[mC.conf];
