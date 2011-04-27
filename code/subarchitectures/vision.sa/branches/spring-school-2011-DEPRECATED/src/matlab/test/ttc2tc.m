function TSs=ttc2tc(Ns,TTSs)

[N,numLM]=size(Ns);
TTSs1=TTSs-[zeros(1,numLM); TTSs(1:N-1,:)];
Ns1=Ns-[zeros(1,numLM); Ns(1:N-1,:)];
TSs=TTSs1./Ns1;


return;

TTSs1=TTSs-[0 TTSs(1:end-1)];
Ns1=Ns'-[0 Ns(1:end-1)'];
TSs=TTSs1./Ns1;



