function ATinterface

global attOn;

if attOn
   disp('ATinterface: Attention!');
   req=-1000; %Att. trigger
   VMinterface(req);
end;   