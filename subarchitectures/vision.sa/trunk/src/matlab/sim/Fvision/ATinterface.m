function ATinterface

global Settings

if Settings.attOn
   disp('ATinterface: Attention!');
   req=-1000; %Att. trigger
   VMinterface(req);
end;   