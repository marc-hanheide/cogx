function scDesc=sceneDesc(A,R,P)

global avNames;

avNames={'red';'green';'blue';'yellow';'small';'large';'square';'circular';'triangular';'rectangular'};
relNames={...
   'to the left of';
   'to the right of';
   'closer than';
   'further away than';
   'near to';
   'far from';
   'on the left';
   'in the middle';
   'on the right';
   'near';
   'far away'};

AB='AB';

ddisp;
%AV and pos
for obj=1:2

   px=P(1,obj);
   py=P(2,obj);

   avs=find(A(:,obj)==1);
   ddisp([AB(obj) ' is ' DSavNames(avs) '.']);

end

%unary relations
for obj=1:2

   ru=6+find(R(7:11,obj)==1);
   rus=relNames(ru);
   for i=1:size(rus,1)
      ddisp([AB(obj) ' is ' cell2mat(rus(i)) '.']);
   end;

end


%binary relations
ru=find(R(1:6,1)==1);
rus=relNames(ru);
for i=1:size(rus,1)
   ddisp([AB(1) ' is ' cell2mat(rus(i)) ' ' AB(2) '.']);
end;

%B to A
ru=find(R(1:6,2)==1);
rus=relNames(ru);
for i=1:size(rus,1)
   ddisp([AB(2) ' is ' cell2mat(rus(i)) ' ' AB(1) '.']);
end;


scDesc=ddisp;
