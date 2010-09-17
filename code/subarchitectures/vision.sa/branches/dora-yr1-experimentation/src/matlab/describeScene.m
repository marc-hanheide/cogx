function describeScene(A,R,P)

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

%AV and pos
for obj=1:2

   px=P(1,obj);
   py=P(2,obj);

   av=A(:,obj)==1;
   avs=avNames(av);
   for i=1:size(avs,1)
      avs1(i,:)={[' ' cell2mat(avs(i,:))]};
   end;
   if size(avs,1)>0
      avs=cell2mat(avs1');
   else
      avs=' ';
   end;

   disp(['Object ' AB(obj) ' (' num2str(px) ',' num2str(py) '):' avs]);

end

%unary relations
for obj=1:2

   ru=6+find(R(7:11,obj)==1);
   rus=relNames(ru);
   for i=1:size(rus,1)
      disp([AB(obj) ' is ' cell2mat(rus(i)) '.']);
   end;

end


%binary relations
ru=find(R(1:6,1)==1);
rus=relNames(ru);
for i=1:size(rus,1)
   disp([AB(1) ' is ' cell2mat(rus(i)) ' ' AB(2) '.']);
end;

