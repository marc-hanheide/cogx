function [gamma, m]=gamma_selection(k, s, b, a, g, train, test)

cmd=sprintf('cp %s train', train);
eval unix(cmd);

cmd=sprintf('cp %s test', test);
eval unix(cmd);
lg=length(g);

for i=1:lg
    cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train', k, s, b, a, g(i))
    eval unix(cmd);
    cmd=sprintf('svm-predict test train.model output > mfh')
    eval unix(cmd);
    cmd5=sprintf('java getError mfh percent');
    eval unix(cmd5);
    cmd6=('rm mfh');
    eval unix(cmd6);
end


% Reading the data from the file procent
A = importdata('percent');
cmd=sprintf('rm percent');
eval unix(cmd);

% Getting the minimum missclassification rate and storing it in array m
m(1)=max(max(A));

for i=1:lg
    if A(i)==m(1);
        gamma=g(i);
    end
end


steg=gamma/2;
gamma1=gamma+steg;
gamma2=gamma-steg;
g=[gamma1 gamma2]
for i=1:2
        cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train', k, s, b, a, g(i))
        eval unix(cmd);
        cmd=sprintf('svm-predict test train.model output > mfh')
        eval unix(cmd);
        cmd5=sprintf('java getError mfh percent');
        eval unix(cmd5);
        cmd6=('rm mfh');
        eval unix(cmd6);
end



% Reading the data from the file procent
A = importdata('percent');
cmd=sprintf('rm percent');
eval unix(cmd);

% Getting the minimum missclassification rate and storing it in array m
m(2)=max(max(A));

if m(2)>=m(1);
    m(1)=m(2);
    for i=1:2
        if A(i)==m(1)
            gamma=g(i);
        end
    end
    if gamma==gamma2
        steg=-steg;
    end
    gamma1=gamma+steg;
    if gamma1 > 0
        cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train', k, s, b, a, gamma1)
        eval unix(cmd);
        cmd=sprintf('svm-predict test train.model output > mfh')
        eval unix(cmd);
        cmd5=sprintf('java getError mfh percent');
        eval unix(cmd5);
        cmd6=('rm mfh');
        eval unix(cmd6);
        m(2)=importdata('percent');
        cmd=sprintf('rm percent');
        eval unix(cmd);
        gamma2=gamma1;
        while m(2)>=m(1) & gamma2 > 0
            m(1)=m(2);
            gamma=gamma2;
            gamma2=gamma+steg;
            if gamma2 > 0
            cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train', k, s, b, a, gamma2)
            eval unix(cmd);
            cmd=sprintf('svm-predict test train.model output > mfh')
            eval unix(cmd);
            cmd5=sprintf('java getError mfh percent');
            eval unix(cmd5);
            cmd6=('rm mfh');
            eval unix(cmd6);
            m(2)=importdata('percent');
            cmd=sprintf('rm percent');
            eval unix(cmd);
            end
        end
    end
end

   steg=abs(steg/5);
   gamma1=gamma+steg;
   gamma2=gamma-steg;
   g=[gamma1 gamma2];
   for i=1:2
           cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train', k, s, b, a, g(i))
           eval unix(cmd);
           cmd=sprintf('svm-predict test train.model output > mfh')
           eval unix(cmd);
           cmd5=sprintf('java getError mfh percent');
           eval unix(cmd5);
           cmd6=('rm mfh');
           eval unix(cmd6);
   end
   
   
   
   % Reading the data from the file procent
   A = importdata('percent');
   cmd=sprintf('rm percent');
   eval unix(cmd);
   % Getting the minimum missclassification rate and storing it in array m
   m(2)=max(max(A));
   
   if m(2)>=m(1);
      m(1)=m(2);
      for i=1:2
         if A(i)==m(1)
            gamma=g(i);
         end
      end
      if gamma==gamma2
         steg=-steg;
      end
      gamma1=gamma+steg;
      if gamma1 > 0
         cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train', k, s, b, a, gamma1)
         eval unix(cmd);
         cmd=sprintf('svm-predict test train.model output > mfh')
         eval unix(cmd);
         cmd5=sprintf('java getError mfh percent');
         eval unix(cmd5);
         cmd6=('rm mfh');
         eval unix(cmd6);
         m(2)=importdata('percent');
         cmd=sprintf('rm percent');
         eval unix(cmd);
         gamma2=gamma1;
         while m(2)>=m(1) & gamma2 > 0
            m(1)=m(2)
            gamma=gamma2;
            gamma2=gamma+steg;
            if gamma2 > 0
               cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train', k, s, b, a, gamma2)
               eval unix(cmd);
               cmd=sprintf('svm-predict test train.model output > mfh')
               eval unix(cmd);
               cmd5=sprintf('java getError mfh percent');
               eval unix(cmd5);
               cmd6=('rm mfh');
               eval unix(cmd6);
               m(2)=importdata('percent');
               cmd=sprintf('rm percent');
               eval unix(cmd);
           end
       end
   end 
end


m=m(1);

