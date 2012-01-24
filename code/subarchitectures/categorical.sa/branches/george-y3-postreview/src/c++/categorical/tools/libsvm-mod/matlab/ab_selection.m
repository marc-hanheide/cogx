function [a , b, m] = ab_selection(k, s, b, a, gamma, train, test);

cmd=sprintf('cp %s train', train);
eval unix(cmd);

cmd=sprintf('cp %s test', test);
eval unix(cmd);

%DETERMINE a and b

sa=length(a);
sb=length(b);
%The missclassification rate is computed for a=0.5 and b=0.5, 1, 1.5 and 2
for i=1:sa
    for j=1:sb
        a(i)
        b(j)
        k
        cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train ', k, s, b(j), a(i), gamma)
        eval unix(cmd);
        cmd=sprintf('svm-predict test train.model output > mfh')
        eval unix(cmd);
        cmd5=sprintf('java getError mfh percent');
        eval unix(cmd5);
        cmd6=('rm mfh');
        eval unix(cmd6);
    end
end

% Reading the data from the fil procent
A = importdata('percent');

% Storing it in a matrix where each row represents a value of a

sa=length(a);
sb=length(b);

index=1;
A(index:(index+sb-1))'
B=[A(index:(index+sb-1))'];
index=sb+1;

for i=2:sa
    B=[B ;A(index:index+sb-1)']
    index=i*sb+1;
end
B
m=[];
% Getting the minimum missclassification rate and storing it in array m
%if (sa>1);
    m(1)=max(max(B));
    %else
    %m(1)=max(B);
    %end

d=[0.5 1 1.5 2];

% The j is a value that indicates if two classification rates are the same
j=1
a_old=a;
a=[];
b_value=[];

% Determining which a and b that corresponds to the minimum missclassification rate
for i=1:sb
    for(l=1:sa)
        c=B(l,i)
        if c==m(1)
            a(j)=a_old(l)
            b_value(j)=d(i)
            j=j+1;
        end
    end
end


cmd=sprintf('rm percent');
eval unix(cmd);

% This part will be implemented if necessary, for now it prints error message when several missclassification rates are the same
if j>2
    str1 = {'WARNING: j is greater than 1, not implemented for this yet'};
    %   if a(1)~=a(j)
    %   a1=a(i)-0.2 
    %   a2=a(i)+0.2  
end

% A new value of a is computed by moving 0.1 up and down
a1=a(1)-0.1 
a2=a(1)+0.1
l=0

% Since a can't be larger than 1 in that case a is moved 0.1 and 0.2 down
if a(1)==1
    a1=a(1)-0.1;
    a2=a(1)-0.2;
    l=1;
end

% b is treated in the same way as a
b1=b_value(1)-0.1
b2=b_value(1)+0.1

if b_value(1)==2
    b1=b_value(1)-0.1;
    b2=b_value(1)-0.2;
    l=2
end

% The missclassification rate is determined in the same way as above with the new values of a and b
for a=[a1 a2] 
    for b=[b1 b2]
        cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train',  k, s, b, a, gamma)
        eval unix(cmd);
        cmd=sprintf('svm-predict test train.model output > mfh')
        eval unix(cmd);
        cmd5=sprintf('java getError mfh percent');
        eval unix(cmd5);
        cmd6=('rm mfh');
        eval unix(cmd6);
    end
end

A = importdata('percent');

cmd=sprintf('rm percent');
eval unix(cmd)

B=[A(1:2)';A(3:4)'];

% The minimum missclassification rate is stored 
m(2)=max(max(B))

j=1;


d=[b1 b2];

% If the new missclassification rate is less than the old one a new a and b is computed otherwise a and b remains unchanged.
% This is repeated until in worst case the whole range between the first division of a and b is covered
if m(2)>m(1)
    for i=1:2
        for(c=B(1,i))
            if c==m(2)
                a(j)=a1;
                b_value(j)=d(i)
                j=j+1;
                a3=a(1)-0.1; 
                if i==1
                    b3=b_value(1)-0.1;
                end
                if i==2
                    b3=b_value(1)+0.1;
                end
            end
        end
        for (c=B(2,i))
            if c==m(2)
                a(j)=a2
                b_value(j)=d(i)
                j=j+1;
                a3=a(1)+0.1
                if i==1
                    b3=b_value(1)-0.1;
                end
                if i==2
                    b3=b_value(1)+0.1;
                end
            end
        end
    end  
    
    m(1)=m(2);
    
    if j>2
        str1 = {'j is greater than 1, not implemented for this yet, 2'}
    end
    
    
    for a=[a3 a(1)]
        for b=[b3 b_value(1)]
            cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train', k, s, b, a, gamma)
            eval unix(cmd);
            cmd=sprintf('svm-predict test train.model output > mfh')
            eval unix(cmd);
            cmd5=sprintf('java getError mfh percent');
            eval unix(cmd5);
            cmd6=('rm mfh');
            eval unix(cmd6);
        end
    end
    
    A = importdata('percent');
    
    cmd=sprintf('rm percent');
    eval unix(cmd);
    
    B=[A(1:2)';A(3:4)'];
    
    m(2)=max(max(B));
    
    j=1;
    
    d=[ b3 b_value(1)];
    if m(2)>m(1)
        for i=1:2
            for(c=B(1,i))
                if c==m(2)
                    a(j)=a3
                    b_value(j)=d(i)
                    j=j+1;
                    a4=a(1)-0.1; 
                    if i==1
                        b4=b_value(1)-0.1;
                    end
                    if i==2
                        b4=b_value(1)+0.1;
                    end
                end
            end
            for (c=B(2,i))
                if c==m(2)          
                    b_value(j)=d(i)
                    j=j+1;
                    a4=a(1)+0.1
                    if i==1
                        b4=b_value(1)-0.1;
                    end
                    if i==2
                        b4=b_value(1)+0.1;
                    end
                end
            end
        end
        m(1)=m(2);
        
        if j>2
            str1 = {'j is greater than 1, not implemented for this yet, 3'};
        end
        
        
        for a=[a4 a(1)]      
            for k=[b4 b_value(1)]
                cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train', k, s, b, a, gamma)
                eval unix(cmd);
                cmd=sprintf('svm-predict test train.model output > mfh')
                eval unix(cmd);
                cmd5=sprintf('java getError mfh percent');
                eval unix(cmd5);
                cmd6=('rm mfh');
                eval unix(cmd6);
            end
        end
        
        A = importdata('percent');
        
        cmd=sprintf('rm percent');
        eval unix(cmd)
        
        B=[A(1:2)';A(3:4)'];
        
        m(2)=max(max(B))
        
        j=1
        
        
        d=[b4 b_value(j)];
        if m(2)>m(1)
            for i=1:2
                for(c=B(1,i))
                    if c==m(2)
                        a(j)=a4
                        b_value(j)=d(i)
                        j=j+1;
                        a5=a(1)-0.1; 
                        if i==1
                            b5=b_value(1)-0.1;
                        end
                        if i==2
                            b5=b_value(1)+0.1;
                        end
                    end
                end
                for (c=B(2,i))
                    if c==m(2)
                        b_value(j)=d(i)
                        j=j+1;
                        a5=a(1)+0.1
                        if i==1
                            b5=b_value(1)-0.1;
                        end
                        if i==2
                            b5=b_value(1)+0.1;
                        end
                    end
                end
            end
            m(1)=m(2);
            
            if j>2
                str1 = {'j is greater than 1, not implemented for this yet, 3'};
            end
            
            
            for a=[a5 a(1)]
                for k=[b5 b_value(1)]
                    cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train',  k, s, b, a, gamma)
                    eval unix(cmd);
                    cmd=sprintf('svm-predict test train.model output > mfh')
                    eval unix(cmd);
                    cmd5=sprintf('java getError mfh percent');
                    eval unix(cmd5);
                    cmd6=('rm mfh');
                    eval unix(cmd6);
                end
            end
            A = importdata('percent');
            
            cmd=sprintf('rm percent');
            eval unix(cmd)
            
            B=[A(1:2)';A(3:4)'];
            
            m(2)=max(max(B))
            
            j=1;
            %a_road=[];
            %b_road=[];
            %a_road(3)=a(1);
            %b_road(3)=b_value(1)
            
            d=[b5 b_value(j)];
            if m(2)>m(1)
                for i=1:2
                    for(c=B(1,i))
                        if c==m(2)
                            a(j)=a5
                            b_value(j)=d(i)
                            j=j+1;
                        end
                    end
                    for (c=B(2,i))
                        if c==m(2)
                            b_value(j)=d(i)
                            j=j+1;
                        end
                    end
                end
                m(1)=m(2);
            end
        end
    end
end

% m is the minimum classication rate, a is the best value of a and b is the best value of b
m=m(1)
a=a(j)
b=b_value(j)
