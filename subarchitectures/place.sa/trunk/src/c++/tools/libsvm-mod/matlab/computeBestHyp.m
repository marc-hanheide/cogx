function [class_rate,v_vote1]=computeBestHyp(n,w,votematrix,votematrices,nr_views,nr_classes)

v_vote1=zeros(nr_views,n);

v_index=v_vote1;
v_vote2=v_vote1;
size(v_vote2)
size(v_vote1)
sorted_matrix=-sort(-votematrix,2);
v_vote1=sorted_matrix(:,1:n);

for i=1:nr_views
    for j=1:n
        for k=1:nr_classes
            if v_vote1(i,j)==votematrix(i,k);
            v_index(i,j)=k-1;
            end
        end
    end                
end

rem=votematrices;
while any(rem)
    [votematrix2,rem]=strtok(rem);
    [opt,w]=strtok(w);
    k=str2num(opt);
    for i=1:nr_views
        for j=1:n
            v_vote2(i,j)=votematrices(i,v_index(i,j)+1);
        end
    end
    k
    v_vote2=k*v_vote2;

    v_vote1=v_vote1+v_vote2;
end
label=[];
size(v_vote1);
m=max(v_vote1')';
for i=1:nr_views;
    for j=1:n
        if m(i)==v_vote1(i,j);
            label(i)=v_index(i,j);
        end
    end
end
label
pause(60)

target=importdata('target');
correct=nr_views;


for i=1:nr_views

    if target(i)~=label(i)
        correct=correct-1
    end
end
class_rate=correct/nr_views*100;
    

