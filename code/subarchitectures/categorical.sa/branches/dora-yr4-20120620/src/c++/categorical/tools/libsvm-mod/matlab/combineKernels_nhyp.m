function [k,class_rate] = combineKernels_nhyp(input_string)

rem = input_string;

v_fixed=0;

train_files=[];
test_files=[];
voteMatrix=[];
nr_train_files=0;
nr_test_files=0;
nr_voteMatrix=0;
hypMatrix=[];
output_string = ' ';
print=0;
n=0;

while (any(rem))
    [opt,rem] = strtok(rem);
    if strcmp(opt, 'k') == 1
        [k, rem] = strtok(rem);
        k = [' k ', k];
        output_string=[output_string, k];
    elseif strcmp(opt, 'b') == 1
        [b, rem] = strtok(rem);
        b = [' b ', b]; 
        output_string=[output_string, b];
    elseif strcmp(opt, 'a') == 1
        [a, rem] = strtok(rem);
        a = [' a ', a]; 
        output_string=[output_string, a];
    elseif strcmp(opt, 'g') == 1
        [g, rem] = strtok(rem);
        g = [' g ', g]; 
        output_string=[output_string, g];
    elseif strcmp(opt, 'v') == 1
        nr_voteMatrix=nr_voteMatrix+1;
        [voteMatrix_name, rem]= strtok(rem);
        voteMatrix=[voteMatrix, ' ', voteMatrix_name];
    elseif strcmp(opt ,'fv') == 1
        v_fixed=1;
    elseif strcmp(opt, 'fab') == 1
        output_string=[output_string, ' fab'];       
    elseif strcmp(opt, 'fg') == 1
        output_string=[output_string, ' fg'];
    elseif strcmp(opt,'train') == 1
        nr_train_files = nr_train_files+1;
        [opt, rem] = strtok(rem);
        train_files = [train_files,' ', opt];
    elseif strcmp(opt,'test') == 1
        nr_test_files = nr_test_files+1; 
        [opt, rem] = strtok(rem);
        test_files = [test_files,' ',opt]; 
     elseif strcmp(opt,'n') == 1
        [opt,rem]=strtok(rem)
        n=str2num(opt);
        [hypMatrix_name, rem]=strtok(rem);
        hypMatrix=[hypMatrix, ' ',hypMatrix_name];
        voteMatrix=hypMatrix;
     elseif strcmp(opt,'p') == 1;
        print=1;
        [Matrix_name,rem]=strtok(rem);
    end
end
a=[];
b=[];
gamma=[];
m=[];

if v_fixed == 0 
    if (nr_train_files ~= nr_test_files)|(nr_train_files~=nr_voteMatrix)
        {'ERROR: Number of training-, testfiles and voteMatrices must be the same!'}
        break
    end


    for i=1:nr_train_files
        [train, train_files]=strtok(train_files);
        [test, test_files]=strtok(test_files);
        [vote, voteMatrix]=strtok(voteMatrix);
        output=[output_string, ' v ', vote, ' train ', train, ' test ',test]
        [a(i),b(i),gamma(i),m(i)] = model_selection_plus(output)
    end
end


temp_labels=[];



for i=1:nr_voteMatrix
    [vote_temp, voteMatrix]=strtok(voteMatrix);
    temp=importdata(vote_temp);
    ltemp=length(temp(:,1));
    temp_labels(i,:)=temp(1,:);
    if i==1
       vote1=temp(2:ltemp,:);
    elseif i==2
       vote2=temp(2:ltemp,:);
    end
 end

 
if temp_labels(1,:)~=temp_labels(2,:)
    {'Error: Labels must be in the same order for all features'}
    break
end

labels=temp_labels(1,:);
ll=length(labels);
label=[];
target=importdata('target');
lt=length(target);
correct_percent=[];
i=0;
if nr_voteMatrix == 2
   k1=[0.01 0.1 1 10 100];
   lk=length(k1);
   for i=1:lk
      combined_votes=vote1+k1(i)*vote2;
      for j=1:ltemp-1
         m=max(combined_votes(j,:));
         for k=1:ll
            if m == combined_votes(j,k)
               label(j)=labels(k);
            end
         end
      end  
      correct=lt;
      for n=1:lt;
         if label(n)~=target(n);
            correct=correct-1;
         end
      end
      correct_percent(i)=correct/lt
   end
   m1=max(correct_percent);
   flag = 0;
   for i=1:lk;
       if correct_percent(i) == m1 & flag == 0
           k1=k1(i);
           flag=1;
       end
   end
    steg=k1/10;
    k1_1=k1+steg;
    k1_2=k1-steg;
    koeff=[k1_1 k1_2];
    for i=1:2
        koeff(i)
        combined_votes_temp=vote1+koeff(i)*vote2;
        for j=1:ltemp-1
            m=max(combined_votes_temp(j,:));
            for k=1:ll
                if m==combined_votes_temp(j,k)
                    label(j)=labels(k);
                end
            end
        end   
        correct=lt;
        correct_percent=[];
        for n=1:lt;
            if label(n)~=target(n);
                correct=correct-1;
            end
        end
        correct_percent(i)=correct/lt;
    end
    m2=max(correct_percent);
   
    if m2>=m1
       m1=m2;
       combined_votes=combined_votes_temp;
        for i=1:2;
            if correct_percent(i) == m1
                k1=koeff(i);
            end
        end
        if k1==k1_2
            steg=-steg;
        end
        k1_1=k1+steg;
        combined_votes_temp=vote1+k1_1*vote2;
        for j=1:ltemp-1
            m=max(combined_votes_temp(j,:));
            for k=1:ll
                if m==combined_votes_temp(j,k)
                    label(j)=labels(k);
                end
            end
        end
        correct=lt;
        for n=1:lt
            if label(n)~=target(n);
                correct=correct-1;
            end
        end
        correct_percent=correct/lt;   
        m2=max(correct_percent);
        while m2 > m1
           m1=m2;
           combined_votes=combined_votes_temp;
            k1=k1_1;
            k1_1=k1_1+steg;
            combined_votes_temp=vote1+k1_1*vote2;
            for j=1:ltemp-1
                m=max(combined_votes_temp(j,:));
                for k=1:ll
                    if m==combined_votes_temp(j,k)
                        label(j)=labels(k);
                    end
                end
            end
            correct=lt;
            for n=1:lt;
                if label(n)~=target(n);
                    correct=correct-1;
                end
            end
            correct_percent=correct/lt;  
            m2=max(correct_percent);
        end
    end
    k=k1;
        
elseif nr_train_files == 3
    for k1=[0.01 0.1 1 10 100]
        for k2=[0.01 0.1 1 10 100]
            
        end
    end
elseif nr_train_files == 4   
     
elseif nr_train_files == 5
elseif nr_train_files == 6
end

k;
class_rate=m1*100;

if print==1
   fid=fopen(Matrix_name,'wb');
   fwrite(fid,labels);
   fwrite(fid,combined_votes);
end
