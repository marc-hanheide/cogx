function[a, b, gamma, m]= model_selection_plus(input_string)

rem = input_string;
ab_fixed=0;
g_fixed=0;
m=0;
k = 6;
v=[];
gamma=0.1;
g_flag=0;
s=5;
a=[0.5 1];
b=[0.5:0.5:2];


while (any(rem))
    [opt, rem] = strtok(rem);
    if strcmp(opt, 'k') == 1
        [opt, rem] = strtok(rem);
        k=str2num(opt);
    elseif strcmp(opt, 'a') == 1
        [opt, rem] = strtok(rem); 
        a=str2num(opt);
    elseif strcmp(opt, 'b') == 1
        [opt, rem] = strtok(rem);
        b=str2num(opt);
    elseif strcmp(opt, 'g') == 1
        [opt, rem] = strtok(rem); 
        gamma=str2num(opt);
        g_flag=1;
    elseif strcmp(opt,'train') == 1
        [train, rem] = strtok(rem);
    elseif strcmp(opt,'test') == 1
        [test, rem] = strtok(rem);
    elseif strcmp(opt, 'v') == 1
        [voteMatrix, rem] = strtok(rem);
        v='-v ';
        v = [v  voteMatrix];
    elseif strcmp(opt, 's') == 1
        [opt, rem] = strtok(rem);
        s=str2num(opt);
    elseif strcmp(opt, 'fab') == 1;
        ab_fixed=1;
    elseif strcmp(opt, 'fg') == 1;
        g_fixed=1;
    end
end

if k == 5
    ab_fixed=1;
    a=0;
    b=0;
end

if ab_fixed == 0;
    [a b m]=ab_selection(k, s, b, a, gamma, train, test);
end
if g_fixed == 0
    if g_flag == 0
        gamma=[0.001 0.01 0.1 1 10 ];
    end
    [gamma m]=gamma_selection(k, s, b, a, gamma, train, test);
end

cmd=sprintf('svm-train -t %i -c 100 -s %i -d %f -r %f -g %f train', k, s, b, a, gamma)
eval unix(cmd);

if isempty(v)==0;
    cmd=sprintf('svm-predict %s test train.model output', v )
    eval unix(cmd);
else 
    cmd=sprintf('svm-predict test train.model output')
    eval unix(cmd);
end
