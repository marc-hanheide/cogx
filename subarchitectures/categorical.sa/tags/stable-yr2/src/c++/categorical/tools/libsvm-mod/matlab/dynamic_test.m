function r = dynamic_test(input_string)

rem=input_string;
train_files = [];
test_files = [];
nr_train_files = 0;
nr_test_files = 0;
a_set = 0;
b_set = 0;
g_set = 0;
k = 6;


while (any(rem))
    [opt, rem] = strtok(rem);
    if strcmp(opt, 'k') == 1
        [opt, rem] = strtok(rem);
        k=opt;
    elseif strcmp(opt, 'a') == 1
        [opt, rem] = strtok(rem);
        a=opt; 
        a_set=1;
    elseif strcmp(opt, 'b') == 1
        [opt, rem] = strtok(rem);
        b=opt;
        b_set=1;
    elseif strcmp(opt, 'g') == 1
        [opt, rem] = strtok(rem);
        g=opt;
        g_set=1;  
    elseif strcmp(opt,'train') == 1
        nr_train_files = nr_train_files+1;
        [opt, rem] = strtok(rem);
        train_files = [train_files,' ',opt];
    elseif strcmp(opt,'test') == 1
        nr_test_files = nr_test_files+1; 
        [opt, rem] = strtok(rem);
        test_files = [test_files,' ',opt]; 
    end
end
