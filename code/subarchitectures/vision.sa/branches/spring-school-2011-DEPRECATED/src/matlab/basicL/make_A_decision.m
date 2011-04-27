function msg =  make_A_decision( i_c,  P )

% 'yes', 'prob yes', 'dont know', 'probably no', 'no'
%   1        2           3              4         5  
% P ... contains probabilities of alternative classes + class unknown
% structure of P: class1, class2, class3, ... , classUnknown 

answers = {'yes', 'prob yes', 'dont know', 'probably no', 'no'} ;
H_lo = 0.5 ; %45 ; %0.3 ; 
H_up = 0.75 ; %0.8 ;

% get most likely classification
[pmax, i_max] = max(P) ;

p0 = 1 - sum(P) ;
% calculate entropy of knowledge base
H = (1-(sum(P.^2)+p0^2))/(1 - 1/(length(P)+1)) ;  

if i_c == i_max
    % affirmative classification
    
    % inspect decision
    if H <= H_lo
        % certain decision
        answ = 1 ;        
    elseif H > H_lo && H < H_up
        % semi certain
        answ = 2 ;
    elseif H >= H_up
        % totally uncertain
        answ = 3 ;
    end    
else
    % alternative classification
    
    % inspect decision
    if H <= H_lo
        % certain decision
        answ = 5 ;        
    elseif H > H_lo && H < H_up
        % semi certain
        answ = 4 ;
    elseif H >= H_up
        % totally uncertain
        answ = 3 ;
    end     
end
msg = answers{answ} ;