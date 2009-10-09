function [optp,optX]=tabuQO(X,C,i_max,L)
  
% Invocation: [optp,optX]=tabuks(X,W,P,m,i_max,L)
% Parameters:   X: Initial X value
%               W: The weight vector
%               P: The profit vector
%               m: Maximum weight
%           i_max: Maximum number of iterations
%               L: Lifespan on tabu list 
% 
% This function implements the tabu search algorithm 
% for solving the knapsack problem.
%
% Copyright Lars Aurdal/Rikshospitalet
  
  % Start by initialising the number of
  % iterations to zero
  
  i=0;
  
  % Get length of input X vector  
  
  n=length(X);
  
  % Initialise the tabu list to all zeros
  
  tL=zeros(1,n);
  
  % Initially the optimal X value equals
  % the X value
  
  optX=X;
  
  % Initialise the optimal profit
  
  optp = X*C*X' ;%optX*P';
  
  % Loop as long as the number of iterations is not exceeded
  
  while(i<i_max)
    % Decrement all non-zero values on the tabu list
    
    tL=tL-(tL>0);
    
    % Here we generate the enighbourhood of the current
    % solution X.  First, make a matrix where each row
    % equals the current X, then xor this with the identity
    % matrix to compute a new matrix containing the neighbouring
    % vectors.
    
    N = ones(1,n)'*X;
    N = xor(N,eye(n));
    
    % Based on the solutions contained in N,
    % calculate a vector of corresponding 
    % current weight values.
    
    currP =zeros(n,1) ;
    for j = 1 : n
      currP(j) = N(j,:)*C*N(j,:)' ;        
    end

    % Keep the maximal current profit corresponding
    % to al LEGAL solution
    
    m_c = min(currP)+1 ;
    currP = (currP-m_c).*(tL==0)';
    [currp,index] = max(currP);
    currp = currp + m_c - 1 ;
    % Keep best X from neighbourhood.
    % Update tabu list to make a transition
    % back to the old X impossible for a period
    
    X=N(index,:);
    tL(index)=L;
    
    % Update if this new profit is better than the
    % existing optimal profit.

    if(currp>=optp)
      optp=currp;
      optX=X;
    end
    
    % Increment iteration counter
    
    i=i+1;
    
  end

  
  
