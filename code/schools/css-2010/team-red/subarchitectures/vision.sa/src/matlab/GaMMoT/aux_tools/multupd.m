function alpha=multupd(Q,D)
%
% Multiplicative update optimisation algorithm for Reduced Set Density Estimation (RSDE).
%    Minimising 0.5*alpha'*Q*alpha-alpha'*D
%    Updating rule: alpha=(alpha.*D')./(Q*alpha);
%    
%    Technical reference:
%         F. Sha, L. Saul and D. Lee. "Multiplicative updates for non-negative quadratic 
%         programming in support vector machines". Technical report MS-CIS-02-19, University
%         of Pennsylvania, 2002.
%     
%    Use format: alpha=multupd(Q,D)
%
%    Input:  Q [NxN]:        Kernal matrix 
%            D [1xN]:        Parzen density estimate   
%    Return: alpha  [Nx1]:   Weight vector obtained by RSDE
%
%    Copyright Mark Girolami & Chao He
%    Last revised on August 22th, 2002
%

alpha_tolerance=1e-6;  %Tolerance to threshold weight be zero
error_tolerance=1e-9;  %Iteration error tolerance to terminate the algorithm

%Initialisation 
alpha=D'./sum(D');

err=0.5*alpha'*Q*alpha-alpha'*D';
dE=1;
while abs(dE)>error_tolerance
   a = alpha./(Q*alpha);
   norm_const = (1/sum(a))*(1-sum(a.*D')); 
   alpha=a.*(D+norm_const)';
   
   I=find(alpha<=alpha_tolerance);
   alpha(I)=0.0;
   alpha=alpha./sum(alpha);
   
   err1=0.5*alpha'*Q*alpha-alpha'*D';
   dE=err1-err;
   err=err1;
   fprintf('Iteration error = %e\n',dE);
end


