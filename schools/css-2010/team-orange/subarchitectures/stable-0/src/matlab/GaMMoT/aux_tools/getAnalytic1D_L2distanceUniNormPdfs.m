function I = getAnalytic1D_L2distanceUniNormPdfs( p1, p2 )

p1 = regularizeform( p1 ) ;
p2 = regularizeform( p2 ) ;

p = makeDifference( p1, p2 ) ;

numUni = length(p.uni.weights) ;
numNorm = length(p.norm.weights) ;

U = 0 ;
% get uniform submatrix
for i = 1 : numUni
    w1 = p.uni.weights(i) ;
    m1 = p.uni.mu(i) ;
    width1 = p.uni.widths(i) ;
    for j = i+1 : numUni
        w2 = p.uni.weights(j) ;
        m2 = p.uni.mu(j) ;
        width2 = p.uni.widths(j) ;
        U = U + 2*integralUniUni( w1, m1, width1, w2, m2, width2 ) ;        
    end
end

for i = 1 : numUni
    w1 = p.uni.weights(i) ;
    m1 = p.uni.mu(i) ;
    width1 = p.uni.widths(i) ;
    j = i ;
        w2 = p.uni.weights(j) ;
        m2 = p.uni.mu(j) ;
        width2 = p.uni.widths(j) ;
        U = U + integralUniUni( w1, m1, width1, w2, m2, width2 ) ;        
end

% get Gauss submatrix
G = 0 ;
for i = 1 : numNorm
    w1 = p.norm.weights(i) ;
    m1 = p.norm.mu(i) ;
    C1 = p.norm.covariances(i) ;
    for j = i+1 : numNorm
        w2 = p.norm.weights(j) ;
        m2 = p.norm.mu(j) ;
        C2 = p.norm.covariances(j) ;
        G = G +2*integralGausGaus( w1, m1, C1, w2, m2, C2 ) ;        
    end
end


for i = 1 : numNorm
    w1 = p.norm.weights(i) ;
    m1 = p.norm.mu(i) ;
    C1 = p.norm.covariances(i) ;
    j = i  ;
        w2 = p.norm.weights(j) ;
        m2 = p.norm.mu(j) ;
        C2 = p.norm.covariances(j) ;
        G = G + integralGausGaus( w1, m1, C1, w2, m2, C2 ) ;        
end

% get Gauss Uni submatrix
GU = 0 ;
for i = 1 : numUni
    w1 = p.uni.weights(i) ;
    m1 = p.uni.mu(i) ;
    width1 = p.uni.widths(i) ;
    for j = 1 : numNorm
        w2 = p.norm.weights(j) ;
        m2 = p.norm.mu(j) ;
        C2 = p.norm.covariances(j) ;
        GU = GU + integralGausUni( w2, m2, C2, w1, m1, width1 ) ;        
    end
end
I = G + U + 2*GU ;

% 
% 
% p1 = regularizeform( p1 ) ;
% p2 = regularizeform( p2 ) ;
% 
% p = makeDifference( p1, p2 ) ;
% 
% numUni = length(p.uni.weights) ;
% numNorm = length(p.norm.weights) ;
% 
% U = zeros(numUni) ;
% % get uniform submatrix
% for i = 1 : numUni
%     w1 = p.uni.weights(i) ;
%     m1 = p.uni.mu(i) ;
%     width1 = p.uni.widths(i) ;
%     for j = i : numUni
%         w2 = p.uni.weights(j) ;
%         m2 = p.uni.mu(j) ;
%         width2 = p.uni.widths(j) ;
%         U(i,j) = integralUniUni( w1, m1, width1, w2, m2, width2 ) ;        
%     end
% end
% U = U + U' - (eye(numUni)).*U;
% 
% % get Gauss submatrix
% G = zeros(numNorm) ;
% for i = 1 : numNorm
%     w1 = p.norm.weights(i) ;
%     m1 = p.norm.mu(i) ;
%     C1 = p.norm.covariances(i) ;
%     for j = i : numNorm
%         w2 = p.norm.weights(j) ;
%         m2 = p.norm.mu(j) ;
%         C2 = p.norm.covariances(j) ;
%         G(i,j) = integralGausGaus( w1, m1, C1, w2, m2, C2 ) ;        
%     end
% end
% G = G + G' - (eye(numNorm)).*G;
% 
% % get Gauss Uni submatrix
% GU = zeros(numUni, numNorm) ;
% for i = 1 : numUni
%     w1 = p.uni.weights(i) ;
%     m1 = p.uni.mu(i) ;
%     width1 = p.uni.widths(i) ;
%     for j = 1 : numNorm
%         w2 = p.norm.weights(j) ;
%         m2 = p.norm.mu(j) ;
%         C2 = p.norm.covariances(j) ;
%         GU(i,j) = integralGausUni( w2, m2, C2, w1, m1, width1 ) ;        
%     end
% end
% M = [ [U, GU]; [GU' G]] ;
% I = sum(M(:)) ;
% 

% ------------------------------------------------------------- %
function I = integralGausUni( w1, mu1, S1, w2, mu2, width2 )
a = mu2 - width2/2 ;
b = mu2 + width2/2 ;

I = w1*w2* 1/2/(b-a)*pi^(1/2)*S1^(1/2)*(erf(1/2*2^(1/2)*(b-mu1)/S1^(1/2))-erf(1/2*2^(1/2)*(a-mu1)/S1^(1/2)))/(pi*S1)^(1/2) ;

% ------------------------------------------------------------- %
function I = integralGausGaus( w1, mu1, S1, w2, mu2, S2 )

I = w1*w2*1/2*2^(1/2)*exp(-1/2*(mu1^2+mu2^2-2*mu1*mu2)/(S2+S1))/pi^(1/2)/(S2+S1)^(1/2) ;

% ------------------------------------------------------------- %
function I = integralUniUni( w1, m1, width1, w2, m2, width2 )

r1 = m1 + [ -1, 1 ]*width1/2 ;
r2 = m2 + [ -1, 1 ]*width2/2 ;
range = sort([r1, r2]) ; 
range = range(2:3) ;

C1 = (diff(r1))^(-2) ;
C2 = (diff(r2))^(-2) ;
I = w1*w2*C1*C2*diff(range) ;

% ------------------------------------------------------------- %
function p = makeDifference( p1, p2 )
p = p1 ;

p.uni.mu = [p1.uni.mu, p2.uni.mu] ;
p.uni.widths = [p1.uni.widths, p2.uni.widths] ;
p.uni.weights = [p1.uni.weights, -p2.uni.weights] ;

p.norm.mu = [p1.norm.mu, p2.norm.mu] ;
p.norm.covariances = [p1.norm.covariances; p2.norm.covariances] ;
p.norm.weights = [p1.norm.weights, -p2.norm.weights] ;


% ------------------------------------------------------- %
function pdf = regularizeform( pdf )

norm.mu = [] ;  norm.weights = [] ; norm.covariances = [] ;
uni.mu = [] ;  uni.weights = [] ; uni.widths = [] ;
if (~isfield(pdf,'norm') & isfield(pdf,'weights'))
    norm.weights = pdf.weights ;
    norm.covariances = pdf.covariances ;
    norm.mu = pdf.mu ;    
end

if (isfield(pdf,'norm'))
   norm = pdf.norm ; 
end
    
if (~isfield(pdf,'uni'))
    pdf.uni = uni ;
else
    uni = pdf.uni ;
end

if ( size(norm.covariances,3) > 1 )
    norm.covariances = reshape(norm.covariances,length(norm.weights),rows(norm.mu)^2,1) ;
end    
 
pdf = [] ;
pdf.uni = uni ;
pdf.norm = norm ;
