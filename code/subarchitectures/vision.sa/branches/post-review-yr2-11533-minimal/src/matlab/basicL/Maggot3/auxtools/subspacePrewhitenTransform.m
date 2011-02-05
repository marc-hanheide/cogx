%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function output = subspacePrewhitenTransform( varargin )
%
% transforms a mixture model forward or backward using intrinsic subspace.
%
% could speedup by directly transform the upper mean and Covariance

additional_data = [] ;
kde_scale = [] ;
regularize = [] ;
allLayers = 1 ;
svdRes = [] ;
minEigenEnergy = 0 ;
pdf = [] ;
transDirection = [] ;
globalCov = [] ;
% process arguments
args = varargin;
nargs = length(args);
for i = 1:2:nargs
    switch args{i}        
        case 'pdf', pdf = args{i+1} ;
        case 'globalCov', globalCov = args{i+1} ; 
        case 'transDirection', transDirection = args{i+1} ; 
        case 'minEigenEnergy', minEigenEnergy = args{i+1} ; 
        case 'svdRes', svdRes = args{i+1} ; 
        case 'allLayers', allLayers = args{i+1} ; 
        case 'regularize', regularize = args{i+1} ; 
        case 'kde_scale', kde_scale = args{i+1} ; 
        case 'additional_data', additional_data = args{i+1} ; 
    end
end

switch(transDirection)
    case 'forward'
        output = goForwardTrans( pdf, globalCov, minEigenEnergy,...
                                 allLayers, svdRes, regularize, kde_scale, additional_data ) ;        
    case 'backward'
        output = goBackwardTrans( pdf, svdRes, allLayers, kde_scale ) ;
    otherwise
        error('Unknown transform direction!') ;
end

% ----------------------------------------------------------------------- %
function output = goBackwardTrans( pdf, svdRes, allLayers, kde_scale )

% add the missing nullspace to pdf and forward transform it 
num_nullDir = size(svdRes.S,1) - length(svdRes.id_valid) ;
pdf.Mu = [ pdf.Mu; zeros(num_nullDir,length(pdf.w)) ] ;
 
F_trns = svdRes.V*sqrt(svdRes.S) ;
C_prot = zeros(size(svdRes.S)) ;

subexists = 0 ;
if isfield(pdf,'smod')
    subexists = 1 ;
end
% forward transform the pdf and remove nonvalid eigendirections
if ( subexists == 1) && ~isempty(pdf.smod.H)
     C_prot = C_prot*0 ;
     C_prot(svdRes.id_valid,svdRes.id_valid) = pdf.smod.H ;
     pdf.smod.H  = F_trns * C_prot * F_trns' ; 
end
for j = 1 : length(pdf.w)
    if subexists == 1
        try
        buff_m = zeros(num_nullDir,length(pdf.smod.q(j).w)) ;
        catch
           sdsg = 9 
        end
        pdf.smod.q(j).Mu = [ pdf.smod.q(j).Mu; buff_m ] ;
        
        pdf.smod.q(j).Mu = F_trns*pdf.smod.q(j).Mu + repmat(svdRes.new_mu,1,length(pdf.smod.q(j).w)) ;
        for i = 1 : length(pdf.smod.q(j).w)
            %            pdf.smod.q(j).Mu(:,i) = F_trns*pdf.smod.q(j).Mu(:,i) + svdRes.new_mu ;            
            C_prot = C_prot*0 ;
            C_prot(svdRes.id_valid,svdRes.id_valid) = pdf.smod.q(j).Cov{i} ;
            pdf.smod.q(j).Cov{i} = F_trns*C_prot*F_trns' ;
        end
        [mu_tmp, C_tmp] = momentMatchPdf( pdf.smod.q(j).Mu, pdf.smod.q(j).Cov, pdf.smod.q(j).w ) ;
        pdf.smod.ps.Cov{j} = C_tmp ;
        
        
        if ~isempty(pdf.Cov)
            pdf.Cov{j} = pdf.smod.ps.Cov{j} + pdf.smod.H ;
        end
    else
        C_prot = C_prot*0 ;
        C_prot(svdRes.id_valid,svdRes.id_valid) = pdf.Cov{j} ;
        pdf.Cov{j} = F_trns*C_prot*F_trns' ; %%   pdf.Cov{j}*F_trns' ;
        mu_tmp = F_trns*pdf.Mu(:,j) + svdRes.new_mu ;
    end
    pdf.Mu(:,j) = mu_tmp ;
end

% if variable bandwidths activated
if subexists==1 && isfield(pdf.smod,'useVbw') && pdf.smod.useVbw == 1
    pdf = recalculateLocalVariableKDE( pdf ) ;
end

if ~isempty(kde_scale)    
    kde_scale.Mu = [ kde_scale.Mu; zeros(num_nullDir,1) ] ;
    kde_scale.Mu = F_trns*kde_scale.Mu + svdRes.new_mu ;
    C_prot = C_prot*0 ;
    C_prot(svdRes.id_valid,svdRes.id_valid) = kde_scale.Cov ;    
    kde_scale.Cov = F_trns*C_prot*F_trns' ;          
end
 
output.pdf = pdf ;
output.kde_scale = kde_scale ;

% ----------------------------------------------------------------------- %
function output = goForwardTrans( pdf, globalCov, minEigenEnergy, allLayers, svdRes, regularize, kde_scale, additional_data )
minVals = 1e-7; %10 ;  % was 1e-32
% practicallyZero = 1e-50 ;
if isequal(regularize, 'subRegularize')
    regularize = 1 ;
else
    regularize = 0 ;
end

isCompletelySingular = 0 ;
id_null = [] ;
id_nullVals = [] ;
if isempty(svdRes)    
    if isempty(pdf.Cov) && isempty(globalCov)
        error('At least some covariance should be provided!') ;
    end
        
    [new_mu, C] = momentMatchPdf( pdf.Mu, pdf.Cov, pdf.w ) ;
    if isempty(globalCov) 
        globalCov = C ;
    else
        C = globalCov ;
    end
    d = size(C,1) ;
    
    % calculate eigen directions determine the subspace
    [U,S,V] = svd(C) ; 
    V = U ;  
    
    % get energy of eigen directions and identify valid directions

% % %     s = diag(S) ;
% % %     e = s / max([minEigenEnergy,sum(s)]) ; 
% % %     if max(e) < minEigenEnergy
% % %         S_inv = eye(d,d)*(2/minEigenEnergy) ;
% % %     else
% % %         id_valid = find(e > minEigenEnergy) ;
% % %         id_null = find(e <= minEigenEnergy) ;
  
    s = diag(S) ;    
    if max(s) < minVals
        S_inv = eye(d,d)*(2/minVals) ;
        id_valid = 1:d ;
        isCompletelySingular = 1 ;
    else
        id_valid = find(s > minVals) ;
        id_null = find(s <= minVals) ;
 
        
        id_nullVals = s(id_null) ;
        
        %     % ratio-based nullspace computation
        %     E = E / max([minVals,sum(E)]) ;
        %     id_valid = find(E > minEigenEnergy) ;
        
        % S_tmp = S ;
        % S = S*0 ; S(id_valid,id_valid) = S_tmp(id_valid,id_valid) ;
        % modify nonvalid directions to prevent singularities         
        S_inv = eye(d,d)*0 ;
        S_inv(id_valid,id_valid) = diag(diag(S(id_valid,id_valid).^(-1))) ;
        % recalculate inverse values
        for i = 1 : length(id_nullVals)
            try
                tmp_val = 1 / id_nullVals(i) ;
            catch
                tmp_val = 1 ; %1/practicallyZero ; % doesn't matter
            end
            S_inv(id_null(i),id_null(i)) = tmp_val ;
        end
    end
    F_trns = sqrt(abs(S_inv))* inv(V) ; % determinant of valid part is one
    %  F_trns = inv(V*sqrt(S)) ; % determinant is one
else
    id_valid = svdRes.nullspace.id_valid ;
    id_nullVals = svdRes.nullspace.id_nullVals ;
    id_null = svdRes.nullspace.id_null ;
    V = svdRes.V ;
    S = svdRes.S ;   
    globalCov = V*S*V' ;
    S_inv = eye(size(S))*0 ;
    S_inv(id_valid,id_valid) = diag(diag(S(id_valid,id_valid).^(-1))) ;
    % recalculate inverse values
    for i = 1 : length(id_nullVals)        
        try 
            tmp_val = 1 / id_nullVals(i) ; 
        catch
            tmp_val = 1 ; % 1/practicallyZero ;             
        end
        if isnan(tmp_val) || isinf(tmp_val) 
            tmp_val = 0 ;
        end
        S_inv(id_null(i),id_null(i)) = tmp_val ;
    end
    
    new_mu = svdRes.new_mu ;
    F_trns = sqrt(abs(S_inv))* inv(V) ; 
    invF_trns = V * sqrt(S) ;
    % if regularization is required, correct the bandwidth such that
    % the input covariances are valid
    if regularize == 1        
        % extract and analyze the bandwidth subspace
        H_internal = pdf.smod.H ;
        H_trn = F_trns*H_internal*F_trns' ;
        [Ut,St,Vt] = svd(H_trn) ;
  
        E = diag(St) ; 
        id_invalid = find(E <= minEigenEnergy) ;
        id_invalid = [id_invalid , find( isnan(E) )] ;
        id_invalid = [id_invalid , find( isinf(E) )] ;
        id_invalid = unique(id_invalid) ;
        
        if ~isempty(id_invalid)
            E(id_invalid) = 2.0*minEigenEnergy ;
            St = diag(E) ;
            H_trn_r = Vt*St*Vt' ;

            H_internal_r = invF_trns*H_trn_r*invF_trns' ;
            pdf = readjustKernels( pdf, H_internal_r ) ;
        end
    end
end
 
% forward transform the pdf and remove nonvalid eigendirections
if ~isempty(pdf.smod.H)
    pdf.smod.H = F_trns*pdf.smod.H*F_trns' ;
    pdf.smod.H = pdf.smod.H(id_valid, id_valid) ;
end
% initialize the mean
pdf.Mu = zeros(length(id_valid),length(pdf.w)) ;
for j = 1 : length(pdf.w)
       pdf.smod.q(j).Mu = F_trns*(pdf.smod.q(j).Mu - repmat(new_mu,1,length(pdf.smod.q(j).w))) ;
       for i = 1 : length(pdf.smod.q(j).w)
%            pdf.smod.q(j).Mu(:,i) = F_trns*(pdf.smod.q(j).Mu(:,i) - new_mu) ;
           pdf.smod.q(j).Cov{i} = F_trns*pdf.smod.q(j).Cov{i}*F_trns' ;
           pdf.smod.q(j).Cov{i} = pdf.smod.q(j).Cov{i}(id_valid,id_valid) ;
       end 
       pdf.smod.q(j).Mu = pdf.smod.q(j).Mu(id_valid,:) ;
       [mu_tmp, C_tmp] = momentMatchPdf( pdf.smod.q(j).Mu, pdf.smod.q(j).Cov, pdf.smod.q(j).w ) ;       
       pdf.smod.ps.Cov{j} = C_tmp ;
       pdf.Mu(:,j) = mu_tmp ;

       if ~isempty(pdf.Cov)
          pdf.Cov{j} = pdf.smod.ps.Cov{j} + pdf.smod.H ;
       end
end
% forward transform the additional_data if it exists
if ~isempty(additional_data)
   additional_data = F_trns*(additional_data - repmat(new_mu,1,size(additional_data,2))) ;
   additional_data = additional_data(id_valid,:) ;
end

 

% if variable bandwidths activated
if pdf.smod.useVbw == 1
    pdf = recalculateLocalVariableKDE( pdf ) ;
end
 
if ~isempty(kde_scale)
    kde_scale.Mu = F_trns*(kde_scale.Mu - new_mu)  ;  
    kde_scale.Mu = kde_scale.Mu(id_valid,:) ;
    C_tmp = F_trns*kde_scale.Cov*F_trns' ;
    kde_scale.Cov = C_tmp(id_valid,id_valid)  ;
end

% transform also the global covariance
globalCov = F_trns*globalCov*F_trns' ;
globalCov = globalCov(id_valid, id_valid) ;

output.svdRes.V = V ;
output.svdRes.S = S ;
output.svdRes.id_valid = id_valid ;
output.svdRes.new_mu = new_mu ;
output.svdRes.nullspace.id_null = id_null ;
output.svdRes.nullspace.id_nullVals = id_nullVals ;
output.svdRes.nullspace.id_valid = id_valid ;
output.svdRes.isCompletelySingular = isCompletelySingular ;
output.globalCov = globalCov ;
output.pdf = pdf ;
output.allLayers = allLayers ;
output.kde_scale = kde_scale ;
output.additional_data = additional_data ;

