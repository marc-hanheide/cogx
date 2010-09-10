%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function output = subspacePrewhitenTransform( varargin )
%
% transforms a mixture model forward or backward using intrinsic subspace.
%
%
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
    end
end

switch(transDirection)
    case 'forward'
        output = goForwardTrans( pdf, globalCov, minEigenEnergy,...
                                 allLayers, svdRes, regularize, kde_scale ) ;        
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
for j = 1 : length(pdf.w)
    mu_o = pdf.Mu(:,j) ;
 
    pdf.Mu(:,j) = F_trns*pdf.Mu(:,j) + svdRes.new_mu ;
    
    C_prot = C_prot*0 ;
    C_prot(svdRes.id_valid,svdRes.id_valid) = pdf.Cov{j} ;    
    pdf.Cov{j} = F_trns*C_prot*F_trns' ;
    
   if isfield(pdf,'suffStat')
       C_prot = C_prot*0 ;
       C_prot(svdRes.id_valid,svdRes.id_valid) = pdf.suffStat.B{j} ;
       pdf.suffStat.B{j} = F_trns*C_prot*F_trns'  ;
       
       C_prot = C_prot*0 ;
       C_prot(svdRes.id_valid,svdRes.id_valid) = pdf.suffStat.A{j} ;
       
       pdf.suffStat.A{j} = F_trns*(C_prot - mu_o*mu_o')*F_trns' + pdf.Mu(:,j)*pdf.Mu(:,j)';
   end
end

if ~isempty(kde_scale)    
    kde_scale.Mu = [ kde_scale.Mu; zeros(num_nullDir,1) ] ;
    kde_scale.Mu = F_trns*kde_scale.Mu + svdRes.new_mu ;
    C_prot = C_prot*0 ;
    C_prot(svdRes.id_valid,svdRes.id_valid) = kde_scale.Cov ;    
    kde_scale.Cov = F_trns*C_prot*F_trns' ;    
end

% do you want to transform also the sublayer?
if allLayers == 1 
    for j = 1 : length(pdf.suffStat.subLayer)        
        % if this is singleton, then it's equal to upper layer
        if length(pdf.suffStat.subLayer(j).w) == 1
            pdf.suffStat.subLayer(j).Mu = pdf.Mu(:,j) ;
            pdf.suffStat.subLayer(j).Cov{1} = pdf.Cov{j} ;
            pdf.suffStat.subLayer(j).B{1} = pdf.suffStat.B{j} ;
            pdf.suffStat.subLayer(j).A{1} = pdf.suffStat.A{j} ;
        else
            % if it's not a singleton
            pdf.suffStat.subLayer(j).Mu = [ pdf.suffStat.subLayer(j).Mu; zeros(num_nullDir,length(pdf.suffStat.subLayer(j).w)) ] ;
            for i = 1 : length(pdf.suffStat.subLayer(j).w)
                mu_o = pdf.suffStat.subLayer(j).Mu(:,i) ;
                pdf.suffStat.subLayer(j).Mu(:,i) = F_trns*pdf.suffStat.subLayer(j).Mu(:,i) + svdRes.new_mu ;
                                
                C_prot = C_prot*0 ;
                C_prot(svdRes.id_valid,svdRes.id_valid) = pdf.suffStat.subLayer(j).Cov{i} ;
                pdf.suffStat.subLayer(j).Cov{i} = F_trns*C_prot*F_trns' ;
                
                C_prot = C_prot*0 ;
                C_prot(svdRes.id_valid,svdRes.id_valid) = pdf.suffStat.subLayer(j).B{i} ;
                pdf.suffStat.subLayer(j).B{i} = F_trns*C_prot*F_trns'  ;
                
                C_prot = C_prot*0 ;
                C_prot(svdRes.id_valid,svdRes.id_valid) = pdf.suffStat.subLayer(j).A{i} ;
                pdf.suffStat.subLayer(j).A{i} = F_trns*(C_prot - mu_o*mu_o')*F_trns' +...
                                                 pdf.suffStat.subLayer(j).Mu(:,i)*pdf.suffStat.subLayer(j).Mu(:,i)';                             
            end
        end
    end
   
end

output.pdf = pdf ;
output.kde_scale = kde_scale ;

% ----------------------------------------------------------------------- %
function output = goForwardTrans( pdf, globalCov, minEigenEnergy, allLayers, svdRes, regularize, kde_scale )
minVals = 1e-10 ;  % was 1e-32
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
    [new_mu, C] = momentMatchPdf( pdf.Mu, pdf.Cov, pdf.w ) ;
    if isempty(globalCov) globalCov = C ; end
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
        S_inv(id_null(i),id_null(i)) = tmp_val ;
    end
    
    new_mu = svdRes.new_mu ;
    F_trns = sqrt(abs(S_inv))* inv(V) ; 
    invF_trns = V * sqrt(S) ;
    % if regularization is required, correct the bandwidth such that
    % the input covariances are valid
    if regularize == 1        
        % extract and analyze the bandwidth subspace
        H_internal = pdf.suffStat.B{1} ;
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
pdft = pdf ;
for j = 1 : length(pdft.w)
    mu_o = pdft.Mu(:,j) ;
    pdft.Mu(:,j) = F_trns*(pdft.Mu(:,j) - new_mu) ;
    
    C_tmp = F_trns*pdft.Cov{j}*F_trns' ;
    pdft.Cov{j} = C_tmp(id_valid,id_valid) ;
 
    B_tmp = F_trns*pdft.suffStat.B{j}*F_trns' ;
    pdft.suffStat.B{j} = B_tmp(id_valid,id_valid)  ;
    A_tmp = F_trns*(pdft.suffStat.A{j} - mu_o*mu_o')*F_trns' + pdft.Mu(:,j)*pdft.Mu(:,j)' ;
    pdft.suffStat.A{j} = A_tmp(id_valid,id_valid) ;
end
pdft.Mu = pdft.Mu(id_valid,:) ;

if ~isempty(kde_scale)
    kde_scale.Mu = F_trns*(kde_scale.Mu - new_mu)  ;  
    kde_scale.Mu = kde_scale.Mu(id_valid,:) ;
    C_tmp = F_trns*kde_scale.Cov*F_trns' ;
    kde_scale.Cov = C_tmp(id_valid,id_valid)  ;
end

% do you want to transform also the sublayer?
if allLayers == 1 
    for j = 1 : length(pdft.suffStat.subLayer)        
        % if this is singleton, then it's equal to upper layer
        if length(pdft.suffStat.subLayer(j).w) == 1
            pdft.suffStat.subLayer(j).Mu = pdft.Mu(:,j) ;
            pdft.suffStat.subLayer(j).Cov{1} = pdft.Cov{j} ;
            pdft.suffStat.subLayer(j).B{1} = pdft.suffStat.B{j} ;
            pdft.suffStat.subLayer(j).A{1} = pdft.suffStat.A{j} ;
        else
            % if it's not a singleton
            for i = 1 : length(pdft.suffStat.subLayer(j).w)
                mu_o = pdft.suffStat.subLayer(j).Mu(:,i) ;
                pdft.suffStat.subLayer(j).Mu(:,i) = F_trns*(pdft.suffStat.subLayer(j).Mu(:,i) - new_mu) ;                
                
                C_tmp = F_trns*pdft.suffStat.subLayer(j).Cov{i}*F_trns' ;
                pdft.suffStat.subLayer(j).Cov{i} = C_tmp(id_valid,id_valid) ;
                
                B_tmp = F_trns*pdft.suffStat.subLayer(j).B{i}*F_trns' ;
                pdft.suffStat.subLayer(j).B{i} = B_tmp(id_valid,id_valid)  ;
                A_tmp = F_trns*(pdft.suffStat.subLayer(j).A{i} - mu_o*mu_o')*F_trns' +...
                        pdft.suffStat.subLayer(j).Mu(:,i)*pdft.suffStat.subLayer(j).Mu(:,i)' ;
                pdft.suffStat.subLayer(j).A{i} = A_tmp(id_valid,id_valid) ;                
            end            
            pdft.suffStat.subLayer(j).Mu = pdft.suffStat.subLayer(j).Mu(id_valid,:) ;
        end
    end        
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
output.pdf = pdft ;
output.allLayers = allLayers ;
output.kde_scale = kde_scale ;

