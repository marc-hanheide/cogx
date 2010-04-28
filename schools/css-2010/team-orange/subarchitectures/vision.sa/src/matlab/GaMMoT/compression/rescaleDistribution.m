function pdf = rescaleDistribution( pdf, scale, shift, transform_type )
%
% Matej Kristan (2007)
%
% Scales distribution to another interval. 
% Proper scale is important so that optimization 
% is well conditioned.
%
% Forward transform:
% transform of mean values:  M_new = (M_old - shift) * scale ;
% transform of covariances:  Cov_new = Cov_old*scale^2 ;
%
% Backward transform:
% transform of mean values:  M_new = M_old/scale + shift  ;
% transform of covariances:  Cov_new = Cov_old/scale^2 ;
%


dim = cols(pdf.mu) ;
len = length(pdf.weights) ;

if ( isequal(transform_type,'forward') )
    % transform mean values
    pdf.mu = (pdf.mu - repmat(shift,1,len))*scale ;
    %transform of covariances
    pdf.covariances = pdf.covariances*scale.^2 ;
elseif ( isequal(transform_type,'backward') )
    pdf.mu = pdf.mu/scale + repmat(shift,1,len) ;
    pdf.covariances = pdf.covariances/scale.^2 ;
else
    error(['Unknown transform parameter: ', transform_type])
end