%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function kde_out = mergeKDEs( kdes )
% merge cell array of KDEs into a single pdf-based kde

pdf.suffStat.B = {} ;
pdf.suffStat.A = {} ;
pdf.suffStat.subLayer = [] ;
pdf.Mu = [] ;
pdf.Cov = {} ;
pdf.w = [] ;

% create prior weights
n_kdes = length(kdes) ;
w = zeros(1,n_kdes) ;
N_eff = 0 ;
for i = 1 : n_kdes
   [ kdes{i}.pdf, H_out ] = readjustKernels( kdes{i}.pdf, 0, 0 ) ;
    
   w(i) = kdes{i}.ikdeParams.N_eff  ;
   N_eff = N_eff + kdes{i}.ikdeParams.N_eff ;
end
w = w / sum(w) ;


% merge all kdes into a single kde
for i = 1 : n_kdes
    pdf.Mu = horzcat(pdf.Mu, kdes{i}.pdf.Mu) ;
    pdf.Cov = horzcat(pdf.Cov, kdes{i}.pdf.Cov) ;
    pdf.w = [pdf.w, w(i)*kdes{i}.pdf.w] ;
    pdf.suffStat.B = horzcat(pdf.suffStat.B,kdes{i}.pdf.suffStat.B) ;
    pdf.suffStat.A = horzcat(pdf.suffStat.A,kdes{i}.pdf.suffStat.A) ;
    pdf.suffStat.subLayer = horzcat(pdf.suffStat.subLayer,kdes{i}.pdf.suffStat.subLayer) ;
end
    
kde_out = kdes{1} ;
kde_out.ikdeParams.N_eff = N_eff ;
kde_out.pdf = pdf ;
kde_out.pdf = readjustKernels( kde_out.pdf, H_out, 0 ) ;

% take care of the covariance scales
Mu = [] ;
Cov = {} ;
for i = 1 : n_kdes
    rescale0 =  max([1, kdes{i}.ikdeParams.N_eff/( kdes{i}.ikdeParams.N_eff - 1 )]) ;
    Mu = horzcat(Mu, kdes{i}.ikdeParams.scale.Mu) ;
    Cov = horzcat(Cov, kdes{i}.ikdeParams.scale.Cov/rescale0) ;
end

rescale1 =  max([1, kde_out.ikdeParams.N_eff/( kde_out.ikdeParams.N_eff - 1 )]) ;
[new_mu, new_Cov, w_out] = momentMatchPdf(Mu, Cov, w) ;
kde_out.ikdeParams.ikdeParams.scale.Cov = new_Cov*rescale1 ;
kde_out.ikdeParams.ikdeParams.scale.Mu = new_mu ;


