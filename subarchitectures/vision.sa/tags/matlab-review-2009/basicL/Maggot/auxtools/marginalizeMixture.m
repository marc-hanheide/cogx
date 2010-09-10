%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function pdf = marginalizeMixture( pdf, dim_curr, ignoreSublayer )
% calculates a pdf marginalized over all dimensions
% except for dim_curr

if nargin < 3
    ignoreSublayer = 0 ;
end

suffstatIn = 0 ;
if isfield(pdf,'suffStat')
    suffstatIn = 1 ;
end

if ignoreSublayer == 1
    suffstatIn = 0 ;
end

N = size(pdf.Mu, 2) ;
d = size(pdf.Mu, 1) ;
pdf.Mu = pdf.Mu(dim_curr,:) ;

% select only the dim_cur'th diagonal elements of the covariance matrices
% C = cell2mat(pdf.Cov) ;
for i = 1 : length(pdf.w)
    pdf.Cov{i} = pdf.Cov{i}(dim_curr, dim_curr) ;
    
    % modify also suffStat components if they exist
    if suffstatIn == 1
        pdf.suffStat.B{i} = pdf.suffStat.B{i}(dim_curr, dim_curr) ;
        pdf.suffStat.A{i} = pdf.suffStat.A{i}(dim_curr, dim_curr) ; 
        
        pdf.suffStat.subLayer(i).Mu = pdf.suffStat.subLayer(i).Mu(dim_curr,:) ;
        for j = 1 : length(pdf.suffStat.subLayer(i).w)
            pdf.suffStat.subLayer(i).Cov{j} = pdf.suffStat.subLayer(i).Cov{j}(dim_curr,dim_curr) ;
            pdf.suffStat.subLayer(i).B{j} = pdf.suffStat.subLayer(i).B{j}(dim_curr,dim_curr) ;
            pdf.suffStat.subLayer(i).A{j} = pdf.suffStat.subLayer(i).A{j}(dim_curr,dim_curr) ;
        end
    end
end

% idx = [dim_curr:d:N*d] ; 
% pdf_tmp.Cov = num2cell(C(dim_curr,idx)) ;
% pdf_tmp.w = pdf.w ;