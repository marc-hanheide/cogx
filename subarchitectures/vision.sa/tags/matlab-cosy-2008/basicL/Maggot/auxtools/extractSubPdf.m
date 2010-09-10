%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function pdf = extractSubPdf( pdf , idx, ignoreSublayer )
%
% Uses idxs indexes to recombine components in pdf and outputs the
% result in pdf2.
%

if nargin == 2
    ignoreSublayer = 0 ;
end

pdf.Mu = pdf.Mu(:,idx) ;
pdf.Cov = pdf.Cov(idx) ;
pdf.w = pdf.w(idx)  ;

suffStat = [] ;
if isfield(pdf,'suffStat') && ignoreSublayer==0 
    suffStat.A = pdf.suffStat.A(idx) ;
    suffStat.B = pdf.suffStat.B(idx) ;
end
pdf.suffStat = suffStat ;
 