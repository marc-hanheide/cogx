function [pdf, obs]  = generateBivariateDensity( no, N )

% Implements Wand1993 densities 
% only one curently

% AM delujejo na originalni MaWa slabše, èe je malo zaèetnih vzorcev, ker
% slabo ocenijo strukturo porazdelitve!

% pdf.w = [1 1 1]/3 ;
% pdf.Mu = [[-6/5;0], [6/5;0], [0;0]] ;
% pdf.Cov = {} ;
% a = 3/5 ; r = 7/10 ;
% C = [a^2, (a^2)*r; (a^2)*r, a^2] ; pdf.Cov = horzcat(pdf.Cov,C) ;
% pdf.Cov = horzcat(pdf.Cov,C) ;
% a = 3/5 ; r = -7/10 ;
% C = [a^2, (a^2)*r; (a^2)*r, a^2] ; pdf.Cov = horzcat(pdf.Cov,C) ;

% pdf.w = [1 1 1 1 0.5 0.5 0.5] ; pdf.w = pdf.w / sum(pdf.w) ;
% pdf.Mu = [[-6/5;0], [6/5;0], [0;0], [-12/5;0], [-1.8;-0.6], [0.6;-0.6], -[0.6;-0.6]] ;
% pdf.Cov = {} ;
% a = 3/5 ; r = 7/10 ;
% C = [a^2, (a^2)*r; (a^2)*r, a^2] ; pdf.Cov = horzcat(pdf.Cov,C) ;
% pdf.Cov = horzcat(pdf.Cov,C) ;
% a = 3/5 ; r = -7/10 ;
% C = [a^2, (a^2)*r; (a^2)*r, a^2] ; pdf.Cov = horzcat(pdf.Cov,C) ;
% pdf.Cov = horzcat(pdf.Cov,C) ;
% C = diag([0.3^2, 0.3^2]) ; pdf.Cov = horzcat(pdf.Cov,C) ;
% pdf.Cov = horzcat(pdf.Cov,C) ; pdf.Cov = horzcat(pdf.Cov,C) ;

% pri tej izgleda, da AM deluje nekoliko slabše v smislu L1 norme
pdf.w = [1 1 1 1 0.5 0.5] ; pdf.w = pdf.w / sum(pdf.w) ;
pdf.Mu = [[-6/5;0], [6/5;0], [0;0], [-12/5;0], [0.6;-0.6], -[0.6;-0.6]] ;
pdf.Cov = {} ;
a = 3/5 ; r = 7/10 ;
C = [a^2, (a^2)*r; (a^2)*r, a^2] ; pdf.Cov = horzcat(pdf.Cov,C) ;
pdf.Cov = horzcat(pdf.Cov,C) ;
a = 3/5 ; r = -7/10 ;
C = [a^2, (a^2)*r; (a^2)*r, a^2] ; pdf.Cov = horzcat(pdf.Cov,C) ;
pdf.Cov = horzcat(pdf.Cov,C) ;
C = diag([0.3^2, 0.3^2]) ; pdf.Cov = horzcat(pdf.Cov,C) ;
pdf.Cov = horzcat(pdf.Cov,C) ; 

obs = sampleGaussianMixture( pdf, N ) ;