function [pdf_out, ikdeParams] = updateAdaptiveMixtures( pdf_in, dat, ikdeParams, otherParams )


if ~isempty(pdf_in.w) && size(dat,2) > 1
    error('Adaptive models does not include functionality to update from multiple observations at a time.') ;
end

wth = 1e-5 ;
f = ikdeParams.suffSt.w_att ;
scl_in = 1; %0.8^2 ; % 0.5^2; %0.5^2 ;
% N_eff = ikdeParams.N_eff ;
% ikdeParams.N_eff = N_eff ;
N_pdf = ikdeParams.N_eff * f ;
N_dat = size(dat,2) ;
N_new = N_pdf + N_dat ;
pdf_out = pdf_in ;
d = size(dat,1) ;
% if initialization
if isempty(pdf_in) || isempty(pdf_in.w) 
    if size(dat,2) == 1
        C1 = eye(size(dat,1)) ;   
    else
        C0 = cov(dat') ;
        C1 =  C0*(4/((d+2)*N_new))^(2/(d+4)) * scl_in;     
        ikdeParams.scale.Cov = C0 ;
        ikdeParams.scale.Mu = mean(dat,2) ;
    end
    pdf_out.w = ones(1,size(dat,2)) ;
    pdf_out.w = pdf_out.w / sum(pdf_out.w) ;
    pdf_out.Mu = dat ;
    pdf_out.Cov = {} ;
    for i = 1 : length(pdf_out.w)
        pdf_out.Cov = horzcat(pdf_out.Cov, C1) ;
    end    
    ikdeParams.N_eff = N_new ;
    return ;
end


% find and remove bad components
idbad = pdf_out.w < wth ;
idsel = ones(1, length(pdf_out.w)) ;
idsel(idbad) = 0 ;
pdf_out = extractSubMixture( pdf_out , find(idsel) ) ;
pdf_out.w = pdf_out.w / sum(pdf_out.w)  ;

% create list of new components
createNew_list = zeros(1,size(dat,2)) ;
for j = 1 : size(dat,2)
    sel = getClosestKernelForThisData( pdf_in, dat ) ;
%     lam = exp(-0.5*sel.val) ;
%     createNew_list(j) = lam < rand() ; %sel.val > 2 ;% lam < rand() ; sel.val > 0.6 ;   % %
    createNew_list(j) = sel.val > 2.34 ; % according to Cwick, Koronacki 1997
end

% generate new components
id = find(createNew_list==1) ;
if ~isempty(id)
    % % % Mu = horzcat(pdf_out.Mu, dat(:,id)) ;
    % % % w = horzcat(pdf_out.w * N_eff/N_new , ones(1,length(id)) * N_dat/N_new) ;
    % % % Cov_tmps = pdf_out.Cov ;
    % % % for j = 1 : length(id)
    % % %    Cov_tmps = horzcat(Cov_tmps, zeros(d) );
    % % % end
    % % % [new_mu, new_Cov, w_out] = momentMatchPdf(Mu, Cov_tmps, w) ;
    Mu = [ ikdeParams.scale.Mu, dat] ;
    w =  horzcat( N_pdf/N_new , ones(1,length(id)) * N_dat/N_new) ;
    Cov_tmps = { ikdeParams.scale.Cov } ;
    for j = 1 : length(id)
        Cov_tmps = horzcat(Cov_tmps, zeros(d) );
    end
    [new_mu, new_Cov, w_out] = momentMatchPdf(Mu, Cov_tmps, w) ;
    ikdeParams.scale.Cov = new_Cov ;
    ikdeParams.scale.Mu = new_mu ;    


% create bandwidth matrix
new_Cov = ikdeParams.scale.Cov ;
C1 = new_Cov * (4/((d+2)*N_new))^(2/(d+4)) * scl_in;
% create matrix  according to Cwick, Koronacki 1997 
% C1 = C1*0 ;
% for j = 1 : length(pdf_out.w)
%    C1 = C1 + pdf_out.w(j)*pdf_out.Cov{j} ;     
% end

% add new components to pdf
for j = 1 : length(id)
   pdf_out.Cov = horzcat( pdf_out.Cov, C1 ) ;     
end
pdf_out.Mu = horzcat(pdf_out.Mu, dat(:,id)) ;
pdf_out.w = horzcat(pdf_out.w * N_pdf/N_new , (ones(1,length(id))/max([1,length(id)])) * N_dat/N_new) ;

end
% N_dat = length(id) ;
% N_new = N_eff*f + N_dat ;
% pdf_out.w = horzcat(pdf_out.w * N_eff/N_new , ones(1,length(id)) * N_dat/N_new) ;
% pdf_out.w = pdf_out.w  / sum(pdf_out.w ) ;
% N_new = N_eff*f + size(dat,2) ;
% ikdeParams.N_eff = N_new ;

if N_dat > 1
    error('Wont work!') ;
end

  
% update components
id = find(createNew_list==0) ;
for j = 1 : length(id)
    i = id(j) ;
    x = dat(:,i) ;
    p = evaluatePointsUnderPdf(pdf_out, x) ; 

    for k = 1 : length(pdf_out.w)
        pdfx.Mu = pdf_out.Mu(:,k) ;
        pdfx.w = pdf_out.w(k) ;
        pdfx.Cov = pdf_out.Cov(k) ;
        p_k = evaluatePointsUnderPdf(pdfx, x) ;
        tau = p_k / p ; 
 
        pdfx.w = [ pdfx.w*N_pdf/N_new, N_dat/N_new*tau ]  ;
        pdfx.Mu = horzcat(pdfx.Mu, x) ;
        pdfx.Cov = horzcat(pdfx.Cov, zeros(d)) ;
        [new_mu, new_Cov, w_out] = momentMatchPdf(pdfx.Mu, pdfx.Cov, pdfx.w) ;

        pdf_out.w(k) = w_out ;
        pdf_out.Mu(:,k) = new_mu ;
        pdf_out.Cov{k} = new_Cov ;
         
    end
end
ikdeParams.N_eff = N_new ;


% % 
% % % update components
% % id = find(createNew_list==0) ;
% % for j = 1 : length(id)
% %     i = id(j) ;
% %     x = dat(:,i) ;
% %     p = evaluatePointsUnderPdf(pdf_out, x) ;
% %     for k = 1 : length(pdf_out.w)
% %         pdfx.Mu = pdf_out.Mu(:,k) ;
% %         pdfx.w = pdf_out.w(k) ;
% %         pdfx.Cov = pdf_out.Cov(k) ;
% %         p_k = evaluatePointsUnderPdf(pdfx, x) ;
% %         tau = p_k / p ;
% %         
% %         w_k = pdf_out.w(k) ;
% %         M_k = pdf_out.Mu(:,k) ; 
% %         C_k = pdf_out.Cov{k} ;
% %         pdf_out.w(k) = w_k + (1/N_new)*(tau - w_k) ;
% %         pdf_out.Mu(:,k) = M_k + ((1/N_new)*tau/w_k)*(x-M_k) ;
% %         pdf_out.Cov{k} = C_k + ((1/N_new)*tau/w_k)*( (x-M_k)*(x-M_k)' - C_k) ;   
% %     end
% % end

%%% ------------------------------------------------------------- %%%
 
% maxN = 10 ;
% % reduce if required
% scale_first = 1.5 ;
% scale_mid = 0.5 ;
% scale_last = 0.6 ;
% N = abs(ikdeParams.maxNumCompsBeforeCompression) ;
% if length(pdf_out.w) > N
%     pdf_out = iapr_reduction( pdf_out, otherParams.compressionClusterThresh, maxN ) ;
%     len_new = length(pdf_out.w) ;
%     if  ( len_new > N )
%         N = N*scale_first ;
%     elseif len_new <= N*scale_mid
%         N = N*scale_last ;
%     end    
%     ikdeParams.maxNumCompsBeforeCompression = N ;
% end

% --------------------------------------------------------------------- %
function pdf_in = iapr_reduction( pdf_in, minDistance, maxN )
minDistance = (minDistance^2)*2;
refval = inf ;
n = length(pdf_in.w) ;
H = ones(n,n)*refval ;
for i = 1 : n
    for j = i+1 : n
        H(i,j) = hellinger2Norm( pdf_in.w(i), pdf_in.Mu(:,i), pdf_in.Cov{i},...
                            pdf_in.w(j), pdf_in.Mu(:,j), pdf_in.Cov{j} ) ;                        
    end
end



while 1==1
%     minDistance = refval ;
%     if length(pdf_in.w) < maxN
%         break ;
%     end
    
    [vals, locs] = min(H) ;
    [vals2, loc] = min(vals) ;
    if vals2 > minDistance
        break ;
    end
    cl = loc ;
    rw = locs(loc) ;
    
    idx = ones(1,length(pdf_in.w)) ;
    rm = [cl, rw] ;
    idx(rm) = 0 ;
    id_remain = find(idx==1) ;
    pdf2 = extractSubMixture( pdf_in, id_remain ) ;
    pdf_tmp = extractSubMixture( pdf_in, rm ) ;
    [new_mu, new_Cov, w_out] = momentMatchPdf(pdf_tmp.Mu, pdf_tmp.Cov, pdf_tmp.w) ;
    pdf_in = pdf2 ;
    pdf_in.Mu = horzcat(pdf2.Mu, new_mu) ;
    pdf_in.Cov = horzcat(pdf2.Cov, new_Cov) ;
    pdf_in.w = horzcat(pdf2.w, w_out) ;
    
    Hin = ones(1,length(pdf_in.w))*refval ;
    j = length(pdf_in.w) ;
    for k = 1 : length(pdf_in.w)-1
        Hin(k) = hellinger2Norm( pdf_in.w(k), pdf_in.Mu(:,k), pdf_in.Cov{k},...
            pdf_in.w(j), pdf_in.Mu(:,j), pdf_in.Cov{j} ) ;
    end 
    H = H(id_remain,id_remain) ;
    
    H = [H, Hin(1:length(Hin)-1)'] ;
    H = [H; Hin*refval] ;
end
 
 



% --------------------------------------------------------------------- %
function sel = getClosestKernelForThisData( pdf, dat )

sel.i = 1 ;
sel.val = inf ;
% for all kernels
for i = 1 : length(pdf.w)
    C = pdf.Cov{i} ;
    M = pdf.Mu(:,i) ;
    
    d = (M - dat)'*inv(C)*(M - dat) ;
    
    if d < sel.val
       sel.i = i ;
       sel.val = d ;
    end    
end





    


