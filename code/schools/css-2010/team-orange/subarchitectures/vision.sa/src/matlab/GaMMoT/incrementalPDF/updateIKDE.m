function kernel_pdf = updateIKDE( kernel_pdf, data, varargin )
 
makeSeparateKDEforNeg = 0 ;
unlearning = 0 ;
turnOffErrorValve = 0 ;
initByGaussian = [] ;
modfyHell = 0 ;
typeOperation = 'accurate' ;
initializationMethod = 'Plugin' ; % 'Silverman'
reportProgress = 0 ;
recordMode = 0 ;
initialize = 0 ;
nMaxComponentsPrior = 10 ;
scaleErrorThreshold = 1/0.7 ;
hellErrorGlobal = 0.1 ;
compression = 1 ;
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'hellErrorGlobal', hellErrorGlobal = args{i+1} ; modfyHell = 1 ;   
        case 'compression', compression = args{i+1} ;
        case 'initialize', initialize = args{i+1} ;
        case 'unlearning', unlearning = args{i+1} ;
        case 'nMaxComponentsPrior', nMaxComponentsPrior = args{i+1} ;
        case 'scaleErrorThreshold', scaleErrorThreshold = args{i+1} ;   
        case 'recordMode', recordMode = args{i+1} ;
        case 'reportProgress', reportProgress = args{i+1} ;
        case 'initializationMethod', initializationMethod = args{i+1} ;
        case 'typeOperation', typeOperation = args{i+1} ;
        case 'initByGaussian', initByGaussian = args{i+1} ;
        case 'makeSeparateKDEforNeg', makeSeparateKDEforNeg = args{i+1} ;
    end
end

if ( unlearning == 1 )
   kernel_pdf = unlearnKDEwithSamples( kernel_pdf, data, ...
                                       'compression', compression, ...
                                       'makeSeparateKDEforNeg', makeSeparateKDEforNeg) ; 
   return ;                                   
end

if ( initialize == 1 )
    kernel_pdf = initializeKDE( data, nMaxComponentsPrior, ...
                                hellErrorGlobal, scaleErrorThreshold,...
                                initializationMethod, typeOperation,...
                                initByGaussian ) ;
    return ;
end

if ( modfyHell == 1 ) 
    kernel_pdf.pars2.hellErrorGlobal = hellErrorGlobal ;
end

if ( compression == 0 )
    kernel_pdf.nMaxComponents = length(kernel_pdf.weights) + length(data)*2 ; 
end

% if there is any data to add
if ( ~isempty(data) )
    % update scale of kde
    [kernel_pdf.scale.mu,kernel_pdf.scale.sg,n_sg] = updateMV(data',kernel_pdf.scale.mu,kernel_pdf.scale.sg,kernel_pdf.components) ;
    kernel_x0 = [] ;
    C_sensor = 0 ;
    % update kde
    [kernel_pdf, C_cov] = updateKDE( kernel_pdf, kernel_x0, data, kernel_pdf.scale.sg, n_sg, [], 1, C_sensor, kernel_pdf.scale.mu ) ;
    if (recordMode == 0)  C_cov = [] ; end
    kernel_pdf.Curr_covariances = [kernel_pdf.Curr_covariances, C_cov] ;
end

% compress kde if it esceeds the number of components or if data is empty
if ( isempty(data) || length(kernel_pdf.weights) > kernel_pdf.nMaxComponents )
    scale = kernel_pdf.scale ;
    nMaxComponents = kernel_pdf.nMaxComponents ;
    components = kernel_pdf.components ;
    pars = kernel_pdf.pars ;
    pars2 = kernel_pdf.pars2 ;
    Curr_covs = kernel_pdf.Curr_covariances ;
     
    kernel_pdf = compressDistribution( kernel_pdf, 'showIntermediate',  0,...
                                       'hellErrorGlobal', kernel_pdf.pars2.hellErrorGlobal,...
                                       'scaleErrorThreshold', kernel_pdf.pars2.scaleErrorThreshold,...
                                       'reportProgress', reportProgress,...
                                       'typeOperation', pars2.typeOperation,...
									   'turnOffErrorValve', turnOffErrorValve,...									   
                                        varargin{:} ) ;
    kernel_pdf.scale = scale ;
    kernel_pdf.nMaxComponents = nMaxComponents ;
    kernel_pdf.components = components ;
    kernel_pdf.pars = pars ;
    kernel_pdf.pars2 = pars2 ;
    kernel_pdf.Curr_covariances = Curr_covs ; 

    % reset the threshold if possible
    if ( length(kernel_pdf.weights) > kernel_pdf.nMaxComponents  )
        kernel_pdf.nMaxComponents = 1.5 * kernel_pdf.nMaxComponents ;
    elseif ( length(kernel_pdf.weights)<  kernel_pdf.nMaxComponents/1.5 )
        kernel_pdf.nMaxComponents =  kernel_pdf.nMaxComponents/1.5 ;
    end
end

% ----------------------------------------------------------------- %
function kernel_pdf = initializeKDE( data, nMaxComponentsPrior,...
                                     hellErrorGlobal, scaleErrorThreshold,...
                                     initializationMethod, typeOperation,...
                                     initByGaussian) 
 
nMaxComponents = nMaxComponentsPrior ;

if isempty(initByGaussian)

    mu = mean(data) ; sg = cov(data) ;
    % initialize covariances
    if ( isequal(initializationMethod,'Plugin') )
        pdf = get_1d_OptimalKDE( data ) ;
    elseif ( isequal(initializationMethod,'Silverman') )
        pdf.covariances = ones(length(data),1) * getSilvermanBWfromGaussian( sg,  length(data) ) ;
    else
        error(sprintf('Unknown initialization method: %s',initializationMethod) ) ;
    end

    C = pdf.covariances(1) ;
    covariances = ones(length(data),1) * C ;
    num_components = length(data) ;
else
    data = initByGaussian.mu ;
    covariances = initByGaussian.covariances ;
    mu = initByGaussian.mu ;
    sg = initByGaussian.covariances ;
    num_components = initByGaussian.num_components ;
end
 
kernel_pdf.mu = data ;
kernel_pdf.weights = ones(1, length(data))/sum(ones(1, length(data))) ;  
kernel_pdf.covariances = covariances ;
kernel_pdf.components = num_components ;
kernel_pdf.pars.Wt1 = 1 ; 
kernel_pdf.pars.wt1 = 0.5 ;
kernel_pdf.pars.histCompNums = [1 1] ;
kernel_pdf.pars2.hellErrorGlobal = hellErrorGlobal ;
kernel_pdf.pars2.scaleErrorThreshold = scaleErrorThreshold ;
kernel_pdf.pars2.typeOperation = typeOperation ;

kernel_pdf.scale.mu = mu ; 
kernel_pdf.scale.sg = sg ;
kernel_pdf.nMaxComponents = nMaxComponents ;
kernel_pdf.Curr_covariances = covariances(:)' ; 

% if ( isequal(typeOperation,'fast') && hellErrorGlobal < 0.15 )
%     kernel_pdf.pars2.hellErrorGlobal = 0.15 ; 
% end