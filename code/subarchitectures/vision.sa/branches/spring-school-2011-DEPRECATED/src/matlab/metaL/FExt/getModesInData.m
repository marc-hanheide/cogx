function [clustCent,num_in_cluster] = getModesInData( data )
 
% clustCent ... centri rojev
% data2cluster ... za vsak podatek indeks kateremu roju pripada
% cluster2dataCell ... za vsak roj indeksi podatkov, ki podpirajo ta roj

scale =1;
minTol = 1e-5;
% centriraj in preskaliraj za numerièno stabilnost
C = cov(data) + eye(size(data,2))*minTol;
[U,S,V] = svd(C) ; 
F_trns = inv(V*sqrt(S)) ;
M = mean(data) ;
datCentered = data - repmat(M,size(data,1),1) ;
dd = zeros(size(data,1),size(data,2)) ; 
for i = 1 : size(data,1) 
    dd(i,:) = (F_trns*datCentered(i,:)')' ; 
end

% izraèunaj žez palec jedro za ocenjevanje porazdelitve
N_eff = size(data,1) ;
d = size(data,2) ;
ww = ones(1,size(data,1)) / size(data,1) ;
H = ones(1,d)*(4/((d+2)*N_eff))^(2/(d+4))*scale^2 ;

% poišèi toèke, ki pod porazdelitvijo konvergirajo k istemu lokalnemu
% maksimumu na porazdelitvi
[clustCent,data2cluster,cluster2dataCell] = MeanShiftCluster(dd',ww, H, 0, 2) ;

% pretransformiraj centre nazaj v originalne podatke
clustCent = clustCent' ;
for i = 1 : size(clustCent,1) 
    clustCent(i,:) = (inv(F_trns)*clustCent(i,:)')' + M ; 
end

num_in_cluster = [] ;
for i = 1 : length(cluster2dataCell)
    num_in_cluster = [ num_in_cluster , length(cluster2dataCell{i})] ; 
end

[num_in_cluster, idx] = sort(num_in_cluster, 'descend') ;
clustCent = clustCent(idx,:)' ;
 
% ----------------------------------------------------------------------- %
function [clustCent,data2cluster,cluster2dataCell] = MeanShiftCluster(dataPts,weightPts, ...
                                                     bandWidth, plotFlag, typeOfKernel)
%performs multivariate MeanShift Clustering of weighted data using a flat OR a Gaussian kernel
%
% ---INPUT---
% dataPts           - input data, (numDim x numPts)
% weightPts         - input weights of data, (1 x numPts); weights do not
%                     need to be normalized.
% bandWidth         - is bandwidth parameter (scalar or vector)
% plotFlag          - display output if 2 or 3 D    (logical)
% typeOfKernel      - type of MeanShif kernel (1 for Flat, 2 for Gaussian)
%                     default value is typeOfKernel = 2
% ---OUTPUT---
% clustCent         - is locations of cluster centers (numDim x numClust)
% data2cluster      - for every data point which cluster it belongs to (numPts)
% cluster2dataCell  - for every cluster which points are in it (numClust)
% 
% Matej Kristan 11/04/07
% This code was derived from Bryan Feldman's (02/24/06) flat kernel mean
% shift code.
% 
% Added features:
%      * Can use a Gaussian kernel
%      * Can use weighted data
%      * Can use multivariate kernel with different kernel widths in each
%        dimension
%      * For efficiency, the function "sqdist" from Thomas Minka's 
%        "lightspeed Matlab toolbox" is used. Therefore, the toolbox should
%        be installed or at least that function should be in the same root as
%        the mean shift clustering. 
%
% MeanShift first appears in
% K. Funkunaga and L.D. Hosteler, "The Estimation of the Gradient of a
% Density Function, with Applications in Pattern Recognition"

% todo: clusterVotes should be made "soft" for Gaussian kernel

%*** Check input ****
if nargin < 3
    error('no bandwidth specified')
end

if nargin < 4
    plotFlag = true;
    plotFlag = false;
end

if nargin < 5 | typeOfKernel < 1
    typeOfKernel = 2 ; % Gaussian kernel
end

%**** Initialize stuff ***
[numDim,numPts] = size(dataPts);
numClust        = 0;
L_kernel        = inv(diag(bandWidth.^2)) ;                %percision matrix, inverse kernel covariance matrix 
initPtInds      = 1:numPts;
maxPos          = max(dataPts,[],2);                          %biggest size in each dimension
minPos          = min(dataPts,[],2);                          %smallest size in each dimension
boundBox        = maxPos-minPos;                        %bounding box size
sizeSpace       = norm(boundBox);                       %indicator of size of data space
stopThresh      = 1e-3 ;                                %when mean has converged
clustCent       = [];                                   %center of clust
beenVisitedFlag = zeros(1,numPts,'uint8');              %track if a points been seen already
numInitPts      = numPts;                               %number of points to posibaly use as initilization points
clusterVotes    = zeros(1,numPts,'uint16');             %used to resolve conflicts on cluster membership
distThreshold   = 0.5 ;                                 %threshold to merge detected centers  

if typeOfKernel == 2                                    %if Gaussian kernel is used 
    if isempty(weightPts) 
       weightPts = dataPts(1,:)*0 + 1 ;
    end
end

if ~isempty(weightPts)                                  %in case of weighted dataPoints
    weightPts = weightPts / sum(weightPts) ;           
end


while numInitPts

    tempInd         = ceil( (numInitPts-1e-6)*rand);        %pick a random seed point
    stInd           = initPtInds(tempInd);                  %use this point as start of mean
    myMean          = dataPts(:,stInd);                           % intilize mean to this points location
    myMembers       = [];                                   % points that will get added to this cluster                          
    thisClusterVotes    = zeros(1,numPts,'uint16');         %used to resolve conflicts on cluster membership

    while 1     %loop untill convergence
        
%       sqDistToAll = sum((repmat(myMean,1,numPts) - dataPts).^2,1);    %dist squared from mean to all points still active
        sqDistToAll = sqdist(myMean, dataPts, L_kernel) ; %squared Mahalanobis distance to all points still active
        inInds      = find(sqDistToAll < 1);               %points within bandWidth
        thisClusterVotes(inInds) = thisClusterVotes(inInds)+1;  %add a vote for all the in points belonging to this cluster
        
        
        myOldMean   = myMean;                                   %save the old mean
        if typeOfKernel == 1                                    %compute the new mean
            myMean      = mean(dataPts(:,inInds),2);            
        elseif typeOfKernel == 2
            Weights     = exp(-0.5*sqDistToAll).*weightPts ; 
            sumNorm     = sum(Weights) ;
            Weights     = repmat(Weights, numDim, 1) ;  
            myMean      = sum(dataPts.*Weights,2)/sumNorm ;
        else
            error('Unknown type of kernel!') ;
        end
        myMembers   = [myMembers inInds];                       %add any point within bandWidth to the cluster
        beenVisitedFlag(myMembers) = 1;                         %mark that these points have been visited
        
        %*** plot stuff ****
        if plotFlag
            figure(12345),clf,hold on
            if numDim == 2
                plot(dataPts(1,:),dataPts(2,:),'.')
                plot(dataPts(1,myMembers),dataPts(2,myMembers),'ys')
                plot(myMean(1),myMean(2),'go')
                plot(myOldMean(1),myOldMean(2),'rd')
                pause(0.01)
            end
        end

        %**** if mean doesn't move much stop this cluster ***
        distToOther = sqrt(sqdist(myMean, myOldMean, L_kernel)) ;
        if distToOther < stopThresh
            
            %check for merge posibilities
            mergeWith = 0;
            for cN = 1:numClust
                distToOther = sqrt(sqdist(myMean, clustCent(:,cN), L_kernel)) ; 
                %distToOther = norm(myMean-clustCent(:,cN));     %distance from posible new clust max to old clust max
                if distToOther < distThreshold                    %if its within bandwidth/2 merge new and old
                    mergeWith = cN;
                    break;
                end
            end
            
            
            if mergeWith > 0    % something to merge
                clustCent(:,mergeWith)       = 0.5*(myMean+clustCent(:,mergeWith));             %record the max as the mean of the two merged (I know biased twoards new ones)
                %clustMembsCell{mergeWith}    = unique([clustMembsCell{mergeWith} myMembers]);   %record which points inside 
                clusterVotes(mergeWith,:)    = clusterVotes(mergeWith,:) + thisClusterVotes;    %add these votes to the merged cluster
            else    %its a new cluster
                numClust                    = numClust+1;                   %increment clusters
                clustCent(:,numClust)       = myMean;                       %record the mean  
                %clustMembsCell{numClust}    = myMembers;                    %store my members
                clusterVotes(numClust,:)    = thisClusterVotes;
            end

            break;
        end

    end
    
    
    initPtInds      = find(beenVisitedFlag == 0);           %we can initialize with any of the points not yet visited
    numInitPts      = length(initPtInds);                   %number of active points in set

end

[val,data2cluster] = max(clusterVotes,[],1);                %a point belongs to the cluster with the most votes

%*** If they want the cluster2data cell find it for them
if nargout > 2
    cluster2dataCell = cell(numClust,1);
    for cN = 1:numClust
        myMembers = find(data2cluster == cN);
        cluster2dataCell{cN} = myMembers;
    end
end


