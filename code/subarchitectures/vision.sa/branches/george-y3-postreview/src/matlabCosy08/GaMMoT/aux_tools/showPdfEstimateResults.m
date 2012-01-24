function showPdfEstimateResults( data, prev_centers, centers, weights, covariances, bounds, parzen_h, weights_data )
% bounds [x_min, x_max, y_min, y_max ]

if nargin < 6 bounds = [] ; end

if nargin < 7 parzen_h = -1 ; end

if nargin < 8 weights_data = [] ; end 

 
dim = sqrt(cols(covariances)) ;
num_centers = length(weights) ; 

switch dim
    case 1
        numpointevals = 100 ;
        % determine bounds
        if ( isempty(bounds) )
             S = max(covariances) ; 
            for i = 1 : cols(data)              
               bounds0 = getBound1D( data(:,i), sqrt(S) ) ; 
               if isempty(bounds)
                  bounds = bounds0 ;
               else
                  bounds(1) = min([bounds(1), bounds0(1)]) ; 
                  bounds(2) = max([bounds(2), bounds0(2)]) ;
               end
            end
        end
        delta = (bounds(2) - bounds(1))/numpointevals ;
        x_evals = [bounds(1) : delta : bounds(2)] ;
        y_evals = evaluatePdfAt( x_evals, centers, covariances, weights );
        normFactor = sum(y_evals) ;
        y_evals = y_evals / normFactor ;
        
        cent_evals = evaluatePdfAt( centers, centers, covariances, weights );
        cent_evals = cent_evals / normFactor ; 
        
        % semi plot evals
        cent_semi_Evals = [] ;
        for i = 1 : cols( centers ) 
            point = centers(:,i) ;
            cent_semi_Evals(i) = evaluatePdfAt( point, point, ...
                                                covariances(i,:),...
                                                weights(:,i) );
        end
        cent_semi_Evals = cent_semi_Evals / normFactor ;
        
        plot(data,zeros(size(data)),'b*') ; hold on ;
        plotUpwardLines( centers, cent_evals ) ;
        plot(x_evals, y_evals, 'b') ;
        plot(prev_centers,zeros(size(prev_centers)),'co') ;
        plot(centers,cent_evals,'mo') ;
        plot(centers, cent_semi_Evals, 'r+') ;
        
%         weights = weights*0+1; weights = weights/sum(weights);
%         data_weights = ones(1,cols(data)) ; 
%         data_weights = data_weights / sum(data_weights) ;
%         data_H = ones(1,cols(data))*parzen_h ;
%         yy = x_evals*0 ;
%         for i = 1 : cols( centers )             
%             ev_point = centers(:,i) ;
%             pdf_c = getRsDEdistribution( data', data_weights', data_H', [ev_point,1,ev_point] ) ;
%             pdf_c = pdf_c(:,2) ;
%             pdf_ev = evaluatePdfAt( ev_point, centers(:,i),...
%                                      covariances(i,:), weights(:,i) );
%             
%             y_evals = evaluatePdfAt( x_evals, centers(:,i),...
%                                      covariances(i,:), weights(:,i) );
%             y2 = y_evals/normFactor *pdf_c/pdf_ev ;
%            plot(x_evals, y_evals/normFactor *pdf_c/pdf_ev, 'k.-') ; 
%            yy = yy + y2 ;
%         end
%        yy = yy / sum(yy) ;
%         plot(x_evals, yy, 'r-') ;
        
        % show just parzen estimate
        if ( ~isempty(data) )
            pdf = getParzenEstimate( data, [bounds(1), delta, bounds(2)] ,-1, parzen_h, weights_data ) ;
            plot( pdf(:,1), pdf(:,2)/sum(pdf(:,2)),'g') ;        
        end
    case 2
        if ( ~isempty(data) )
           subplot(1,2,1)
           plot(data(1,:),data(2,:),'b*') ; hold on ;
        
        
        for i = 1 : num_centers 
           S = reshape(covariances(i,:), dim, dim ) ; 
           [U,L,V] = svd(S);
           l = diag(L) ;
           phi = acos(V(1,1));
           if V(2,1) < 0
              phi = 2*pi - phi;
           end
           di = sqrt(sum(l))*0.1 ;
           text(centers(1,i)+di,centers(2,i)+di, ...
                num2str(weights(i)),'FontSize', 9, 'BackgroundColor',[.7 .9 .7]) ;
           ellipse(2*sqrt(l(1)),2*sqrt(l(2)),phi,centers(1,i),centers(2,i),'k');
        end
        if ~isempty(prev_centers)
            plot(prev_centers(1,:),prev_centers(2,:),'co') ;
        end
        plot(centers(1,:),centers(2,:),'mo') ;
        
        if ( ~isempty(bounds) )
           axis( [bounds(3),bounds(4), bounds(1), bounds(2)]) ; 
        end
        
        subplot(1,2,2)
        end
        generate3Dplot( centers, weights, covariances ) ;
    otherwise
        warning('The dimension of data is too high to be displayed.') ;
end


function generate3Dplot( centers, weights, covariances )

num_gridPoints = 50 ;
dim = sqrt(cols(covariances)) ;
num_centers = length(weights) ; 
bounds = [] ;
for i = 1 : num_centers
   S = reshape(covariances(i,:), dim, dim ) ; 
   bounds0 = getBound2D( centers(:,i), S ) ; 
   if isempty(bounds)
       bounds = bounds0 ;
   else
       bounds(1) = min([bounds(1), bounds0(1)]) ; 
       bounds(2) = max([bounds(2), bounds0(2)]) ;
       bounds(3) = min([bounds(3), bounds0(3)]) ; 
       bounds(4) = max([bounds(4), bounds0(4)]) ;
   end
end

x = [ bounds(1): (bounds(2)-bounds(1))/num_gridPoints :bounds(2) ] ;
y = [ bounds(3): (bounds(4)-bounds(3))/num_gridPoints :bounds(4) ] ;
[X,Y] = meshgrid(x,y) ;

siz = size(X) ;

X = reshape(X,1,siz(1)*siz(2)) ;
Y = reshape(Y,1,siz(1)*siz(2)) ;
T = [X;Y] ;
Z = zeros(size(X)) ;
for i_center = 1 : cols(centers)
    Covariance = reshape(covariances(i_center,:),dim,dim ) ;
    Precision = inv(Covariance) ;
    D_2 = sqdist(T, centers(:,i_center),Precision)' ;   
    Z = Z + weights(i_center)*( 1/sqrt( (2*pi)^dim * abs(det(Covariance))) ) *exp(-0.5*D_2) ;   
end
X = reshape(X,siz(1),siz(2)) ;
Y = reshape(Y,siz(1),siz(2)) ;
Z = reshape(Z,siz(1),siz(2)) ;
surf(X,Y,Z) ;


function plotUpwardLines( jointCenters, cent_evals ) 
for i = 1 : length(jointCenters)
   plot([jointCenters(i),jointCenters(i)], [0,cent_evals(i)],'k') ;
end

function y_evals = evaluatePdfAt( x_evals, centers, covariances, weights )

num_centers = length(weights) ; 
y_evals = zeros( size(x_evals) ) ;
for i = 1 : length(x_evals)
   y_evals(i) = 0 ;
   for j = 1 : num_centers
      x_tmp = x_evals(i) ; 
      
      
      
      D = (x_tmp - centers(:,j))^2/covariances(j,:) ;
      y_evals(i) = y_evals(i) + weights(j)*(1/( sqrt(2*pi*abs(det(covariances(j,:))))))*exp(-0.5*D) ; 
   end 
end



function bounds = getBound1D( mean, S ) 

x_min = mean(1) - S ; x_max = mean(1) + S ;
bounds = [ x_min, x_max ] ;

function bounds = getBound2D( mean, S ) 

[U,L,V] = svd(S);
l = diag(L) ;
phi = acos(V(1,1));
if V(2,1) < 0
     phi = 2*pi - phi;
end

Dx = abs(2*sqrt(l(1))*sin(phi)) ;
Dy = abs(2*sqrt(l(2))*cos(phi)) ;

x_min = mean(1) - Dx ; x_max = mean(1) + Dx ;
y_min = mean(2) - Dy ; y_max = mean(2) + Dy ;
bounds = [ x_min, x_max, y_min, y_max ] ;



