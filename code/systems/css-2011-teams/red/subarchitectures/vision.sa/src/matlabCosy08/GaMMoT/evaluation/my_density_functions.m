function [actual_density, X] = my_density_functions(MyDi, N)

if ( MyDi == 1 )
    disp('mixture uniform, normal separated')
    norm.weights = [7/10] ;
    norm.mu = [0.5] ;
    norm.covariances = 0.1^2 ;
    uni.mu = [-0.5] ;
    uni.widths = [1] ;
    uni.weights = [3/10] ;
end

if ( MyDi == 2 )
    disp('mixture normal, normal separated different scales')
    norm.weights = [3/10, 2/10] ;
    norm.mu = [-0.9, 1] ; %0.70
    norm.covariances = [0.15; 0.05].^2 ;
    uni.mu = [0] ;
    uni.widths = [1] ;
    uni.weights = [5/10] ;
end

if ( MyDi == 3 )
    disp('mixture normal, normal, normal separated different scales')
    norm.weights = [3/10, 0.5/10, 2/10] ;
    norm.mu = [-0.9, 0, 1.2] ; %0.70] ;
    norm.covariances = [0.15; 0.05; 0.05].^2 ;
    uni.mu = [0] ;
    uni.widths = [1] ;
    uni.weights = [4.5/10] ;
end

if ( MyDi == 4 )
    disp('uniform')
    norm.weights = [] ;
    norm.mu = [] ;
    norm.covariances = [] ;
    uni.mu = [0] ;
    uni.widths = [1] ;
    uni.weights = [1] ;
end

actual_density.uni = uni ;
actual_density.norm = norm ;

% sample mixture
X = generateSamplesFromNormUniPdf( actual_density, N ) ;



