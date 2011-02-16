% clear all;
% close all;
% clear functions;
% clc;

newPath = '..\optimalIncrementalKDE\' ;
rmpath(newPath) ; addpath(newPath) ;

disp('---------------------------------------------');
disp(sprintf('Example to demonstrate the use of fast optimal bandwidth selection'));
disp('---------------------------------------------');

% sample normal mixture density of Marron and Wand [ can vary form 1-15]

density_number=2;

%number of datapoints

N=1000;

disp('---------------------------------------------');
disp(sprintf('Sampling %d points from a sample normal mixture density %d of Marron and Wand',N,density_number));
disp('---------------------------------------------');
% 
Y=[-5.0:0.01:5.0];
[actual_density, X] = marron_wand_normal_mixtures(density_number, Y, N);

% X = load('rand_nums.txt') ; X = X(1,1:100) ; N = length(X)
% N = length(X) ; actual_density = actual_density* 0 +1;

disp('---------------------------------------------');
disp(sprintf('Estimating the AMISE optimal bandwidth-SLOW method'));
disp('---------------------------------------------');

to=clock;    
[h_slow]=slow_univariate_bandwidth_estimate_STEPI(N,X);
slow_time=etime(clock,to);

disp(sprintf('h_optimal=%f Time taken=%f seconds',h_slow,slow_time));

epsil=1e-3;

disp('---------------------------------------------');
disp(sprintf('Estimating the AMISE optimal bandwidth-Fast method with epsilon=%e',epsil));
disp('---------------------------------------------');

to=clock;    
[h_fast]=fast_univariate_bandwidth_estimate_STEPI(N,X,epsil);
fast_time=etime(clock,to);




disp(sprintf('h_optimal=%f Time taken=%f seconds',h_fast,fast_time));

rel_error_per=abs((h_slow-h_fast)/h_slow)*100;
speedup=slow_time/fast_time;
 
disp('---------------------------------------------');
disp(sprintf('Speedup=%f Relative percentage error=%d %% ',speedup,rel_error_per));
disp('---------------------------------------------');

disp('---------------------------------------------');
disp(sprintf('Plotting the KDE'));
disp('---------------------------------------------');

min_x=min(X);
min_y=min(Y);
shift=min(min_x, min_y); 
X_shifted=X-shift;
Y_shifted=Y-shift;
max_x=max(X_shifted);
max_y=max(Y_shifted);
scale=1/max(max_x,max_y); 
X_shifted_scaled=X_shifted*scale;
Y_shifted_scaled=Y_shifted*scale;

M=length(Y);
epsil=1e-6;
[kde]=FastUnivariateDensityDerivative(N,M,X_shifted_scaled,Y_shifted_scaled,h_fast*scale,0,epsil);
 
disp(' ')
figure(1); clf ;
plot(Y,actual_density,'k');
hold on;
plot(Y,scale*kde,'r--');
plot(X,0,'k+');
legend('Actual','KDE with optimal h');