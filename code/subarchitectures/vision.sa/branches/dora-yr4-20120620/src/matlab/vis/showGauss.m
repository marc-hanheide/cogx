function showGauss(mu,var,int,ha,col)


if nargin<3 || isempty(int)
   int=[mu-5*var,mu+5*var];
end

if nargin<4 || isempty(ha)
   ha=gca;
end   

if  nargin<5
   col='k';
end;   

intv=int(1):(int(2)-int(1))/100:int(2);

pdf=normpdf(intv,mu,sqrt(var));

pdf(pdf<1e-12)=0;%to overcome a stupid Matlap plot bug
plot(ha,intv,pdf,'Color',col);
set(ha,'Xlim',int);