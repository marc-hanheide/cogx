function I = sampleDiscrete( P )

c = cumsum(P) ;

I = -1 ;
x = rand(1) ;
for i = 1 : length(P)
    if x <= c(i)
       I = i ;
       break ;
    end
end