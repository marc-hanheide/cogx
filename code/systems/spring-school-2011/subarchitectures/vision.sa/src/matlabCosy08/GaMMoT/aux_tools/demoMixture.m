N = 300 ;
Y=[-5.0:0.01:5.0]; [actual_density,X] = marron_wand_normal_mixtures(3,Y,N) ;
figure(1); clf; plot(Y,actual_density) ;