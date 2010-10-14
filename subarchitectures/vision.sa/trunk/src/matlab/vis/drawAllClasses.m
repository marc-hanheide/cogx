function drawAllClasses( model )

decompose = 0 ;
colrs = {'r', 'g', 'b', 'y', 'k', 'k', 'm', 'c'} ;
labls = { 'Hu','Sa','In','S1','S2','S3','S4'} ;
%  { 'red', 'green', 'blue', 'yellow', 'black', 'white', 'orange',
%  'pink','compact', 'elongated' } ;
nam = {'Colors', 'Shapes'} ;
figure(3) ; clf ;
for i = 1 : 2   
    subplot(1,2,i) ;hold on ;
    for j = 1 : length(model{i}.class_labels)
        if (i == 1 )
            clr = model{i}.kde_cl{j}.ikdeParams.scale.Mu(1:3) ;
            clr = hsv2rgb(clr'); 
        else
            clr = colrs(j) ;
        end
 
        executeOperatorIKDEClsfr( model{i}, 'showKDE_of_class_index', j, 'showkdecolor', clr, 'decompose', decompose) ;
        xlabel(labls(model{i}.sub_selected_features(1))) ;
        if length(model{i}.sub_selected_features) > 1
            ylabel(labls(model{i}.sub_selected_features(2))) ;
        end
        if length(model{i}.sub_selected_features) > 2
            zlabel(labls(model{i}.sub_selected_features(3))) ;
        end 
    end
    title(nam{i})
end


