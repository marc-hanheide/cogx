function drawAllClasses( model, hp )

%return;

decompose = 0 ;
colrs = {'r', 'g', 'b', 'y', 'k', 'k', 'm', 'c'} ;
labls = { 'Hu','Sa','In','S1','S2','S3','S4'} ;
%  { 'red', 'green', 'blue', 'yellow', 'black', 'white', 'orange',
%  'pink','compact', 'elongated' } ;
nam = {'Colors', 'Shapes'} ;
%figure(hp) ; %clf ;
for i = 1 : 2   
    ha=subplot(1,2,i,'Parent',hp);set(ha,'nextPlot','add');%hold on ;
    for j = 1 : length(model{i}.class_labels)
        if (i == 1 )
            clr = model{i}.kde_cl{j}.ikdeParams.scale.Mu(1:3) ;
            clr = hsv2rgb(clr'); 
        else
            clr = colrs(j) ;
        end
 
        executeOperatorIKDEClsfr( model{i}, 'showKDE_of_class_index', j, 'showkdecolor', clr, 'decompose', decompose, 'draw_to_these_axes', ha) ;
        fbs=getc(model,i,'Fb');
        xlabel(labls(fbs(1)),'Parent',ha) ;
        if length(fbs) > 1
            ylabel(labls(fbs(2)),'Parent',ha) ;
        end
        if length(fbs) > 2
            zlabel(labls(fbs(3)),'Parent',ha) ;
        end 
    end
    title(nam{i},'Parent',ha)
end


