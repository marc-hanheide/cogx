function visualizeKDE(varargin)

useEdgeColorBlack = 1 ;
deactivateFaceColor = 0 ;
useAlphaWeights = 1 ;
color = 'r' ;
grans = 100 ;
kde = [] ;
data = [] ;
showdata = [] ;
tabulated = [] ;
% process arguments
args = varargin;
nargs = length(args);
for i = 1:2:nargs
    switch args{i}        
        case 'kde', kde = args{i+1} ;
        case 'data', data = args{i+1} ;          
        case 'showdata', showdata = args{i+1} ;       
        case 'tabulated', tabulated = args{i+1} ;
        case 'grans', grans = args{i+1} ;
        case 'showkdecolor', color = args{i+1} ;
        case 'useAlphaWeights', useAlphaWeights = args{i+1} ;
        case 'deactivateFaceColor', deactivateFaceColor = args{i+1} ;
        case 'useEdgeColorBlack', useEdgeColorBlack =  args{i+1}   ;         
    end
end

showTabulated = 1 ;
if size(kde.pdf.Mu,1) ~= 2 || tabulated == 0
    showTabulated = 0 ;    
end   

if showTabulated ~= 1
    drawDistributionGMM( 'pdf',kde.pdf, 'color', color, 'decompose', 1, ...
           'useAlphaWeights', useAlphaWeights, 'deactivateFaceColor', deactivateFaceColor, ...
           'useEdgeColorBlack', useEdgeColorBlack) ;            
else
    %hold off ; plot(kde.pdf.Mu(1,1),kde.pdf.Mu(2,1),'*'); 
    hold off ;
    drawDistributionGMM( 'pdf',kde.pdf, 'color', [1 1 1] ) ;
    boundsIm = axis ; hold off ;    
    visualizePdf2d2( kde.pdf, boundsIm, [], grans ) ;  
    set(gca,'XTickLabel',{}); set(gca,'YTickLabel',{}) ;
    axis equal ; axis tight  ; title('Estimated') ;
end

h = ishold ;
if ~isempty(data) && showdata == 1
    hold on ;
    if size(kde.pdf.Mu,1) == 1
        plot(obsAll(1,:),obsAll(1,:)*0,'.r') ;
    elseif size(kde.pdf.Mu,1) == 2
        plot(obsAll(1,:),obsAll(2,:),'.r') ;
    elseif size(kde.pdf.Mu,1) == 3
        plot3(obsAll(1,:),obsAll(2,:),obsAll(3,:),'.r') ;
    end
    if ( h == 0 ) hold off ; end
end