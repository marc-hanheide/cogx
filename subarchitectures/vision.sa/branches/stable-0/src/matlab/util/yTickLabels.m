function yTickLabels(labels);

if ischar(labels(1))
   set(gca,'YtickLabel',labels);
   set(gca,'Ytick',1:size(labels,1));
else
   set(gca,'YtickLabel',num2str(labels'));
   set(gca,'Ytick',1:length(labels));
end;   
