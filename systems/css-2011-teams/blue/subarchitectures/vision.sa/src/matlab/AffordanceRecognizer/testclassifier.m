[BoxData{1} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_023520');
[BoxData{2} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_032757');
[BoxData{3} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_032922');
[BoxData{4} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_032943');
[BoxData{5} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_033003');
[BoxData{6} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_033024');
[BoxData{7} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_043547');
[BoxData{8} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_074720');
[BoxData{9} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101012_232131');
[BoxData{10} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101012_232153');
[BoxData{11} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_020438');
[BoxData{12} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_020604');
[BoxData{13} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_023602');
[BoxData{14} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_023643');
[BoxData{15} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_023953');
[BoxData{16} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_024118');
[BoxData{17} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_024534');
[BoxData{18} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_024552');
[BoxData{19} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_115134');
[BoxData{20} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_115029');

[CylData{1} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101012_233331');
[CylData{2} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101012_233331');
[CylData{3} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101012_234139');
[CylData{4} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_043132');
[CylData{5} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_043155');
[CylData{6} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_042739');
[CylData{7} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_054256');
[CylData{8} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_054339');
[CylData{9} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101012_234508');
[CylData{10} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101012_234655');
[CylData{11} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_014453');
[CylData{12} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_014512');
[CylData{13} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_021618');
[CylData{14} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_021659');
[CylData{15} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_021927');
[CylData{16} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_021951');
[CylData{17} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_022549');
[CylData{18} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_022630');
[CylData{19} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_074350');
[CylData{20} Foo] = processallfeatures('/home/cogx/svn/current/ul/xdata/snapshot(5)/soifilter20101013_074408');

fprintf('Boxes:\n');
for iBox = 1:length(BoxData)
    [BoxDataResults{iBox} Mask] = Classifier.gtclassify('data', BoxData{iBox}');
    fprintf([BoxDataResults{iBox}.Results.GroundTruthClassification{1} '\n']);
end
fprintf('Mask: ');
fprintf('%f ', Mask);
fprintf('\n\n');


fprintf('Cylinders:\n');
for iCyl = 1:length(CylData)
    [CylDataResults{iCyl} Mask] = Classifier.gtclassify('data', CylData{iCyl}');
    fprintf([CylDataResults{iCyl}.Results.GroundTruthClassification{1} '\n']);
end
fprintf('Mask: ');
fprintf('%f ', Mask);
fprintf('\n\n');

% Classifier.set('feature_selection', 'fuzzy');
% fprintf('\n\nFeature Selection: fuzzy\n\n');
% 
% fprintf('Boxes:\n');
% for iBox = 1:length(BoxData)
%     [BoxDataResults{iBox} Mask] = Classifier.gtclassify('data', BoxData{iBox}');
%     fprintf([BoxDataResults{iBox}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% fprintf('\nCylinders:\n');
% for iCyl = 1:length(CylData)
%     [CylDataResults{iCyl} Mask] = Classifier.gtclassify('data', CylData{iCyl}');
%     fprintf([CylDataResults{iCyl}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% Classifier.set('feature_selection', 'hard', 'feature_selection_max', 1);
% fprintf('\n\nFeature Selection: hard\n\n');
% 
% fprintf('\nBoxes:\n');
% for iBox = 1:length(BoxData)
%     [BoxDataResults{iBox} Mask] = Classifier.gtclassify('data', BoxData{iBox}');
%     fprintf([BoxDataResults{iBox}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% fprintf('\nCylinders:\n');
% for iCyl = 1:length(CylData)
%     [CylDataResults{iCyl} Mask] = Classifier.gtclassify('data', CylData{iCyl}');
%     fprintf([CylDataResults{iCyl}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% 
% Classifier.set('feature_selection', 'nodestats_fuzzy');
% fprintf('\n\nFeature Selection: nodestats_fuzzy\n\n');
% 
% fprintf('\nBoxes:\n');
% for iBox = 1:length(BoxData)
%     [BoxDataResults{iBox} Mask] = Classifier.gtclassify('data', BoxData{iBox}');
%     fprintf([BoxDataResults{iBox}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% fprintf('\nCylinders:\n');
% for iCyl = 1:length(CylData)
%     [CylDataResults{iCyl} Mask] = Classifier.gtclassify('data', CylData{iCyl}');
%     fprintf([CylDataResults{iCyl}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% 
% Classifier.set('feature_selection', 'nodestats_hard', 'feature_selection_max', 1);
% fprintf('\n\nFeature Selection: nodestats_hard\n\n');
% 
% fprintf('\nBoxes:\n');
% for iBox = 1:length(BoxData)
%     [BoxDataResults{iBox} Mask] = Classifier.gtclassify('data', BoxData{iBox}');
%     fprintf([BoxDataResults{iBox}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% fprintf('\nCylinders:\n');
% for iCyl = 1:length(CylData)
%     [CylDataResults{iCyl} Mask] = Classifier.gtclassify('data', CylData{iCyl}');
%     fprintf([CylDataResults{iCyl}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% 
% Classifier.set('feature_selection', 'lda_fuzzy');
% fprintf('\n\nFeature Selection: lda_fuzzy\n\n');
% 
% fprintf('\nBoxes:\n');
% for iBox = 1:length(BoxData)
%     [BoxDataResults{iBox} Mask] = Classifier.gtclassify('data', BoxData{iBox}');
%     fprintf([BoxDataResults{iBox}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% fprintf('\nCylinders:\n');
% for iCyl = 1:length(CylData)
%     [CylDataResults{iCyl} Mask] = Classifier.gtclassify('data', CylData{iCyl}');
%     fprintf([CylDataResults{iCyl}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% 
% Classifier.set('feature_selection', 'lda_hard', 'feature_selection_max', 1);
% fprintf('\n\nFeature Selection: lda_hard\n\n');
% 
% fprintf('\nBoxes:\n');
% for iBox = 1:length(BoxData)
%     [BoxDataResults{iBox} Mask] = Classifier.gtclassify('data', BoxData{iBox}');
%     fprintf([BoxDataResults{iBox}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% fprintf('\nCylinders:\n');
% for iCyl = 1:length(CylData)
%     [CylDataResults{iCyl} Mask] = Classifier.gtclassify('data', CylData{iCyl}');
%     fprintf([CylDataResults{iCyl}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% 
% Classifier.set('feature_selection', 'nodestats_exp');
% fprintf('\n\nFeature Selection: nodestats_exp\n\n');
% 
% fprintf('\nBoxes:\n');
% for iBox = 1:length(BoxData)
%     [BoxDataResults{iBox} Mask] = Classifier.gtclassify('data', BoxData{iBox}');
%     fprintf([BoxDataResults{iBox}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
% 
% fprintf('\nCylinders:\n');
% for iCyl = 1:length(CylData)
%     [CylDataResults{iCyl} Mask] = Classifier.gtclassify('data', CylData{iCyl}');
%     fprintf([CylDataResults{iCyl}.Results.GroundTruthClassification{1} '\n']);
% end
% fprintf('Mask: ');
% fprintf('%f ', Mask);
