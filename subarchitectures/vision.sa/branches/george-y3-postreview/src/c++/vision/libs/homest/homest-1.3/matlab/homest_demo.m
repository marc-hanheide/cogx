matchesfile='../test/matches.txt';

% read in matches
[x1, y1, x2, y2]=textread(matchesfile, '%f%f%f%f', 'commentstyle', 'shell');
pts0(:, :)=[x1'; y1']';
pts1(:, :)=[x2'; y2']';
%pts0, pts1

%[H, idxOutliers]=homest(pts0, pts1, 0.7, 1, 'sym_xfer_err', 2);
[H, idxOutliers]=homest(pts0, pts1, 0.7, 1, 'sym_xfer_err');
nbOutliers=max(size(idxOutliers));

H, nbOutliers

% affine H
%[H, idxOutliers]=homest(pts0, pts1, 0.7, 0, 'aff')
