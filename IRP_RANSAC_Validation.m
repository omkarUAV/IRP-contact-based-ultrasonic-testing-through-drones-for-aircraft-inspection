%% IRP_RANSAC_Validation.m
% Synthetic robustness validation for local plane/normal estimation from the
% central 6x6 ROI of a VL53L5CX 8x8 multizone range frame.
% Full executable MATLAB code; no pseudocode.

clear; clc; close all;
rng(42,'twister');
thisFile = mfilename('fullpath');
if isempty(thisFile); baseDir = pwd; else; baseDir = fileparts(thisFile); end
figDir = fullfile(baseDir,'Figures_300dpi');
dataDir = fullfile(baseDir,'Data');
if ~exist(figDir,'dir'); mkdir(figDir); end
if ~exist(dataDir,'dir'); mkdir(dataDir); end

P.roiSize = 6;
P.gridExtent = 0.12;        % m local width represented in synthetic cloud
P.rangeNoiseSigma = 0.006;  % m synthetic depth noise
P.ransacIterations = 150;
P.inlierThreshold = 0.018;  % m
P.residualScale = 0.012;    % m, confidence normalization
P.tiltDeg = 12;
P.curvature = 1/3;          % 1/m, representative R=3 m curvature
P.repetitions = 30;
outlierCounts = [0 4 7 11 14];

rows = [];
r = 0;
for io = 1:numel(outlierCounts)
    nOut = outlierCounts(io);
    for rep = 1:P.repetitions
        seed = 5000+nOut*100+rep;
        T = runRansacTrial(P,nOut,seed);
        r = r+1;
        rows(r,:) = [nOut,100*nOut/36,rep,T.normalErrorDeg,1000*T.residualRMS, ...
                     T.inlierRatio,T.qR,T.nInliers]; %#ok<SAGROW>
    end
end
Results = array2table(rows,'VariableNames', ...
    {'outliers','outlier_pct','repetition','normal_error_deg','residual_mm', ...
     'inlier_ratio','q_R','n_inliers'});
writetable(Results,fullfile(dataDir,'ransac_outlier_validation.csv'));

% Summary statistics: median and IQR by design choice.
S = table();
for io = 1:numel(outlierCounts)
    nOut = outlierCounts(io);
    mask = Results.outliers==nOut;
    S.outliers(io,1) = nOut;
    S.outlier_pct(io,1) = 100*nOut/36;
    S.normal_error_median_deg(io,1) = median(Results.normal_error_deg(mask));
    S.normal_error_IQR_deg(io,1) = iqr(Results.normal_error_deg(mask));
    S.residual_median_mm(io,1) = median(Results.residual_mm(mask));
    S.inlier_ratio_median(io,1) = median(Results.inlier_ratio(mask));
    S.q_R_median(io,1) = median(Results.q_R(mask));
end
writetable(S,fullfile(dataDir,'ransac_outlier_summary.csv'));

f = figure('Color','w','Position',[100 100 800 500]);
boxchart(categorical(Results.outlier_pct),Results.normal_error_deg);
xlabel('Corrupted ROI pixels [%]'); ylabel('Normal-angle error [deg]');
title('RANSAC normal-estimation robustness, 30 trials per condition'); grid on;
exportgraphics(f,fullfile(figDir,'Fig_RANSAC_normal_error.png'),'Resolution',300); close(f);

f = figure('Color','w','Position',[100 100 800 500]);
plot(S.outlier_pct,S.q_R_median,'-o','LineWidth',1.5); hold on;
yline(0.70,'--','q_{on}=0.70'); yline(0.55,'--','q_{off}=0.55');
xlabel('Corrupted ROI pixels [%]'); ylabel('Median confidence q_R');
title('Confidence degradation with synthetic outliers'); grid on; ylim([0 1]);
exportgraphics(f,fullfile(figDir,'Fig_RANSAC_confidence.png'),'Resolution',300); close(f);

disp(S);

function R = runRansacTrial(P,nOut,seed)
    rng(seed,'twister');
    g = linspace(-P.gridExtent/2,P.gridExtent/2,P.roiSize);
    [X,Y] = meshgrid(g,g);
    x = X(:); y = Y(:);
    a = tand(P.tiltDeg);
    z = 1.0 + a*x + 0.5*P.curvature*(x.^2+y.^2);
    pts = [x y z];
    pts(:,3) = pts(:,3) + P.rangeNoiseSigma*randn(size(pts,1),1);

    if nOut > 0
        idx = randperm(size(pts,1),nOut);
        signs = 2*(rand(nOut,1)>0.5)-1;
        pts(idx,3) = pts(idx,3) + signs.*(0.06+0.12*rand(nOut,1));
    end

    [n,inliers,residual] = ransacPlane(pts,P.ransacIterations,P.inlierThreshold);
    nTrue = [-a 0 1]; nTrue = nTrue/norm(nTrue);
    normalErrorDeg = acosd(max(-1,min(1,dot(n,nTrue))));
    inlierRatio = sum(inliers)/size(pts,1);
    qR = inlierRatio*exp(-(residual/P.residualScale)^2);

    R.normalErrorDeg = normalErrorDeg;
    R.residualRMS = residual;
    R.inlierRatio = inlierRatio;
    R.qR = qR;
    R.nInliers = sum(inliers);
end

function [nBest,inBest,residual] = ransacPlane(P,iters,threshold)
    N = size(P,1);
    bestCount = -1; bestRMS = inf; inBest = false(N,1);
    for i = 1:iters
        idx = randperm(N,3);
        p1=P(idx(1),:); p2=P(idx(2),:); p3=P(idx(3),:);
        n = cross(p2-p1,p3-p1);
        if norm(n)<1e-10; continue; end
        n = n/norm(n); if n(3)<0; n=-n; end
        d = -dot(n,p1);
        dist = abs(P*n'+d);
        in = dist<threshold;
        count = sum(in);
        if count>=3
            rmsVal = sqrt(mean(dist(in).^2));
            if count>bestCount || (count==bestCount && rmsVal<bestRMS)
                bestCount=count; bestRMS=rmsVal; inBest=in;
            end
        end
    end
    Q = P(inBest,:);
    c = mean(Q,1);
    [~,~,V] = svd(Q-c,0);
    nBest = V(:,end)';
    if nBest(3)<0; nBest=-nBest; end
    nBest = nBest/norm(nBest);
    d = -dot(nBest,c);
    dist = abs(P*nBest'+d);
    inBest = dist<threshold;
    residual = sqrt(mean(dist(inBest).^2));
end
