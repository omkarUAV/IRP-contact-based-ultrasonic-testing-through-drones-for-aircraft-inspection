%% IRP_RANSAC_Surface_Normal_Comparison.m
% RANSAC versus ordinary least-squares plane regression for an IRP:
% Contact-based UAV ultrasonic inspection of curved aircraft skin.
%
% The script simulates a local VL53L5CX-style depth point cloud containing:
%   1. Aircraft-skin curvature
%   2. Sensor Gaussian noise
%   3. Clustered gross range outliers
%
% It compares:
%   A. Ordinary least-squares plane fitting using every measurement
%   B. RANSAC plane fitting followed by least-squares refinement on inliers
%
% No Statistics and Machine Learning Toolbox is required.
% Local functions at the end of scripts require MATLAB R2016b or newer.

clear;
close all;
clc;
rng(11, 'twister');

%% 1. Simulation parameters
surfaceRadius_m       = 3.0;       % Cylindrical aircraft-skin radius
patchHalfWidth_m      = 0.25;      % Local patch extends +/- 0.25 m
gridPointsPerAxis     = 15;        % Synthetic depth grid
surfaceTiltX_deg      = 4.0;       % Local surface slope about y-axis
surfaceTiltY_deg      = -3.0;      % Local surface slope about x-axis
sensorNoiseStd_m      = 0.0025;    % 2.5 mm Gaussian depth noise
outlierFraction       = 0.22;      % Fraction of gross false returns
outlierMinOffset_m    = 0.06;      % Minimum gross range error
outlierMaxOffset_m    = 0.16;      % Maximum gross range error

% RANSAC settings
distanceThreshold_m   = 0.008;     % 8 mm orthogonal inlier threshold
maximumIterations     = 3000;
requestedConfidence   = 0.999;
minimumSampleSize     = 3;         % Three non-collinear points define a plane

%% 2. Generate a curved aircraft-skin patch
axisVector = linspace(-patchHalfWidth_m, patchHalfWidth_m, ...
                      gridPointsPerAxis);
[X, Y] = meshgrid(axisVector, axisVector);

tiltX_rad = deg2rad(surfaceTiltX_deg);
tiltY_rad = deg2rad(surfaceTiltY_deg);

% Cylindrical sag in x plus a known local orientation.
Zcurvature = surfaceRadius_m - sqrt(surfaceRadius_m^2 - X.^2);
Ztrue = Zcurvature ...
      + tan(tiltX_rad).*X ...
      + tan(tiltY_rad).*Y;

% Add normal depth noise.
Zmeasured = Ztrue + sensorNoiseStd_m.*randn(size(Ztrue));

%% 3. Add spatially clustered gross outliers
% Clustered outliers imitate multipath, edge reflections, poor return
% strength, surface reflectivity problems, or pixels seeing another object.
x = X(:);
y = Y(:);
z = Zmeasured(:);
numberOfPoints = numel(z);

numberOfOutliers = round(outlierFraction*numberOfPoints);

% Bias the false returns toward one side. This is more challenging than
% symmetric random outliers, which may cancel during ordinary regression.
candidateIndices = find(x > 0.02);
if numberOfOutliers > numel(candidateIndices)
    error('Not enough candidate points for the requested outlier fraction.');
end

selectedOrder = randperm(numel(candidateIndices), numberOfOutliers);
trueOutlierIndices = candidateIndices(selectedOrder);
trueOutlierMask = false(numberOfPoints, 1);
trueOutlierMask(trueOutlierIndices) = true;

grossOffsets = outlierMinOffset_m ...
             + (outlierMaxOffset_m - outlierMinOffset_m) ...
             .*rand(numberOfOutliers, 1);

z(trueOutlierIndices) = z(trueOutlierIndices) + grossOffsets;

designMatrix = [x, y, ones(numberOfPoints, 1)];

%% 4. Regression without RANSAC: ordinary least squares
% Plane model: z = a*x + b*y + c
betaLeastSquares = designMatrix \ z;
zPlaneLeastSquares = reshape(designMatrix*betaLeastSquares, size(X));

%% 5. Regression with RANSAC
bestInlierMask = false(numberOfPoints, 1);
bestInlierCount = 0;
bestMedianResidual = inf;

for iteration = 1:maximumIterations
    sampleIndices = randperm(numberOfPoints, minimumSampleSize);
    sampleMatrix = designMatrix(sampleIndices, :);

    % Reject nearly collinear/degenerate samples.
    if rcond(sampleMatrix) < 1e-10
        continue;
    end

    candidateBeta = sampleMatrix \ z(sampleIndices);
    candidateResiduals = pointToPlaneResidual( ...
        designMatrix, z, candidateBeta);

    candidateInlierMask = candidateResiduals < distanceThreshold_m;
    candidateInlierCount = sum(candidateInlierMask);

    if candidateInlierCount >= minimumSampleSize
        candidateMedian = median( ...
            candidateResiduals(candidateInlierMask));
    else
        candidateMedian = inf;
    end

    % Primary score: number of inliers.
    % Tie-breaker: lower median inlier residual.
    isBetterModel = candidateInlierCount > bestInlierCount || ...
       (candidateInlierCount == bestInlierCount && ...
        candidateMedian < bestMedianResidual);

    if isBetterModel
        bestInlierMask = candidateInlierMask;
        bestInlierCount = candidateInlierCount;
        bestMedianResidual = candidateMedian;
    end
end

if bestInlierCount < minimumSampleSize
    error('RANSAC failed to identify a valid plane.');
end

% Final least-squares refinement using only RANSAC inliers.
betaRANSAC = designMatrix(bestInlierMask, :) \ z(bestInlierMask);
zPlaneRANSAC = reshape(designMatrix*betaRANSAC, size(X));

%% 6. Quantitative comparison
% True local tangent normal at the centre of the patch.
trueNormal = [-tan(tiltX_rad); -tan(tiltY_rad); 1];
trueNormal = trueNormal/norm(trueNormal);

leastSquaresNormal = planeNormal(betaLeastSquares);
ransacNormal = planeNormal(betaRANSAC);

leastSquaresAngleError_deg = normalAngleError( ...
    leastSquaresNormal, trueNormal);
ransacAngleError_deg = normalAngleError(ransacNormal, trueNormal);

leastSquaresRMSE_mm = 1000*sqrt(mean( ...
    (zPlaneLeastSquares(:) - Ztrue(:)).^2));
ransacRMSE_mm = 1000*sqrt(mean( ...
    (zPlaneRANSAC(:) - Ztrue(:)).^2));

detectedOutlierMask = ~bestInlierMask;
truePositive = sum(detectedOutlierMask & trueOutlierMask);
falsePositive = sum(detectedOutlierMask & ~trueOutlierMask);
falseNegative = sum(~detectedOutlierMask & trueOutlierMask);

outlierPrecision = truePositive/max(truePositive + falsePositive, 1);
outlierRecall = truePositive/max(truePositive + falseNegative, 1);

estimatedInlierRatio = bestInlierCount/numberOfPoints;
if estimatedInlierRatio >= 1
    theoreticallyRequiredIterations = 1;
else
    denominator = log(1 - estimatedInlierRatio^minimumSampleSize);
    theoreticallyRequiredIterations = ceil( ...
        log(1 - requestedConfidence)/denominator);
end

resultTable = table( ...
    ["Ordinary least squares"; "RANSAC + inlier refinement"], ...
    [leastSquaresAngleError_deg; ransacAngleError_deg], ...
    [leastSquaresRMSE_mm; ransacRMSE_mm], ...
    'VariableNames', ...
    {'Method', 'SurfaceNormalError_deg', 'PlaneRMSE_mm'});

disp(' ');
disp('================ RANSAC IRP COMPARISON ================');
disp(resultTable);
fprintf('RANSAC inliers: %d of %d points (%.1f%%)\n', ...
    bestInlierCount, numberOfPoints, 100*estimatedInlierRatio);
fprintf('True gross outliers added: %d\n', numberOfOutliers);
fprintf('Outlier-detection precision: %.3f\n', outlierPrecision);
fprintf('Outlier-detection recall: %.3f\n', outlierRecall);
fprintf(['Estimated minimum iterations for %.2f%% confidence, ' ...
         'using the measured inlier ratio: %d\n'], ...
    100*requestedConfidence, theoreticallyRequiredIterations);
disp('========================================================');

%% 7. Visualisation: raw corrupted depth points
figure('Name', 'IRP simulated depth data', 'Color', 'w');
surfaceHandle = surf(X, Y, Ztrue);
set(surfaceHandle, 'FaceAlpha', 0.45, 'EdgeColor', 'none');
hold on;
scatter3(x(~trueOutlierMask), y(~trueOutlierMask), ...
    z(~trueOutlierMask), 26, 'filled');
scatter3(x(trueOutlierMask), y(trueOutlierMask), ...
    z(trueOutlierMask), 42, 'x', 'LineWidth', 1.5);
grid on;
axis equal;
xlabel('x position (m)');
ylabel('y position (m)');
zlabel('Measured range z (m)');
title('Simulated curved skin and corrupted depth returns');
legend('True curved surface', 'Noisy valid returns', ...
       'Injected gross outliers', 'Location', 'best');
view(42, 24);

%% 8. Visualisation: regression without RANSAC
figure('Name', 'Without RANSAC', 'Color', 'w');
scatter3(x, y, z, 25, 'filled');
hold on;
planeHandle = surf(X, Y, zPlaneLeastSquares);
set(planeHandle, 'FaceAlpha', 0.55, 'EdgeColor', 'none');
grid on;
axis equal;
xlabel('x position (m)');
ylabel('y position (m)');
zlabel('Depth z (m)');
title(sprintf(['Without RANSAC: all points used\n' ...
    'normal error = %.2f deg, RMSE = %.2f mm'], ...
    leastSquaresAngleError_deg, leastSquaresRMSE_mm));
legend('All depth measurements', 'Least-squares plane', ...
       'Location', 'best');
view(42, 24);

%% 9. Visualisation: regression with RANSAC
figure('Name', 'With RANSAC', 'Color', 'w');
scatter3(x(bestInlierMask), y(bestInlierMask), ...
    z(bestInlierMask), 26, 'filled');
hold on;
scatter3(x(~bestInlierMask), y(~bestInlierMask), ...
    z(~bestInlierMask), 42, 'x', 'LineWidth', 1.5);
planeHandle = surf(X, Y, zPlaneRANSAC);
set(planeHandle, 'FaceAlpha', 0.55, 'EdgeColor', 'none');
grid on;
axis equal;
xlabel('x position (m)');
ylabel('y position (m)');
zlabel('Depth z (m)');
title(sprintf(['With RANSAC: rejected points excluded\n' ...
    'normal error = %.2f deg, RMSE = %.2f mm'], ...
    ransacAngleError_deg, ransacRMSE_mm));
legend('RANSAC inliers', 'RANSAC rejected points', ...
       'Refined RANSAC plane', 'Location', 'best');
view(42, 24);

%% 10. Metric figures
figure('Name', 'Surface-normal error comparison', 'Color', 'w');
bar(categorical({'Without RANSAC', 'With RANSAC'}), ...
    [leastSquaresAngleError_deg, ransacAngleError_deg]);
ylabel('Surface-normal angular error (deg)');
title('Geometry-estimation accuracy');
grid on;

figure('Name', 'Plane RMSE comparison', 'Color', 'w');
bar(categorical({'Without RANSAC', 'With RANSAC'}), ...
    [leastSquaresRMSE_mm, ransacRMSE_mm]);
ylabel('RMSE relative to true curved patch (mm)');
title('Local surface-fit error');
grid on;

%% 11. IRP interpretation
fprintf('\nIRP interpretation:\n');
fprintf(['The non-RANSAC estimate is pulled toward the clustered false ' ...
         'depth returns.\n']);
fprintf(['The RANSAC estimate is suitable for calculating a robust local ' ...
         'surface normal.\n']);
fprintf(['That normal can be supplied to the UAV attitude reference, probe ' ...
         'alignment, or geometry-guided force controller.\n']);

%% Local functions
function residuals = pointToPlaneResidual(designMatrix, z, beta)
% Orthogonal point-to-plane distance for z = a*x + b*y + c.
    numerator = abs(designMatrix*beta - z);
    denominator = sqrt(beta(1)^2 + beta(2)^2 + 1);
    residuals = numerator/denominator;
end

function normalVector = planeNormal(beta)
% Normal corresponding to plane z = a*x + b*y + c.
    normalVector = [-beta(1); -beta(2); 1];
    normalVector = normalVector/norm(normalVector);
end

function angleError_deg = normalAngleError(estimatedNormal, trueNormal)
% Absolute dot product makes the comparison independent of normal direction.
    dotValue = abs(dot(estimatedNormal, trueNormal));
    dotValue = min(max(dotValue, -1), 1);
    angleError_deg = acosd(dotValue);
end
