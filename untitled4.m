%% ================================================================
% ESP32 RANSAC PROCESSING-TIME VALIDATION
% 10 experimental samples
%
% Geometry update rate = 15 Hz
% Geometry period       = 66.67 ms
%
% CSV format required:
%
% Sample,RANSAC_Time_ms
% 1,41.2
% 2,42.1
% ...
% 10,40.8
%
% This is executable MATLAB code.
% No pseudocode is used.
% ================================================================

clear;
clc;
close all;

%% 1. SYSTEM REQUIREMENTS

N_required = 10;

geometryRate_Hz = 15;

deadline_ms = 66.67;

%% 2. SELECT CSV FILE

[fileName,filePath] = uigetfile( ...
    '*.csv', ...
    'Select ESP32 RANSAC Timing CSV');

if isequal(fileName,0)

    error('No CSV file was selected.');

end

fullFileName = fullfile(filePath,fileName);

fprintf('\nSelected file:\n%s\n',fullFileName);

%% 3. READ CSV FILE

data = readtable(fullFileName);

%% 4. CHECK THAT REQUIRED COLUMN EXISTS

requiredColumn = 'RANSAC_Time_ms';

if ~ismember(requiredColumn,data.Properties.VariableNames)

    error(['CSV must contain a column named: ' ...
           requiredColumn]);

end

processingTime = data.RANSAC_Time_ms;

%% 5. REMOVE INVALID VALUES

processingTime = processingTime(:);

processingTime = processingTime( ...
    isfinite(processingTime) & processingTime > 0);

%% 6. REQUIRE EXACTLY 10 SAMPLES

N = length(processingTime);

if N ~= N_required

    error(['Exactly 10 valid processing-time measurements are ' ...
           'required. Current valid samples = %d'],N);

end

fprintf('\nValid measurements = %d\n',N);

%% ================================================================
% 7. STATISTICAL ANALYSIS
% ================================================================

meanTime = mean(processingTime);

medianTime = median(processingTime);

stdTime = std(processingTime);

minTime = min(processingTime);

maxTime = max(processingTime);

%% 8. SORT DATA

sortedTime = sort(processingTime);

%% 9. 95th PERCENTILE

P95Time = prctile(processingTime,95);

%% 10. 99th PERCENTILE

P99Time = prctile(processingTime,99);

%% ================================================================
% 11. TIMING MARGIN
% ================================================================

meanMargin = deadline_ms - meanTime;

P95Margin = deadline_ms - P95Time;

P99Margin = deadline_ms - P99Time;

maxMargin = deadline_ms - maxTime;

%% ================================================================
% 12. TIMING-BUDGET UTILISATION
% ================================================================

meanUtilisation = ...
    (meanTime/deadline_ms)*100;

P95Utilisation = ...
    (P95Time/deadline_ms)*100;

P99Utilisation = ...
    (P99Time/deadline_ms)*100;

maxUtilisation = ...
    (maxTime/deadline_ms)*100;

%% ================================================================
% 13. EFFECTIVE UPDATE RATE
% ================================================================

rateMean_Hz = 1000/meanTime;

rateP95_Hz = 1000/P95Time;

rateP99_Hz = 1000/P99Time;

rateMaximum_Hz = 1000/maxTime;

%% ================================================================
% 14. PASS / FAIL TEST
% ================================================================

meanPass = meanTime <= deadline_ms;

P95Pass = P95Time <= deadline_ms;

P99Pass = P99Time <= deadline_ms;

maxPass = maxTime <= deadline_ms;

%% ================================================================
% 15. DISPLAY RESULTS
% ================================================================

fprintf('\n');
fprintf('====================================================\n');
fprintf(' ESP32 RANSAC PROCESSING-TIME VALIDATION\n');
fprintf('====================================================\n');

fprintf('Number of samples          : %d\n',N);

fprintf('Geometry update rate       : %.2f Hz\n', ...
    geometryRate_Hz);

fprintf('Geometry period/deadline   : %.2f ms\n', ...
    deadline_ms);

fprintf('\n');
fprintf('PROCESSING-TIME RESULTS\n');

fprintf('Mean                       : %.3f ms\n', ...
    meanTime);

fprintf('Median                     : %.3f ms\n', ...
    medianTime);

fprintf('Standard deviation         : %.3f ms\n', ...
    stdTime);

fprintf('Minimum                    : %.3f ms\n', ...
    minTime);

fprintf('Maximum                    : %.3f ms\n', ...
    maxTime);

fprintf('P95                        : %.3f ms\n', ...
    P95Time);

fprintf('P99                        : %.3f ms\n', ...
    P99Time);

%% ================================================================
% 16. DISPLAY TIMING MARGIN
% ================================================================

fprintf('\n');
fprintf('TIMING MARGIN\n');

fprintf('Mean margin                : %.3f ms\n', ...
    meanMargin);

fprintf('P95 margin                 : %.3f ms\n', ...
    P95Margin);

fprintf('P99 margin                 : %.3f ms\n', ...
    P99Margin);

fprintf('Maximum-case margin        : %.3f ms\n', ...
    maxMargin);

%% ================================================================
% 17. DISPLAY UTILISATION
% ================================================================

fprintf('\n');
fprintf('66.67 ms TIMING BUDGET USED\n');

fprintf('Mean                       : %.2f %%\n', ...
    meanUtilisation);

fprintf('P95                        : %.2f %%\n', ...
    P95Utilisation);

fprintf('P99                        : %.2f %%\n', ...
    P99Utilisation);

fprintf('Maximum                    : %.2f %%\n', ...
    maxUtilisation);

%% ================================================================
% 18. DISPLAY EFFECTIVE FREQUENCY
% ================================================================

fprintf('\n');
fprintf('SUPPORTED UPDATE RATE\n');

fprintf('Based on mean              : %.2f Hz\n', ...
    rateMean_Hz);

fprintf('Based on P95               : %.2f Hz\n', ...
    rateP95_Hz);

fprintf('Based on P99               : %.2f Hz\n', ...
    rateP99_Hz);

fprintf('Based on maximum           : %.2f Hz\n', ...
    rateMaximum_Hz);

%% ================================================================
% 19. FINAL PASS / FAIL
% ================================================================

fprintf('\n');
fprintf('====================================================\n');
fprintf(' REAL-TIME VALIDATION RESULT\n');
fprintf('====================================================\n');

if meanPass
    fprintf('Mean <= 66.67 ms       : PASS\n');
else
    fprintf('Mean <= 66.67 ms       : FAIL\n');
end

if P95Pass
    fprintf('P95 <= 66.67 ms        : PASS\n');
else
    fprintf('P95 <= 66.67 ms        : FAIL\n');
end

if P99Pass
    fprintf('P99 <= 66.67 ms        : PASS\n');
else
    fprintf('P99 <= 66.67 ms        : FAIL\n');
end

if maxPass
    fprintf('Maximum <= 66.67 ms    : PASS\n');
else
    fprintf('Maximum <= 66.67 ms    : FAIL\n');
end

%% Main validation decision

if P99Pass

    finalResult = "PASS";

    fprintf('\nFINAL RESULT: PASS\n');

    fprintf(['RANSAC processing satisfies the 15 Hz ' ...
             'geometry-update timing requirement.\n']);

else

    finalResult = "FAIL";

    fprintf('\nFINAL RESULT: FAIL\n');

    fprintf(['RANSAC processing does not satisfy the 15 Hz ' ...
             'geometry-update timing requirement.\n']);

end

%% ================================================================
% 20. FIGURE 1
% PROCESSING TIME FOR ALL 10 RUNS
% ================================================================

figure;

sampleNumber = 1:N;

plot(sampleNumber,processingTime, ...
    '-o', ...
    'LineWidth',1.5, ...
    'MarkerSize',6);

hold on;

yline(deadline_ms,'--', ...
    '15 Hz deadline = 66.67 ms', ...
    'LineWidth',1.5);

xlabel('Experimental run');

ylabel('RANSAC processing time (ms)');

title('ESP32 RANSAC Processing Time');

xticks(1:10);

grid on;

%% ================================================================
% 21. FIGURE 2
% PROCESSING-TIME COMPARISON
% ================================================================

figure;

comparison = [ ...
    meanTime ...
    P95Time ...
    P99Time ...
    maxTime ...
    deadline_ms];

bar(comparison);

xticklabels({ ...
    'Mean', ...
    'P95', ...
    'P99', ...
    'Maximum', ...
    '15 Hz Deadline'});

ylabel('Processing time (ms)');

title('RANSAC Processing Time vs 66.67 ms Deadline');

grid on;

%% ================================================================
% 22. FIGURE 3
% GEOMETRY UPDATE RATE REPRESENTATION
% ================================================================

updateRates = 5:30;

availableTime_ms = 1000 ./ updateRates;

figure;

plot(updateRates, ...
    availableTime_ms, ...
    'LineWidth',2);

hold on;

yline(P99Time,'--', ...
    sprintf('Measured P99 = %.2f ms',P99Time), ...
    'LineWidth',1.5);

xline(geometryRate_Hz,'--', ...
    'Selected geometry rate = 15 Hz', ...
    'LineWidth',1.5);

plot(geometryRate_Hz, ...
    deadline_ms, ...
    'o', ...
    'MarkerSize',9, ...
    'LineWidth',2);

xlabel('Geometry update rate (Hz)');

ylabel('Available processing time (ms)');

title('Geometry Update Rate vs RANSAC Processing Time');

grid on;

%% ================================================================
% 23. SAVE RESULTS
% ================================================================

results = table( ...
    N, ...
    geometryRate_Hz, ...
    deadline_ms, ...
    meanTime, ...
    medianTime, ...
    stdTime, ...
    minTime, ...
    maxTime, ...
    P95Time, ...
    P99Time, ...
    meanMargin, ...
    P95Margin, ...
    P99Margin, ...
    meanUtilisation, ...
    P95Utilisation, ...
    P99Utilisation, ...
    rateP99_Hz, ...
    finalResult, ...
    'VariableNames',{ ...
    'Samples', ...
    'GeometryRate_Hz', ...
    'Deadline_ms', ...
    'Mean_ms', ...
    'Median_ms', ...
    'StandardDeviation_ms', ...
    'Minimum_ms', ...
    'Maximum_ms', ...
    'P95_ms', ...
    'P99_ms', ...
    'MeanMargin_ms', ...
    'P95Margin_ms', ...
    'P99Margin_ms', ...
    'MeanUtilisation_percent', ...
    'P95Utilisation_percent', ...
    'P99Utilisation_percent', ...
    'SupportedRateFromP99_Hz', ...
    'ValidationResult'});

outputFile = fullfile( ...
    filePath, ...
    'RANSAC_10_Sample_Validation.csv');

writetable(results,outputFile);

fprintf('\nResults saved to:\n%s\n',outputFile);

disp(results);