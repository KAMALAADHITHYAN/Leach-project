% LEACH vs TEEN Comparison from Saved Results
% ------------------------------------------
% This script does NOT run simulations. It reads stored outputs generated
% by LEACH and TEEN runs, then plots comparison curves in Figures 1-4.
%
% Expected files (MATLAB .mat in current folder):
%   - leach_results.mat
%   - teen_results.mat
%
% Expected variables in each file:
%   rounds, ALIVE, RESIDUAL_ENERGY, THROUGHPUT_CUM, CLUSTERHS
%   first_dead, half_dead, all_dead

clear;
clc;
close all;

leachFile = 'leach_results.mat';
teenFile  = 'teen_results.mat';

if ~isfile(leachFile)
    error('Missing %s. Run LEACH script and save outputs first.', leachFile);
end
if ~isfile(teenFile)
    error('Missing %s. Run TEEN script and save outputs first.', teenFile);
end

leach = load(leachFile);
teen  = load(teenFile);

required = {'rounds','ALIVE','RESIDUAL_ENERGY','THROUGHPUT_CUM','CLUSTERHS', ...
    'first_dead','half_dead','all_dead'};

for k = 1:numel(required)
    if ~isfield(leach, required{k})
        error('Field "%s" not found in %s.', required{k}, leachFile);
    end
    if ~isfield(teen, required{k})
        error('Field "%s" not found in %s.', required{k}, teenFile);
    end
end

% Use full protocol ranges so TEEN tail (late rounds) is visible even if
% LEACH dies early.
maxRound = max([leach.rounds(end), teen.rounds(end)]);

% 1) Alive Nodes vs Rounds
figure(1);
plot(leach.rounds, leach.ALIVE, 'b', 'LineWidth', 2); hold on;
plot(teen.rounds, teen.ALIVE, 'r--', 'LineWidth', 2);
grid on; box on;
xlabel('Round Number');
ylabel('Number of Alive Nodes');
title('LEACH vs TEEN: Alive Nodes vs Rounds');
legend('LEACH', 'TEEN', 'Location', 'best');
xlim([0 maxRound]);
annotate_lnd(leach.all_dead, teen.all_dead);

% 2) Residual Energy vs Rounds
figure(2);
plot(leach.rounds, leach.RESIDUAL_ENERGY, 'b', 'LineWidth', 2); hold on;
plot(teen.rounds, teen.RESIDUAL_ENERGY, 'r--', 'LineWidth', 2);
grid on; box on;
xlabel('Round Number');
ylabel('Total Residual Energy (J)');
title('LEACH vs TEEN: Residual Energy vs Rounds');
legend('LEACH', 'TEEN', 'Location', 'best');
xlim([0 maxRound]);
annotate_lnd(leach.all_dead, teen.all_dead);

% 3) Throughput vs Rounds (cumulative)
figure(3);
plot(leach.rounds, leach.THROUGHPUT_CUM, 'b', 'LineWidth', 2); hold on;
plot(teen.rounds, teen.THROUGHPUT_CUM, 'r--', 'LineWidth', 2);
grid on; box on;
xlabel('Round Number');
ylabel('Cumulative Throughput (Packets)');
title('LEACH vs TEEN: Throughput vs Rounds');
legend('LEACH', 'TEEN', 'Location', 'best');
xlim([0 maxRound]);
annotate_lnd(leach.all_dead, teen.all_dead);

% 4) CH count per round
figure(4);
plot(leach.rounds, leach.CLUSTERHS, 'b', 'LineWidth', 2); hold on;
plot(teen.rounds, teen.CLUSTERHS, 'r--', 'LineWidth', 2);
grid on; box on;
xlabel('Round Number');
ylabel('Number of Cluster Heads');
title('LEACH vs TEEN: Cluster Heads per Round');
legend('LEACH', 'TEEN', 'Location', 'best');
xlim([0 maxRound]);
annotate_lnd(leach.all_dead, teen.all_dead);

% Console summary
fprintf('\n===== LEACH Milestones =====\n');
fprintf('FND: %g | HND: %g | LND: %g\n', leach.first_dead, leach.half_dead, leach.all_dead);

fprintf('===== TEEN Milestones =====\n');
fprintf('FND: %g | HND: %g | LND: %g\n', teen.first_dead, teen.half_dead, teen.all_dead);

if ~isnan(leach.all_dead) && ~isnan(teen.all_dead)
    fprintf('LND ratio (TEEN/LEACH): %.3f\n', teen.all_dead / max(leach.all_dead, eps));
else
    fprintf('LND ratio unavailable (one protocol did not reach LND in rmax).\n');
end


function annotate_lnd(leachLnd, teenLnd)
% Add vertical milestone markers to make end-of-life differences obvious.
yl = ylim;
if ~isnan(leachLnd)
    line([leachLnd leachLnd], yl, 'Color', [0 0.2 0.9], 'LineStyle', ':', ...
        'LineWidth', 1.2, 'HandleVisibility', 'off');
    text(leachLnd, yl(2) - 0.05 * (yl(2) - yl(1)), 'LEACH LND', ...
        'Color', [0 0.2 0.9], 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
end
if ~isnan(teenLnd)
    line([teenLnd teenLnd], yl, 'Color', [0.85 0 0], 'LineStyle', ':', ...
        'LineWidth', 1.2, 'HandleVisibility', 'off');
    text(teenLnd, yl(2) - 0.12 * (yl(2) - yl(1)), 'TEEN LND', ...
        'Color', [0.85 0 0], 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
end
end
