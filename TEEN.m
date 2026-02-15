% TEEN over LEACH-Style Clustering (Homogeneous WSN)
% --------------------------------------------------
% Implements LEACH-style setup + TEEN reactive steady-state behavior per round:
% 1) CH election using LEACH threshold T(n)
% 2) Non-CH nodes associate to nearest CH
% 3) Steady-state data transmission:
%    - Member -> CH (CH spends ERX+EDA)
%    - CH -> BS (aggregated packet)

clear;
clc;
close all;

%% ----------------------- PARAMETERS ----------------------- %%
rng(1); % Reproducibility

% Field dimensions (m)
xm = 100;
ym = 100;

% Sink position (center)
sink.x = 0.5 * xm;
sink.y = 0.5 * ym;

% Network parameters
n = 100;          % number of nodes
p = 0.1;          % optimal CH probability
rmax = 5000;      % simulation rounds (set high enough to capture all-dead event)
packetLength = 4000; % bits

% TEEN thresholds (reactive transmission)
% Keep core network/radio constants identical to LEACH baseline for fair comparison.
% Threshold values taken from the repository's LEACH+TEEN performance paper table.
HT = 100;         % hard threshold
ST = 2;           % soft threshold

% TEEN sensing model based on original TEEN simulation description:
% - 100x100 field split into 4 quadrants
% - each quadrant gets random temperature in [0, 200]
% - temperature map refreshes every 5 rounds (paper mentions 5 seconds)
quadUpdateRounds = 5;
quadTempMin = 0;
quadTempMax = 200;

% Local sensing variation (keeps regional temperature model while avoiding
% unrealistically identical readings for all nodes in a quadrant)
nodeOffsetMax = 2;
sensorNoiseSigma = 2;

% Radio energy model (J)
Eo  = 0.5;                % initial energy/node
ETX = 50e-9;              % transmitter electronics (J/bit)
ERX = 50e-9;              % receiver electronics (J/bit)
Efs = 10e-12;             % free-space amplifier (J/bit/m^2)
Emp = 0.0013e-12;         % multipath amplifier (J/bit/m^4)
EDA = 5e-9;               % data aggregation (J/bit)

do = sqrt(Efs / Emp);     % threshold distance
epoch = round(1 / p);     % LEACH epoch length

% TEEN state reset cadence
% Use a multi-round cluster period (not per-round) so HT/ST state is retained,
% but refresh often enough to avoid unrealistically sparse reporting.
clusterResetInterval = max(1, quadUpdateRounds);

%% ----------------------- NODE DEPLOYMENT ----------------------- %%
S = struct([]);
for i = 1:n
    S(i).xd = rand * xm;
    S(i).yd = rand * ym;
    S(i).E = Eo;
    S(i).G = 0;           % rounds left before CH eligibility
    S(i).type = 'N';
    S(i).CH = 0;          % associated cluster index (0 means direct BS fallback)

    % TEEN per-node reactive reporting state
    S(i).SV = NaN;        % last reported sensed value
    S(i).lastSense = NaN; % latest sensed value (for diagnostics)
    S(i).sentInPeriod = false;
    S(i).tempOffset = (2 * rand - 1) * nodeOffsetMax;
end

%% ----------------------- PREALLOCATION ----------------------- %%
rounds = 0:rmax;

DEAD = zeros(1, rmax + 1);
ALIVE = zeros(1, rmax + 1);
CLUSTERHS = zeros(1, rmax + 1);
RESIDUAL_ENERGY = zeros(1, rmax + 1);

% Per-round packets
PACKETS_TO_CH = zeros(1, rmax + 1); % member -> CH packets
PACKETS_TO_BS = zeros(1, rmax + 1); % CH -> BS packets
THROUGHPUT_ROUND = zeros(1, rmax + 1); % total delivered packets per round

% Cumulative packets (research-paper-friendly)
PACKETS_TO_CH_CUM = zeros(1, rmax + 1);
PACKETS_TO_BS_CUM = zeros(1, rmax + 1);
THROUGHPUT_CUM = zeros(1, rmax + 1);

first_dead = NaN;
half_dead = NaN;
all_dead = NaN;

% Snapshot containers for topology visualizations
snapInitial = struct('captured', false, 'round', NaN, 'alive', NaN, 'dead', NaN, ...
    'isCH', false(1, n), 'assocCH', zeros(1, n), 'deadMask', false(1, n));
snapFirstDead = snapInitial;
snapHalfDead = snapInitial;
snapAllDead = snapInitial;

% Quadrant temperatures for TEEN sensing (Q1: x<=xm/2,y>ym/2; Q2: x>xm/2,y>ym/2;
% Q3: x<=xm/2,y<=ym/2; Q4: x>xm/2,y<=ym/2)
quadTemp = quadTempMin + (quadTempMax - quadTempMin) * rand(1, 4);

%% ----------------------- MAIN TEEN LOOP ----------------------- %%
C = struct([]);
cluster = 0;
setupRound = 0;
for r = 0:rmax
    % Update quadrant temperatures every configured interval
    if mod(r, quadUpdateRounds) == 0
        quadTemp = quadTempMin + (quadTempMax - quadTempMin) * rand(1, 4);
    end

    % TEEN cluster-change time: re-cluster and rebroadcast thresholds.
    doClusterChange = (mod(r, clusterResetInterval) == 0) || (r == 0) || (cluster == 0);
    if doClusterChange
        for i = 1:n
            if S(i).E > 0
                S(i).SV = NaN;
                S(i).sentInPeriod = false;
                S(i).type = 'N';
                S(i).CH = 0;
            end
        end

        % ----------------- Setup Phase: CH Election -----------------
        C = struct([]);
        cluster = 0;

        % LEACH epoch progression should be tied to setup-phase count,
        % not raw simulation rounds, when TEEN keeps clusters for periods.
        if mod(setupRound, epoch) == 0
            for i = 1:n
                if S(i).E > 0
                    S(i).G = 0;
                end
            end
        end

        denom = 1 - p * mod(setupRound, epoch);
        if denom <= 0
            threshold = 1; % safeguard for numerical edge case
        else
            threshold = p / denom;
        end

        for i = 1:n
            if S(i).E > 0 && S(i).G <= 0
                if rand <= threshold
                    cluster = cluster + 1;
                    S(i).type = 'C';
                    S(i).G = epoch - 1;

                    C(cluster).id = i;
                    C(cluster).xd = S(i).xd;
                    C(cluster).yd = S(i).yd;
                    C(cluster).members = 0;
                end
            end
        end

        % Countdown G for alive non-eligible nodes
        for i = 1:n
            if S(i).E > 0 && S(i).G > 0 && S(i).type ~= 'C'
                S(i).G = S(i).G - 1;
            end
        end

        setupRound = setupRound + 1;

        % -------- Setup Phase: Cluster Formation (association) --------
        for i = 1:n
            if S(i).E > 0 && S(i).type == 'N'
                if cluster > 0
                    min_dis = inf;
                    chosenCH = 0;
                    for c = 1:cluster
                        d = hypot(S(i).xd - C(c).xd, S(i).yd - C(c).yd);
                        if d < min_dis
                            min_dis = d;
                            chosenCH = c;
                        end
                    end
                    S(i).CH = chosenCH;
                else
                    S(i).CH = 0;
                end
            end
        end
    else
        % Between cluster-change times, keep cluster memberships, but
        % re-associate nodes if their CH died.
        for i = 1:n
            if S(i).E > 0 && S(i).type == 'N'
                needsReassoc = (S(i).CH == 0) || (S(i).CH > cluster);
                if ~needsReassoc && cluster > 0
                    oldChId = C(S(i).CH).id;
                    if S(oldChId).E <= 0
                        needsReassoc = true;
                    end
                end

                if needsReassoc
                    min_dis = inf;
                    chosenCH = 0;
                    for c = 1:cluster
                        chId = C(c).id;
                        if S(chId).E > 0
                            d = hypot(S(i).xd - C(c).xd, S(i).yd - C(c).yd);
                            if d < min_dis
                                min_dis = d;
                                chosenCH = c;
                            end
                        end
                    end
                    S(i).CH = chosenCH;
                end
            end
        end
    end

    packets_to_ch_round = 0;
    packets_to_bs_round = 0;

    for c = 1:cluster
        C(c).members = 0;
    end

    % Capture initial setup snapshot right after initial CH formation/association
    if r == 0 && ~snapInitial.captured
        snapInitial.captured = true;
        snapInitial.round = r;
        snapInitial.alive = sum([S.E] > 0);
        snapInitial.dead = n - snapInitial.alive;
        snapInitial.deadMask = [S.E] <= 0;

        if cluster > 0
            chIds = [C.id];
        else
            chIds = [];
        end
        for i = 1:n
            if ~isempty(chIds) && any(chIds == i)
                snapInitial.isCH(i) = true;
            end
            if S(i).E > 0 && S(i).CH > 0 && S(i).CH <= cluster
                snapInitial.assocCH(i) = C(S(i).CH).id;
            end
        end
    end

    % -------- Steady State Phase: Data Transmission --------
    % TEEN reactive Member -> CH transmission + CH receive/aggregate
    for i = 1:n
        if S(i).E > 0 && S(i).type == 'N'
            % TEEN sensing value from quadrant temperature map
            if S(i).xd <= xm / 2 && S(i).yd > ym / 2
                qIdx = 1;
            elseif S(i).xd > xm / 2 && S(i).yd > ym / 2
                qIdx = 2;
            elseif S(i).xd <= xm / 2 && S(i).yd <= ym / 2
                qIdx = 3;
            else
                qIdx = 4;
            end
            sensedValue = quadTemp(qIdx) + S(i).tempOffset + sensorNoiseSigma * randn;
            sensedValue = max(quadTempMin, min(quadTempMax, sensedValue));
            S(i).lastSense = sensedValue;

            % TEEN trigger condition
            triggerTx = false;
            if sensedValue >= HT
                if ~S(i).sentInPeriod || isnan(S(i).SV)
                    triggerTx = true;
                elseif abs(sensedValue - S(i).SV) >= ST
                    triggerTx = true;
                end
            end

            if ~triggerTx
                continue;
            end

            if S(i).CH > 0
                c = S(i).CH;
                d2ch = hypot(S(i).xd - C(c).xd, S(i).yd - C(c).yd);

                if d2ch > do
                    txCost = ETX * packetLength + Emp * packetLength * (d2ch^4);
                else
                    txCost = ETX * packetLength + Efs * packetLength * (d2ch^2);
                end

                if S(i).E >= txCost
                    S(i).E = S(i).E - txCost;
                    packets_to_ch_round = packets_to_ch_round + 1;
                    S(i).SV = sensedValue;
                    S(i).sentInPeriod = true;

                    chId = C(c).id;
                    if S(chId).E > 0
                        rxAggCost = (ERX + EDA) * packetLength;
                        S(chId).E = S(chId).E - rxAggCost;
                        C(c).members = C(c).members + 1;
                    end
                else
                    S(i).E = 0;
                end
            else
                % Strict fallback for round with no CH: node sends directly to BS
                d2bs = hypot(S(i).xd - sink.x, S(i).yd - sink.y);
                if d2bs > do
                    txBS = ETX * packetLength + Emp * packetLength * (d2bs^4);
                else
                    txBS = ETX * packetLength + Efs * packetLength * (d2bs^2);
                end

                if S(i).E >= txBS
                    S(i).E = S(i).E - txBS;
                    packets_to_bs_round = packets_to_bs_round + 1;
                    S(i).SV = sensedValue;
                    S(i).sentInPeriod = true;
                else
                    S(i).E = 0;
                end
            end
        end
    end

    % CH sensing + CH -> BS transmission
    % Standard TEEN behavior proxy:
    % - CH forwards aggregated data if it received member reports.
    % - CH may also report its own sensed event using HT/ST trigger.
    for c = 1:cluster
        chId = C(c).id;
        if S(chId).E > 0
            % CH local sensing from same regional model
            if S(chId).xd <= xm / 2 && S(chId).yd > ym / 2
                qIdxCH = 1;
            elseif S(chId).xd > xm / 2 && S(chId).yd > ym / 2
                qIdxCH = 2;
            elseif S(chId).xd <= xm / 2 && S(chId).yd <= ym / 2
                qIdxCH = 3;
            else
                qIdxCH = 4;
            end
            sensedCH = quadTemp(qIdxCH) + S(chId).tempOffset + sensorNoiseSigma * randn;
            sensedCH = max(quadTempMin, min(quadTempMax, sensedCH));
            S(chId).lastSense = sensedCH;

            triggerCH = false;
            if sensedCH >= HT
                if ~S(chId).sentInPeriod || isnan(S(chId).SV)
                    triggerCH = true;
                elseif abs(sensedCH - S(chId).SV) >= ST
                    triggerCH = true;
                end
            end

            shouldTxBS = (C(c).members > 0) || triggerCH;
            if shouldTxBS
                d2bs = hypot(S(chId).xd - sink.x, S(chId).yd - sink.y);
                if d2bs > do
                    chTxCost = (ETX + EDA) * packetLength + Emp * packetLength * (d2bs^4);
                else
                    chTxCost = (ETX + EDA) * packetLength + Efs * packetLength * (d2bs^2);
                end

                if S(chId).E >= chTxCost
                    S(chId).E = S(chId).E - chTxCost;
                    packets_to_bs_round = packets_to_bs_round + 1;
                    if triggerCH
                        S(chId).SV = sensedCH;
                        S(chId).sentInPeriod = true;
                    end
                else
                    S(chId).E = 0;
                end
            end
        end
    end

    % Clamp energy to non-negative
    for i = 1:n
        if S(i).E < 0
            S(i).E = 0;
        end
    end

    % ----------------- Statistics -----------------
    energies = [S.E];
    dead = sum(energies <= 0);
    alive = n - dead;

    DEAD(r + 1) = dead;
    ALIVE(r + 1) = alive;
    CLUSTERHS(r + 1) = cluster;
    RESIDUAL_ENERGY(r + 1) = sum(energies);

    PACKETS_TO_CH(r + 1) = packets_to_ch_round;
    PACKETS_TO_BS(r + 1) = packets_to_bs_round;
    THROUGHPUT_ROUND(r + 1) = packets_to_ch_round + packets_to_bs_round;

    if r == 0
        PACKETS_TO_CH_CUM(r + 1) = PACKETS_TO_CH(r + 1);
        PACKETS_TO_BS_CUM(r + 1) = PACKETS_TO_BS(r + 1);
        THROUGHPUT_CUM(r + 1) = THROUGHPUT_ROUND(r + 1);
    else
        PACKETS_TO_CH_CUM(r + 1) = PACKETS_TO_CH_CUM(r) + PACKETS_TO_CH(r + 1);
        PACKETS_TO_BS_CUM(r + 1) = PACKETS_TO_BS_CUM(r) + PACKETS_TO_BS(r + 1);
        THROUGHPUT_CUM(r + 1) = THROUGHPUT_CUM(r) + THROUGHPUT_ROUND(r + 1);
    end

    if isnan(first_dead) && dead >= 1
        first_dead = r;
        if ~snapFirstDead.captured
            snapFirstDead.captured = true;
            snapFirstDead.round = r;
            snapFirstDead.alive = alive;
            snapFirstDead.dead = dead;
            snapFirstDead.deadMask = [S.E] <= 0;

            if cluster > 0
                chIds = [C.id];
            else
                chIds = [];
            end
            for i = 1:n
                if ~isempty(chIds) && any(chIds == i)
                    snapFirstDead.isCH(i) = true;
                end
                if S(i).E > 0 && S(i).CH > 0 && S(i).CH <= cluster
                    snapFirstDead.assocCH(i) = C(S(i).CH).id;
                end
            end
        end
    end
    if isnan(half_dead) && dead >= ceil(n / 2)
        half_dead = r;
        if ~snapHalfDead.captured
            snapHalfDead.captured = true;
            snapHalfDead.round = r;
            snapHalfDead.alive = alive;
            snapHalfDead.dead = dead;
            snapHalfDead.deadMask = [S.E] <= 0;

            if cluster > 0
                chIds = [C.id];
            else
                chIds = [];
            end
            for i = 1:n
                if ~isempty(chIds) && any(chIds == i)
                    snapHalfDead.isCH(i) = true;
                end
                if S(i).E > 0 && S(i).CH > 0 && S(i).CH <= cluster
                    snapHalfDead.assocCH(i) = C(S(i).CH).id;
                end
            end
        end
    end
    if isnan(all_dead) && dead == n
        all_dead = r;
        if ~snapAllDead.captured
            snapAllDead.captured = true;
            snapAllDead.round = r;
            snapAllDead.alive = alive;
            snapAllDead.dead = dead;
            snapAllDead.deadMask = [S.E] <= 0;

            if cluster > 0
                chIds = [C.id];
            else
                chIds = [];
            end
            for i = 1:n
                if ~isempty(chIds) && any(chIds == i)
                    snapAllDead.isCH(i) = true;
                end
                if S(i).E > 0 && S(i).CH > 0 && S(i).CH <= cluster
                    snapAllDead.assocCH(i) = C(S(i).CH).id;
                end
            end
        end
    end

    if dead == n
        % All nodes dead; keep the rest of the arrays constant automatically (already preallocated)
        break;
    end
end

% Use only simulated rounds for plotting/reporting (prevents trailing zeros after early stop)
lastRound = r;
idx = 1:(lastRound + 1);
rounds_plot = rounds(idx);

%% ----------------------- PLOTS ----------------------- %%
% Keep classic/default styling for Figures 1-4

% 1) Alive Nodes vs Rounds
figure(1);
plot(rounds_plot, ALIVE(idx), 'b', 'LineWidth', 2);
grid on; box on;
xlabel('Round Number');
ylabel('Number of Alive Nodes');
title('TEEN: Alive Nodes vs Rounds');
xlim([0 lastRound]);
ylim([0 n]);

% 2) Residual Energy vs Rounds
figure(2);
plot(rounds_plot, RESIDUAL_ENERGY(idx), 'r', 'LineWidth', 2);
grid on; box on;
xlabel('Round Number');
ylabel('Total Residual Energy (J)');
title('TEEN: Residual Energy vs Rounds');
xlim([0 lastRound]);
ylim([0 n * Eo]);

% 3) Throughput vs Rounds (cumulative for publication clarity)
figure(3);
plot(rounds_plot, THROUGHPUT_CUM(idx), 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
grid on; box on;
xlabel('Round Number');
ylabel('Cumulative Throughput (Packets)');
title('TEEN: Throughput vs Rounds');
xlim([0 lastRound]);

% 4) CH count per round
figure(4);
plot(rounds_plot, CLUSTERHS(idx), 'm', 'LineWidth', 1.8);
grid on; box on;
xlabel('Round Number');
ylabel('Number of Cluster Heads');
title('TEEN: Cluster Heads per Round');
xlim([0 lastRound]);

% Dark-theme style for network-topology visuals only (Figures 5 & 6)
figBgColor = [0.06 0.06 0.08];
axisBgColor = [0.05 0.05 0.07];
axisFontColor = [0.92 0.92 0.92];
gridColor = [0.70 0.70 0.70];

% 5) Topology snapshots (first two milestones) with more room
snapshotSetA = {snapInitial, snapFirstDead};
snapshotTitlesA = {'Initial CH Formation', 'First Node Death (FND)'};
figure(5); clf;
set(gcf, 'Color', figBgColor, 'Position', [60 70 1600 700]);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'loose');

for sIdx = 1:2
    nexttile;
    snap = snapshotSetA{sIdx};
    ax = gca;
    set(ax, 'Color', axisBgColor, 'XColor', axisFontColor, 'YColor', axisFontColor, ...
        'GridColor', gridColor, 'GridAlpha', 0.22, 'FontWeight', 'bold', 'FontSize', 13);
    hold on; grid on; box on;

    hAlive = plot(nan, nan, '.', 'Color', [0.25 0.82 1.00], 'MarkerSize', 16, 'DisplayName', 'Alive Node');
    hDead = plot(nan, nan, 'x', 'Color', [1.00 0.35 0.35], 'MarkerSize', 7, 'LineWidth', 1.0, 'DisplayName', 'Dead Node');
    hCH = plot(nan, nan, 'p', 'Color', [1.00 0.90 0.30], 'MarkerFaceColor', [1.00 0.90 0.30], ...
        'MarkerSize', 10, 'LineWidth', 0.8, 'DisplayName', 'Cluster Head');
    hBS = plot(nan, nan, 's', 'Color', [0.40 1.00 0.45], 'MarkerFaceColor', [0.40 1.00 0.45], ...
        'MarkerSize', 9, 'DisplayName', 'Base Station (BS)');
    hCHBS = plot(nan, nan, '-', 'Color', [1.00 0.55 0.10], 'LineWidth', 1.2, 'DisplayName', 'CH -> BS');

    if snap.captured
        deadNodes = snap.deadMask;
        aliveNodes = ~deadNodes;
        aliveNonCH = aliveNodes & ~snap.isCH;

        % Draw member-to-CH links
        for i = 1:n
            if snap.assocCH(i) > 0 && ~snap.isCH(i)
                chNode = snap.assocCH(i);
                plot([S(i).xd, S(chNode).xd], [S(i).yd, S(chNode).yd], '-', ...
                    'Color', [0.45 0.45 0.50], 'LineWidth', 0.35);
            end
        end

        % Draw CH-to-BS links
        chNodeIds = find(snap.isCH);
        for chIdx = 1:numel(chNodeIds)
            chNode = chNodeIds(chIdx);
            plot([S(chNode).xd, sink.x], [S(chNode).yd, sink.y], '-', ...
                'Color', [1.00 0.55 0.10], 'LineWidth', 0.9);
        end

        % Draw cluster boundaries around formed clusters (circle around CH + members)
        for chIdx = 1:numel(chNodeIds)
            chNode = chNodeIds(chIdx);
            memberIdx = find((snap.assocCH == chNode) & aliveNodes);
            memberX = [S(memberIdx).xd, S(chNode).xd];
            memberY = [S(memberIdx).yd, S(chNode).yd];
            distances = hypot(memberX - S(chNode).xd, memberY - S(chNode).yd);
            radius = max([2, distances]);
            rectangle('Position', [S(chNode).xd - radius, S(chNode).yd - radius, 2 * radius, 2 * radius], ...
                'Curvature', [1 1], 'EdgeColor', [0.60 0.60 0.70], 'LineStyle', '--', 'LineWidth', 0.7);
        end

        plot([S(aliveNonCH).xd], [S(aliveNonCH).yd], '.', 'Color', [0.25 0.82 1.00], 'MarkerSize', 16);
        plot([S(deadNodes).xd], [S(deadNodes).yd], 'x', 'Color', [1.00 0.35 0.35], 'MarkerSize', 7, 'LineWidth', 1.0);
        plot([S(snap.isCH).xd], [S(snap.isCH).yd], 'p', 'Color', [1.00 0.90 0.30], ...
            'MarkerFaceColor', [1.00 0.90 0.30], 'MarkerSize', 10, 'LineWidth', 0.8);

        title(sprintf('%s - Round %d', snapshotTitlesA{sIdx}, snap.round), 'Color', axisFontColor, 'FontWeight', 'bold');
        text(2, ym - 4, sprintf('Alive: %d | Dead: %d', snap.alive, snap.dead), ...
            'Color', axisFontColor, 'FontSize', 11, 'FontWeight', 'bold');
    else
        title(sprintf('%s - Not Reached', snapshotTitlesA{sIdx}), 'Color', axisFontColor, 'FontWeight', 'bold');
        text(4, ym / 2, 'Milestone not reached in current rmax', ...
            'Color', axisFontColor, 'FontSize', 10, 'FontWeight', 'bold');
    end

    plot(sink.x, sink.y, 's', 'Color', [0.40 1.00 0.45], ...
        'MarkerFaceColor', [0.40 1.00 0.45], 'MarkerSize', 9);
    text(sink.x + 1.5, sink.y + 1.5, 'BS', 'Color', axisFontColor, 'FontSize', 10, 'FontWeight', 'bold');

    legend([hAlive, hDead, hCH, hBS, hCHBS], 'Location', 'southoutside', 'Orientation', 'horizontal', ...
        'TextColor', axisFontColor, 'Color', axisBgColor, 'EdgeColor', [0.35 0.35 0.35], 'FontSize', 10);
    xlim([0 xm]);
    ylim([0 ym]);
    xlabel('X (m)', 'Color', axisFontColor, 'FontWeight', 'bold');
    ylabel('Y (m)', 'Color', axisFontColor, 'FontWeight', 'bold');
    hold off;
end

% 6) Topology snapshots (last two milestones) moved to next figure for spacing
snapshotSetB = {snapHalfDead, snapAllDead};
snapshotTitlesB = {'Half Node Death (HND)', 'Last Node Death (LND)'};
figure(6); clf;
set(gcf, 'Color', figBgColor, 'Position', [80 90 1600 700]);
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'loose');

for sIdx = 1:2
    nexttile;
    snap = snapshotSetB{sIdx};
    ax = gca;
    set(ax, 'Color', axisBgColor, 'XColor', axisFontColor, 'YColor', axisFontColor, ...
        'GridColor', gridColor, 'GridAlpha', 0.22, 'FontWeight', 'bold', 'FontSize', 13);
    hold on; grid on; box on;

    hAlive = plot(nan, nan, '.', 'Color', [0.25 0.82 1.00], 'MarkerSize', 16, 'DisplayName', 'Alive Node');
    hDead = plot(nan, nan, 'x', 'Color', [1.00 0.35 0.35], 'MarkerSize', 7, 'LineWidth', 1.0, 'DisplayName', 'Dead Node');
    hCH = plot(nan, nan, 'p', 'Color', [1.00 0.90 0.30], 'MarkerFaceColor', [1.00 0.90 0.30], ...
        'MarkerSize', 10, 'LineWidth', 0.8, 'DisplayName', 'Cluster Head');
    hBS = plot(nan, nan, 's', 'Color', [0.40 1.00 0.45], 'MarkerFaceColor', [0.40 1.00 0.45], ...
        'MarkerSize', 9, 'DisplayName', 'Base Station (BS)');
    hCHBS = plot(nan, nan, '-', 'Color', [1.00 0.55 0.10], 'LineWidth', 1.2, 'DisplayName', 'CH -> BS');

    if snap.captured
        deadNodes = snap.deadMask;
        aliveNodes = ~deadNodes;
        aliveNonCH = aliveNodes & ~snap.isCH;

        % Draw member-to-CH links
        for i = 1:n
            if snap.assocCH(i) > 0 && ~snap.isCH(i)
                chNode = snap.assocCH(i);
                plot([S(i).xd, S(chNode).xd], [S(i).yd, S(chNode).yd], '-', ...
                    'Color', [0.45 0.45 0.50], 'LineWidth', 0.35);
            end
        end

        % Draw CH-to-BS links
        chNodeIds = find(snap.isCH);
        for chIdx = 1:numel(chNodeIds)
            chNode = chNodeIds(chIdx);
            plot([S(chNode).xd, sink.x], [S(chNode).yd, sink.y], '-', ...
                'Color', [1.00 0.55 0.10], 'LineWidth', 0.9);
        end

        % Draw cluster boundaries around formed clusters (circle around CH + members)
        for chIdx = 1:numel(chNodeIds)
            chNode = chNodeIds(chIdx);
            memberIdx = find((snap.assocCH == chNode) & aliveNodes);
            memberX = [S(memberIdx).xd, S(chNode).xd];
            memberY = [S(memberIdx).yd, S(chNode).yd];
            distances = hypot(memberX - S(chNode).xd, memberY - S(chNode).yd);
            radius = max([2, distances]);
            rectangle('Position', [S(chNode).xd - radius, S(chNode).yd - radius, 2 * radius, 2 * radius], ...
                'Curvature', [1 1], 'EdgeColor', [0.60 0.60 0.70], 'LineStyle', '--', 'LineWidth', 0.7);
        end

        plot([S(aliveNonCH).xd], [S(aliveNonCH).yd], '.', 'Color', [0.25 0.82 1.00], 'MarkerSize', 16);
        plot([S(deadNodes).xd], [S(deadNodes).yd], 'x', 'Color', [1.00 0.35 0.35], 'MarkerSize', 7, 'LineWidth', 1.0);
        plot([S(snap.isCH).xd], [S(snap.isCH).yd], 'p', 'Color', [1.00 0.90 0.30], ...
            'MarkerFaceColor', [1.00 0.90 0.30], 'MarkerSize', 10, 'LineWidth', 0.8);

        title(sprintf('%s - Round %d', snapshotTitlesB{sIdx}, snap.round), 'Color', axisFontColor, 'FontWeight', 'bold');
        text(2, ym - 4, sprintf('Alive: %d | Dead: %d', snap.alive, snap.dead), ...
            'Color', axisFontColor, 'FontSize', 11, 'FontWeight', 'bold');
    else
        title(sprintf('%s - Not Reached', snapshotTitlesB{sIdx}), 'Color', axisFontColor, 'FontWeight', 'bold');
        text(4, ym / 2, 'Milestone not reached in current rmax', ...
            'Color', axisFontColor, 'FontSize', 10, 'FontWeight', 'bold');
    end

    plot(sink.x, sink.y, 's', 'Color', [0.40 1.00 0.45], ...
        'MarkerFaceColor', [0.40 1.00 0.45], 'MarkerSize', 9);
    text(sink.x + 1.5, sink.y + 1.5, 'BS', 'Color', axisFontColor, 'FontSize', 10, 'FontWeight', 'bold');

    legend([hAlive, hDead, hCH, hBS, hCHBS], 'Location', 'southoutside', 'Orientation', 'horizontal', ...
        'TextColor', axisFontColor, 'Color', axisBgColor, 'EdgeColor', [0.35 0.35 0.35], 'FontSize', 10);
    xlim([0 xm]);
    ylim([0 ym]);
    xlabel('X (m)', 'Color', axisFontColor, 'FontWeight', 'bold');
    ylabel('Y (m)', 'Color', axisFontColor, 'FontWeight', 'bold');
    hold off;
end

% Sanity checks to avoid reporting artifacts
if any(diff(ALIVE(idx)) > 0)
    warning('ALIVE is expected to be non-increasing. Check node-state updates.');
end
if any(diff(RESIDUAL_ENERGY(idx)) > 1e-12)
    warning('Residual energy is expected to be non-increasing. Check energy accounting.');
end
if any(diff(THROUGHPUT_CUM(idx)) < 0)
    warning('Cumulative throughput must be non-decreasing. Check packet counters.');
end

%% ----------------------- TEXT OUTPUT ----------------------- %%
fprintf('TEEN simulation completed.\n');
fprintf('First dead node round : %g\n', first_dead);
fprintf('Half nodes dead round : %g\n', half_dead);
fprintf('All nodes dead round  : %g\n', all_dead);
fprintf('Final residual energy : %.6f J\n', RESIDUAL_ENERGY(lastRound + 1));
if isnan(first_dead)
    fprintf('Warning: No node died within rmax=%d. Consider lowering HT/ST or increasing rmax.\n', rmax);
end
