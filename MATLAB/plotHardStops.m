%PLOTHARDSTOPS  One figure per motor type, one subplot per unit of that
%   type, overlaying the naive/coarse/fine hard-stop range-finding
%   sweeps (pulse width vs. shaft angle) with the identified min/max
%   pulse clearly marked. Calls setup() itself, so this runs standalone.

setup;

for t = 1:numel(motors)
    plotOneType(motors(t));
end

function plotOneType(motorType)
    nUnits = numel(motorType.Units);
    fig = figure('Name', sprintf('Type%d Hard-Stop Sweeps', motorType.TypeNumber), ...
        'Color', 'w');
    tl = tiledlayout(fig, 1, nUnits, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('Type %d — Naive Vs. Coarse Vs. Fine Range-Finding', motorType.TypeNumber), ...
        'FontWeight', 'bold', 'FontSize', 12);

    for i = 1:nUnits
        nexttile(tl);
        plotOneUnit(motorType.Units(i), motorType.TypeNumber);
    end
end

function plotOneUnit(unitData, typeNumber)
    % Same parula-family gradient logic as plotAccuracy.m's model
    % colors, here standing in for increasing trust: naive is the crude
    % baseline, coarse narrows it down, fine is the final, most precise
    % pass -- so color AND line thickness both step up together,
    % reinforcing which trace to actually believe.
    stageColors = parula(4);
    naiveColor  = [stageColors(1, :), 0.55];   % alpha -- the least-trusted trace recedes
    coarseColor = [stageColors(2, :), 0.75];
    fineColor   = stageColors(3, :);
    markerColor = [0.93 0.69 0.13];

    % Angle columns arrive already normalized to the identified min-pulse
    % edge (0deg) -- see MotorTypeData.normalizeAngles(), which does this
    % once for every unit/table so every script gets it for free instead
    % of each one recomputing its own.
    lowRow = table();
    highRow = table();
    if ~isempty(unitData.Summary)
        s = unitData.Summary;
        isSmart = strcmp(s.method, 'smart');
        lowRow = s(isSmart & strcmp(s.side, 'low'), :);
        highRow = s(isSmart & strcmp(s.side, 'high'), :);
    end

    hold on;
    label = sprintf('type%d unit%d', typeNumber, unitData.UnitNumber);
    plotSweepPair(cleanTrace(unitData.NaiveLow, [label ' naive low']), ...
                  cleanTrace(unitData.NaiveHigh, [label ' naive high']), ...
                  naiveColor, 1.0, 'Naive');
    plotSweepPair(cleanTrace(unitData.SmartCoarseLow, [label ' coarse low']), ...
                  cleanTrace(unitData.SmartCoarseHigh, [label ' coarse high']), ...
                  coarseColor, 1.5, 'Coarse');
    plotSweepPair(cleanTrace(unitData.SmartFineLow, [label ' fine low']), ...
                  cleanTrace(unitData.SmartFineHigh, [label ' fine high']), ...
                  fineColor, 2.5, 'Fine');

    if ~isempty(lowRow)
        markEdge(lowRow.pulse_us(1), lowRow.angle_deg(1), 'Min', markerColor);
    end
    if ~isempty(highRow)
        markEdge(highRow.pulse_us(1), highRow.angle_deg(1), 'Max', markerColor);
    end
    hold off;

    grid on;
    box on;
    xlabel('Pulse Width (µs)');
    ylabel('Shaft Angle, Relative To Min µs (°)');
    title(sprintf('Type%d Unit%d', typeNumber, unitData.UnitNumber));
    legend('Location', 'eastoutside');
end

function plotSweepPair(lowTable, highTable, color, lineWidth, label)
    % Plots the low- and high-side traces of one sweep kind in the same
    % color, on the already-held-open current axes -- one legend entry
    % per sweep kind, not two, however many of the pair are present.
    labelUsed = false;
    if ~isempty(lowTable)
        plot(lowTable.pulse_us, lowTable.angle_deg, 'o-', ...
            'Color', color, 'LineWidth', lineWidth, 'MarkerSize', 3, 'DisplayName', label);
        labelUsed = true;
    end
    if ~isempty(highTable)
        if labelUsed
            plot(highTable.pulse_us, highTable.angle_deg, 'o-', ...
                'Color', color, 'LineWidth', lineWidth, 'MarkerSize', 3, 'HandleVisibility', 'off');
        else
            plot(highTable.pulse_us, highTable.angle_deg, 'o-', ...
                'Color', color, 'LineWidth', lineWidth, 'MarkerSize', 3, 'DisplayName', label);
        end
    end
end

function tbl = cleanTrace(tbl, traceLabel)
    %CLEANTRACE  The two artifact-removal passes applied to every sweep
    %   trace before plotting -- see each pass's own docstring. Order
    %   matters: the first-point outlier must go before the monotonicity
    %   check establishes "the expected direction" from the trace's
    %   early points, or a bad first point could set the wrong
    %   expectation.
    tbl = dropFirstPointOutlier(tbl, traceLabel);
    tbl = dropNonMonotonicTail(tbl, traceLabel);
end

function tbl = dropNonMonotonicTail(tbl, traceLabel)
    %DROPNONMONOTONICTAIL  Truncates a sweep right before the first
    %   SUSTAINED reversal in its direction of travel -- e.g. a real
    %   stall-recovery backoff (see ../ServoDAQ/README.md's own
    %   investigation) or a spin-past-the-edge event: real servo
    %   behavior right at the physical limit that the naive method
    %   (unlike the coarse/fine algorithm's own reversal check) has no
    %   protection against on its own.
    %
    %   Two things have to both hold to avoid mistaking ordinary noise
    %   for a real reversal: the expected direction is the MAJORITY sign
    %   across the whole trace, not just an early window (a window that
    %   itself starts in a slow/plateau region can be wrong about the
    %   trend -- real data, caught the hard way: a lone -0.44deg blip
    %   right where a fine sweep starts near its own edge, surrounded by
    %   genuine +deg steps either side, is real noise, not the start of
    %   a reversal); and a reversal must persist for sustainCount
    %   consecutive steps, not just one -- *unless* it's the very last
    %   step in the trace, with nothing after it to disprove sustainment
    %   either (a coarse-resolution pass can legitimately hit the real
    %   edge and stop within a single step of it, leaving no second
    %   point to confirm against; real data, not a reason to miss it).
    %   An isolated contra-direction blip anywhere else is exactly what
    %   real noise near a slowing region looks like. Logged, not silent.
    noiseFloorDeg = 0.05;
    if isempty(tbl) || height(tbl) < 4
        return
    end

    deltas = diff(tbl.angle_deg);
    isPos = deltas > noiseFloorDeg;
    isNeg = deltas < -noiseFloorDeg;
    if sum(isPos) == sum(isNeg)
        return   % no clear majority direction to judge against
    end
    expectedSign = sign(sum(isPos) - sum(isNeg));
    reversed = (sign(deltas) == -expectedSign) & (isPos | isNeg);

    reversedIdx = [];
    for i = 1:numel(reversed)
        if reversed(i) && (i == numel(reversed) || reversed(i + 1))
            reversedIdx = i;
            break
        end
    end
    if ~isempty(reversedIdx)
        fprintf('  [%s] dropped %d non-monotonic tail sample(s), starting at %gus -> %.2fdeg\n', ...
            traceLabel, height(tbl) - reversedIdx, tbl.pulse_us(reversedIdx + 1), tbl.angle_deg(reversedIdx + 1));
        tbl = tbl(1:reversedIdx, :);
    end
end

function tbl = dropFirstPointOutlier(tbl, traceLabel)
    %DROPFIRSTPOINTOUTLIER  Drops a sweep's first row if it looks like a
    %   transient/stale reading rather than real motion, not just for
    %   type3 -- e.g. the servo's own first-command twitch right after a
    %   fresh connection, seen so far only on the very first sweep of a
    %   session. Detected generically, not by unit/type: the first
    %   step's delta is compared against the second step's, which
    %   reflects genuine local behavior once things have had a moment to
    %   settle. Units whose first point already matches its neighbors
    %   (the normal case) are left untouched. Logged, not silent -- this
    %   is worth noticing if it starts happening often or on new units,
    %   not just quietly discarded.
    minAbsDeg = 5;   % ignore near-zero first steps -- nothing to flag
    ratio = 8;       % first step must be at least this many times the second
    if isempty(tbl) || height(tbl) < 3
        return
    end
    delta1 = tbl.angle_deg(2) - tbl.angle_deg(1);
    delta2 = tbl.angle_deg(3) - tbl.angle_deg(2);
    if abs(delta1) > minAbsDeg && abs(delta1) > ratio * max(abs(delta2), eps)
        fprintf('  [%s] dropped first-point outlier: %gus -> %.2fdeg (next: %gus -> %.2fdeg)\n', ...
            traceLabel, tbl.pulse_us(1), tbl.angle_deg(1), tbl.pulse_us(2), tbl.angle_deg(2));
        tbl(1, :) = [];
    end
end

function markEdge(pulseUs, angleDeg, label, color)
    plot(pulseUs, angleDeg, 'p', 'MarkerSize', 16, 'MarkerFaceColor', color, ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1, 'HandleVisibility', 'off');

    % Push the label away from the marker, direction chosen by which
    % corner of the axes the point is actually in (min sits bottom-left,
    % max sits top-right, but this doesn't assume that -- it reads the
    % current axes limits so it still lands sensibly either way), so it
    % clears both the star glyph and the axes edge instead of sitting
    % right on top of the marker. A background box keeps it legible over
    % whatever data/gridlines happen to be behind it.
    xl = xlim;
    yl = ylim;
    if pulseUs < mean(xl)
        ha = 'left';
        dx = 0.025 * diff(xl);
    else
        ha = 'right';
        dx = -0.025 * diff(xl);
    end
    if angleDeg < mean(yl)
        va = 'bottom';
        dy = 0.06 * diff(yl);
    else
        va = 'top';
        dy = -0.06 * diff(yl);
    end
    % A thin leader line ties the label to its marker explicitly, rather
    % than relying on placement alone -- with a diagonal curve running
    % right into the marker, the offset above (chosen just to clear the
    % axes edge) can otherwise land close enough to the incoming trace
    % that the label reads as pointing at the sweep, not the star.
    plot([pulseUs, pulseUs + dx], [angleDeg, angleDeg + dy], '-', ...
        'Color', [0.45 0.45 0.45], 'LineWidth', 0.75, 'HandleVisibility', 'off');
    text(pulseUs + dx, angleDeg + dy, sprintf('%s: %gµs, %.1f°', label, pulseUs, angleDeg), ...
        'HorizontalAlignment', ha, 'VerticalAlignment', va, ...
        'FontWeight', 'bold', 'FontSize', 8.5, ...
        'BackgroundColor', [1 1 1 0.88], 'EdgeColor', [0.6 0.6 0.6], 'Margin', 2);
end
