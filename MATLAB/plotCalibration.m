%PLOTCALIBRATION  One figure per motor type, three tiles per unit of
%   that type: top is the fine calibration sweep's up-direction, down-
%   direction, and direction-averaged (ground truth) pulse->angle
%   curves, with light shading between the up/down curves showing the
%   hysteresis/backlash band; middle is the averaged curve's local
%   slope against pulse, revealing any bowing or other non-linearity a
%   straight line through the endpoints alone would hide; bottom is a
%   zoomed-in view of the same up/down/average/shading over a fixed-
%   width window anchored at each unit's own min pulse, since the full-
%   range view alone makes a real hysteresis band (typically under 2deg
%   against a 200+deg range) too thin to actually see. Calls setup()
%   itself, so this runs standalone.

ZOOM_WIDTH_US = 100;   % same window width for every unit, anchored at
                        % each unit's own min pulse -- an absolute
                        % window wouldn't necessarily land inside every
                        % unit's real range, but a fixed offset from
                        % each unit's own edge always does

setup;

for t = 1:numel(motors)
    plotOneType(motors(t), ZOOM_WIDTH_US);
end

function plotOneType(motorType, zoomWidthUs)
    nUnits = numel(motorType.Units);
    label = typeLabel(motorType.TypeNumber, motorType.TypeName);
    fig = figure('Name', sprintf('%s Calibration Sweeps', label), ...
        'Color', 'w', 'Position', [100 100 480 * max(nUnits, 1) + 120, 1080]);
    tl = tiledlayout(fig, 3, nUnits, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('%s — Fine Calibration Sweep: Up Vs. Down Vs. Average', label), ...
        'FontWeight', 'bold', 'FontSize', 13);

    curveAxes = gobjects(1, nUnits);
    slopeAxes = gobjects(1, nUnits);
    zoomAxes = gobjects(1, nUnits);
    for i = 1:nUnits
        curveAxes(i) = nexttile(tl, i);
        plotCurves(motorType.Units(i), motorType.TypeNumber, motorType.TypeName);
        slopeAxes(i) = nexttile(tl, nUnits + i);
        plotSlope(motorType.Units(i), motorType.TypeNumber, motorType.TypeName);
        zoomAxes(i) = nexttile(tl, 2 * nUnits + i);
        plotZoom(motorType.Units(i), motorType.TypeNumber, motorType.TypeName, zoomWidthUs);
    end
    % Each row synced independently, not across rows -- curve/slope/zoom
    % are different measurements (angle, deg/us, zoomed angle) and have
    % no shared scale to begin with.
    syncAxes(curveAxes, 'xy');
    syncAxes(slopeAxes, 'xy');
    syncAxes(zoomAxes, 'xy');
end

function syncAxes(axesHandles, whichAxis)
    %SYNCAXES  Makes every axes in axesHandles share the same limits
    %   along whichAxis ('x', 'y', or 'xy') -- the union of what each
    %   would have auto-scaled to on its own, so units within one type's
    %   figure are directly visually comparable instead of each silently
    %   getting its own scale.
    axesHandles = axesHandles(isgraphics(axesHandles));
    if numel(axesHandles) < 2
        return
    end
    if contains(whichAxis, 'x')
        xl = cell2mat(get(axesHandles, {'XLim'}));
        set(axesHandles, 'XLim', [min(xl(:, 1)), max(xl(:, 2))]);
    end
    if contains(whichAxis, 'y')
        yl = cell2mat(get(axesHandles, {'YLim'}));
        set(axesHandles, 'YLim', [min(yl(:, 1)), max(yl(:, 2))]);
    end
end

function label = typeLabel(typeNumber, typeName)
    %TYPELABEL  The resolved model name (e.g. "Miuzei 25kg Servo") when
    %   setup.m found one via motorTypeNames.m -- "Type <N>" only as a
    %   last resort when no name is registered for this type, since
    %   there's genuinely nothing else to call it.
    if isempty(typeName)
        label = sprintf('Type %d', typeNumber);
    else
        label = typeName;
    end
end

function label = unitLabel(typeNumber, typeName, unitNumber)
    %UNITLABEL  "Miuzei 25kg Servo Unit 1" -- the name/type label plus
    %   unit number, used everywhere a specific unit is identified on a
    %   plot (never bare "type1 unit1").
    label = sprintf('%s Unit %d', typeLabel(typeNumber, typeName), unitNumber);
end

function plotCurves(unitData, typeNumber, typeName)
    up = unitData.FineUp;
    down = unitData.FineDown;
    titleStr = sprintf('%s — Calibration Curve', unitLabel(typeNumber, typeName, unitData.UnitNumber));

    if isempty(up) || isempty(down)
        emptyPanel(gca, titleStr, 'No Calibration Sweep Yet');
        return
    end

    drawCalibrationTraces(up, down, []);

    grid on;
    box on;
    xlabel('Pulse Width (µs)');
    ylabel('Shaft Angle, Relative To Min µs (°)');
    title(titleStr);
    legend('Location', 'eastoutside');
end

function plotZoom(unitData, typeNumber, typeName, zoomWidthUs)
    up = unitData.FineUp;
    down = unitData.FineDown;
    titleStr = sprintf('%s — Zoomed Near Min µs', unitLabel(typeNumber, typeName, unitData.UnitNumber));

    if isempty(up) || isempty(down)
        emptyPanel(gca, titleStr, 'No Calibration Sweep Yet');
        return
    end

    allPulse = intersect(up.pulse_us, down.pulse_us);
    lo = min(allPulse);
    hi = min(lo + zoomWidthUs, max(allPulse));

    drawCalibrationTraces(up, down, [lo hi]);
    xlim([lo hi]);
    setTightYLim(up, down, lo, hi);

    grid on;
    box on;
    xlabel('Pulse Width (µs)');
    ylabel('Shaft Angle, Relative To Min µs (°)');
    title(titleStr);
    legend('Location', 'eastoutside');
end

function drawCalibrationTraces(up, down, xRange)
    % Shared by plotCurves() (full range) and plotZoom() (windowed) --
    % same traces, same styling, either the whole sweep or a slice of
    % it. Up/down recede (cool vs. warm, semi-transparent) since they're
    % the raw constituent passes; average is the bold, fully-opaque
    % "hero" line -- the one real answer the other two exist to build.
    % Its color is reused for the trusted line in the slope tile, so all
    % three tiles for one unit read as one coherent set.
    upColor      = [0.20 0.45 0.70, 0.55];
    downColor    = [0.80 0.35 0.25, 0.55];
    avgColor     = heroColor();
    shadingColor = [0.55 0.45 0.60];

    if ~isempty(xRange)
        up = up(up.pulse_us >= xRange(1) & up.pulse_us <= xRange(2), :);
        down = down(down.pulse_us >= xRange(1) & down.pulse_us <= xRange(2), :);
    end

    hold on;
    if ~isempty(up) && ~isempty(down)
        [commonPulse, avgAngle] = averageByPulse(up, down);
        shadeBetween(up, down, commonPulse, shadingColor);
        plot(commonPulse, avgAngle, '-', 'Color', avgColor, 'LineWidth', 2.5, 'DisplayName', 'Average');
    end
    if ~isempty(up)
        plot(up.pulse_us, up.angle_deg, 'o-', 'Color', upColor, 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', 'Up');
    end
    if ~isempty(down)
        plot(down.pulse_us, down.angle_deg, 'o-', 'Color', downColor, 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', 'Down');
    end
    hold off;
end

function setTightYLim(up, down, lo, hi)
    %SETTIGHTYLIM  auto-y from the full-range data would still be scaled
    %   to the whole sweep, defeating the point of zooming in -- rescale
    %   to just what's actually visible in [lo hi], with a small margin.
    inUp = up.angle_deg(up.pulse_us >= lo & up.pulse_us <= hi);
    inDown = down.angle_deg(down.pulse_us >= lo & down.pulse_us <= hi);
    yAll = [inUp; inDown];
    if isempty(yAll)
        return
    end
    span = max(yAll) - min(yAll);
    margin = max(span * 0.15, 0.05);
    ylim([min(yAll) - margin, max(yAll) + margin]);
end

function plotSlope(unitData, typeNumber, typeName)
    rawColor    = [0.80 0.80 0.85];
    smoothColor = heroColor();   % same as the top tile's "Average" line -- see plotCurves()
    refColor    = [0.40 0.40 0.40];
    smoothWindow = 21;   % points, ~100us at the 5us fine-sweep step -- enough to
                          % average out single-count encoder quantization jitter
                          % without smoothing away a genuine bow across the range

    up = unitData.FineUp;
    down = unitData.FineDown;
    titleStr = sprintf('%s — Local Slope', unitLabel(typeNumber, typeName, unitData.UnitNumber));

    if isempty(up) || isempty(down)
        emptyPanel(gca, titleStr, 'No Calibration Sweep Yet');
        return
    end

    [commonPulse, avgAngle] = averageByPulse(up, down);
    % Smoothing the curve before differentiating, not the raw derivative
    % after -- Savitzky-Golay preserves a genuine large-scale bow while
    % rejecting the quantization-scale noise a plain point-to-point
    % gradient amplifies (the AS5600's own ~0.09deg step shows up as a
    % huge swing in a raw derivative over a mere 5us pulse step).
    smoothedAngle = smoothdata(avgAngle, 'sgolay', smoothWindow);
    rawSlope = gradient(avgAngle, commonPulse);
    smoothSlope = gradient(smoothedAngle, commonPulse);
    overallSlope = polyfit(commonPulse, avgAngle, 1);

    hold on;
    plot(commonPulse, rawSlope, '-', 'Color', rawColor, 'LineWidth', 0.75, ...
        'DisplayName', 'Raw (Point-To-Point)');
    yline(overallSlope(1), '--', 'Color', refColor, 'LineWidth', 1, ...
        'DisplayName', 'Overall Best-Fit Slope');
    plot(commonPulse, smoothSlope, '-', 'Color', smoothColor, 'LineWidth', 2.5, ...
        'DisplayName', 'Smoothed Local Slope');
    hold off;

    grid on;
    box on;
    xlabel('Pulse Width (µs)');
    ylabel('Local Slope (°/µs)');
    title(titleStr);
    legend('Location', 'eastoutside');
end

function c = heroColor()
    %HEROCOLOR  The one shared "trusted answer" color used for both the
    %   averaged calibration curve and its smoothed slope -- deep navy,
    %   fully opaque, distinct from the semi-transparent up/down/raw
    %   traces it's built from.
    c = [0.12 0.14 0.35];
end

function [commonPulse, avgAngle] = averageByPulse(up, down)
    %AVERAGEBYPULSE  Direction-averaged ground truth at every pulse both
    %   sweeps share -- same convention as build_ground_truth() in
    %   study_range.py, re-implemented here rather than read from a CSV
    %   since study_range.py never writes the averaged curve out on its
    %   own (only feeds it straight into the calibration table builder).
    [commonPulse, iUp, iDown] = intersect(up.pulse_us, down.pulse_us);
    avgAngle = (up.angle_deg(iUp) + down.angle_deg(iDown)) / 2;
end

function shadeBetween(up, down, commonPulse, color)
    %SHADEBETWEEN  Light fill between the up/down curves over their
    %   shared pulse range -- the hysteresis/backlash band. Built from
    %   the same shared-pulse points averageByPulse() uses, so the
    %   shaded region's edges are exactly the up/down lines, not an
    %   approximation across mismatched sample points.
    [~, iUp] = ismember(commonPulse, up.pulse_us);
    [~, iDown] = ismember(commonPulse, down.pulse_us);
    xPoly = [commonPulse; flipud(commonPulse)];
    yPoly = [up.angle_deg(iUp); flipud(down.angle_deg(iDown))];
    fill(xPoly, yPoly, color, 'FaceAlpha', 0.15, 'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
end

function emptyPanel(ax, titleStr, message)
    text(ax, 0.5, 0.5, message, 'HorizontalAlignment', 'center', ...
        'Units', 'normalized', 'FontAngle', 'italic', 'Color', [0.5 0.5 0.5]);
    xlim(ax, [0 1]);
    ylim(ax, [0 1]);
    ax.XTick = [];
    ax.YTick = [];
    box(ax, 'on');
    title(ax, titleStr);
end
