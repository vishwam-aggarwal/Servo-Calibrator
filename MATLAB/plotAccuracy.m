%PLOTACCURACY  One figure per motor type, two tiles per unit of that
%   type (top: raw per-trial error distribution, bottom: summary
%   statistics), comparing the naive 2-point linear model against the
%   N-point lookup tables. Calls setup() itself, so this runs
%   standalone.

MODEL_ORDER = {'linear2', 'table10', 'table20', 'table30', 'table40', 'table50'};

setup;

for t = 1:numel(motors)
    plotOneType(motors(t), MODEL_ORDER);
end

function plotOneType(motorType, modelOrder)
    nUnits = numel(motorType.Units);
    label = typeLabel(motorType.TypeNumber, motorType.TypeName);
    fig = figure('Name', sprintf('%s Accuracy Test', label), ...
        'Color', 'w', 'Position', [100 100 520 * max(nUnits, 1) + 120, 760]);
    tl = tiledlayout(fig, 2, nUnits, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('%s — Table Accuracy Vs. Naive Linear2 Baseline', label), ...
        'FontWeight', 'bold', 'FontSize', 13);

    distAxes = gobjects(1, nUnits);
    statsAxes = gobjects(1, nUnits);
    for i = 1:nUnits
        distAxes(i) = nexttile(tl, i);
        plotErrorDistribution(motorType.Units(i), motorType.TypeNumber, motorType.TypeName, modelOrder);
        statsAxes(i) = nexttile(tl, nUnits + i);
        plotSummaryStats(motorType.Units(i), motorType.TypeNumber, motorType.TypeName, modelOrder);
    end
    % Y only -- the x-axis on both rows is the categorical model list,
    % already identical across units (MODEL_ORDER), nothing to sync.
    syncAxes(distAxes, 'y');
    syncAxes(statsAxes, 'y');
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

function plotErrorDistribution(unitData, typeNumber, typeName, modelOrder)
    ax = gca;
    titleStr = sprintf('%s — Trial Error Distribution', unitLabel(typeNumber, typeName, unitData.UnitNumber));

    if isempty(unitData.AccuracyTrials)
        emptyPanel(ax, titleStr, 'No Accuracy Trials Yet');
        return
    end

    trials = unitData.AccuracyTrials;
    present = ismember(modelOrder, unique(trials.model));
    order = modelOrder(present);
    modelCat = categorical(trials.model, order, 'Ordinal', true);

    colors = modelColors(numel(order));
    hold(ax, 'on');
    yline(ax, 0, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1, 'HandleVisibility', 'off');
    for m = 1:numel(order)
        inModel = modelCat == order{m};
        boxchart(ax, modelCat(inModel), trials.error_deg(inModel), ...
            'BoxFaceColor', colors(m, :), 'MarkerColor', colors(m, :) * 0.7, ...
            'BoxFaceAlpha', 0.75, 'MarkerStyle', '.');
    end
    hold(ax, 'off');

    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'Model');
    ylabel(ax, 'Error, Actual − Target (°)');
    title(ax, titleStr);
    subtitle(ax, sprintf('N = %d Trials', height(trials)));
end

function plotSummaryStats(unitData, typeNumber, typeName, modelOrder)
    ax = gca;
    titleStr = sprintf('%s — Summary Statistics', unitLabel(typeNumber, typeName, unitData.UnitNumber));

    if isempty(unitData.AccuracySummary)
        emptyPanel(ax, titleStr, 'Accuracy Test Still Running — No Summary Yet');
        return
    end

    s = unitData.AccuracySummary;
    present = ismember(modelOrder, s.model);
    order = modelOrder(present);
    [~, idx] = ismember(order, s.model);
    statsMatrix = [s.mean_abs_error_deg(idx), s.rms_error_deg(idx), s.max_abs_error_deg(idx)];

    modelCat = categorical(order, order, 'Ordinal', true);
    b = bar(ax, modelCat, statsMatrix, 'grouped');
    b(1).FaceColor = [0.31 0.51 0.74];   % mean -- cool, the headline number
    b(2).FaceColor = [0.13 0.29 0.49];   % rms -- darker shade, same family
    b(3).FaceColor = [0.75 0.22 0.17];   % max -- warm, draws the eye to worst-case

    labelBars(ax, b(1));

    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'Model');
    ylabel(ax, 'Error (°)');
    legend(ax, {'Mean |Err|', 'RMS', 'Max |Err|'}, 'Location', 'eastoutside');
    title(ax, titleStr);
end

function colors = modelColors(n)
    colors = parula(n + 1);
    colors = colors(1:n, :);
end

function labelBars(ax, barSeries)
    xTips = barSeries.XEndPoints;
    yTips = barSeries.YEndPoints;
    for i = 1:numel(xTips)
        text(ax, xTips(i), yTips(i), sprintf('%.2f', yTips(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8);
    end
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
