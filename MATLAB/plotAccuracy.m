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
    fig = figure('Name', sprintf('Type%d Accuracy Test', motorType.TypeNumber), ...
        'Color', 'w', 'Position', [100 100 520 * max(nUnits, 1) + 120, 760]);
    tl = tiledlayout(fig, 2, nUnits, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('Type %d — Table Accuracy Vs. Naive Linear2 Baseline', motorType.TypeNumber), ...
        'FontWeight', 'bold', 'FontSize', 13);

    for i = 1:nUnits
        nexttile(tl, i);
        plotErrorDistribution(motorType.Units(i), motorType.TypeNumber, modelOrder);
        nexttile(tl, nUnits + i);
        plotSummaryStats(motorType.Units(i), motorType.TypeNumber, modelOrder);
    end
end

function plotErrorDistribution(unitData, typeNumber, modelOrder)
    ax = gca;
    titleStr = sprintf('Type%d Unit%d — Trial Error Distribution', typeNumber, unitData.UnitNumber);

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

function plotSummaryStats(unitData, typeNumber, modelOrder)
    ax = gca;
    titleStr = sprintf('Type%d Unit%d — Summary Statistics', typeNumber, unitData.UnitNumber);

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
