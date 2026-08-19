%PLOTERRORVSANGLE  One figure per motor type, one subplot per unit of
%   that type: mean |error| vs. target angle, one line per model,
%   binned into deciles across each unit's own angle range. The
%   accuracy box plots (plotAccuracy.m) collapse every trial across the
%   whole range into one distribution per model -- this instead shows
%   WHERE in the range each model's error is worst, which a flat
%   aggregate number hides (e.g. linear2's error climbing steeply
%   toward one end because the real curve bows away from a straight
%   line -- see CLAUDE.md's own decile finding). Calls setup() itself,
%   so this runs standalone.

MODEL_ORDER = {'linear2', 'table10', 'table20', 'table30', 'table40', 'table50'};
NUM_BINS = 10;   % deciles across each unit's own target-angle range

setup;

for t = 1:numel(motors)
    plotOneType(motors(t), MODEL_ORDER, NUM_BINS);
end

function plotOneType(motorType, modelOrder, numBins)
    nUnits = numel(motorType.Units);
    label = typeLabel(motorType.TypeNumber, motorType.TypeName);
    fig = figure('Name', sprintf('%s Error Vs. Angle', label), ...
        'Color', 'w', 'Position', [100 100 480 * max(nUnits, 1) + 120, 480]);
    tl = tiledlayout(fig, 1, nUnits, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('%s — Mean |Error| By Target Angle Decile', label), ...
        'FontWeight', 'bold', 'FontSize', 13);

    axesHandles = gobjects(1, nUnits);
    for i = 1:nUnits
        axesHandles(i) = nexttile(tl, i);
        plotOneUnit(motorType.Units(i), motorType.TypeNumber, motorType.TypeName, modelOrder, numBins);
    end
    syncAxes(axesHandles, 'xy');
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

function plotOneUnit(unitData, typeNumber, typeName, modelOrder, numBins)
    titleStr = sprintf('%s — Error By Angle', unitLabel(typeNumber, typeName, unitData.UnitNumber));

    if isempty(unitData.AccuracyTrials)
        emptyPanel(gca, titleStr, 'No Accuracy Trials Yet');
        return
    end

    trials = unitData.AccuracyTrials;
    present = ismember(modelOrder, unique(trials.model));
    order = modelOrder(present);
    colors = modelColors(numel(order));

    edges = linspace(min(trials.target_angle_deg), max(trials.target_angle_deg), numBins + 1);
    binCenters = (edges(1:end-1) + edges(2:end)) / 2;

    hold on;
    for m = 1:numel(order)
        inModel = strcmp(trials.model, order{m});
        angles = trials.target_angle_deg(inModel);
        errs = abs(trials.error_deg(inModel));
        binIdx = discretize(angles, edges);

        binMean = accumarray(binIdx(~isnan(binIdx)), errs(~isnan(binIdx)), [numBins, 1], @mean, NaN);

        isLinear = strcmp(order{m}, 'linear2');
        if isLinear
            % The one model expected to actually bow -- made visually
            % distinct (thicker, its own color) rather than folded into
            % the table-model gradient, since it's the point of comparison.
            plot(binCenters, binMean, 'o-', 'Color', [0.75 0.15 0.15], ...
                'LineWidth', 2.5, 'MarkerSize', 5, 'MarkerFaceColor', [0.75 0.15 0.15], ...
                'DisplayName', order{m});
        else
            plot(binCenters, binMean, 'o-', 'Color', colors(m, :), ...
                'LineWidth', 1.25, 'MarkerSize', 4, 'DisplayName', order{m});
        end
    end
    hold off;

    grid on;
    box on;
    xlabel('Target Angle (°)');
    ylabel('Mean |Error| (°)');
    title(titleStr);
    subtitle(sprintf('N = %d Trials, %d Bins', height(trials), numBins));
    legend('Location', 'eastoutside');
end

function colors = modelColors(n)
    % Same parula-family gradient used for models in plotAccuracy.m --
    % duplicated here rather than shared, matching every other script's
    % standalone-file convention in this project.
    colors = parula(n + 1);
    colors = colors(1:n, :);
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
