classdef MotorTypeData
    %MOTORTYPEDATA  All studied units of one motor type.
    %
    %   One instance per motor type actually found in the data folder
    %   (see setup.m) -- see ../ServoDAQ/MOTOR_TYPES.md for what a type
    %   number means (which physical servo model). Units is a struct
    %   array, one entry per physical unit found, each holding that
    %   unit's most recent study run as MATLAB tables.
    %
    %   "Most recent" matters: if a unit was studied more than once
    %   (e.g. a short smoke test before the real 3h run), only the
    %   latest-timestamped run is kept -- a rerun supersedes an earlier
    %   partial/smoke one, and runs from different stamps sit on
    %   different, incomparable position references (opening the serial
    %   port re-zeros the board's tracking -- see ../CLAUDE.md), so
    %   they must never be mixed.
    %
    %   Every absolute-angle column (anything named "*angle_deg") across
    %   every table is normalized so the identified min-pulse edge sits
    %   at 0deg -- see normalizeAngles() -- so every script that reads a
    %   unit's data gets the same normalized frame automatically,
    %   instead of each plotting script recomputing its own.
    %
    %   Example:
    %       motors = setup();
    %       t1 = motors([motors.TypeNumber] == 1);
    %       u2 = t1.unit(2);
    %       plot(u2.FineUp.pulse_us, u2.FineUp.angle_deg)

    properties
        TypeNumber
        Units
    end

    methods
        function obj = MotorTypeData(typeNumber, records, allKinds)
            obj.TypeNumber = typeNumber;
            unitNumbers = unique([records.unit]);
            obj.Units = repmat(MotorTypeData.emptyUnit(allKinds), 1, numel(unitNumbers));
            for i = 1:numel(unitNumbers)
                unitRecords = records([records.unit] == unitNumbers(i));
                obj.Units(i) = MotorTypeData.buildUnit(unitNumbers(i), unitRecords, allKinds);
            end
        end

        function u = unit(obj, unitNumber)
            %UNIT  Convenience accessor, e.g. motors(1).unit(2), instead
            %   of indexing into the Units struct array by hand.
            u = obj.Units([obj.Units.UnitNumber] == unitNumber);
            if isempty(u)
                error('MotorTypeData:noSuchUnit', ...
                    'type%d has no unit%d', obj.TypeNumber, unitNumber);
            end
        end
    end

    methods (Static, Access = private)
        function u = buildUnit(unitNumber, records, allKinds)
            stamps = unique({records.stamp});
            latestStamp = stamps{end};   % YYYYMMDD-HHMMSS sorts lexically = chronologically
            records = records(strcmp({records.stamp}, latestStamp));

            u = MotorTypeData.emptyUnit(allKinds);
            u.UnitNumber = unitNumber;
            u.Stamp = latestStamp;
            for i = 1:numel(records)
                u.(MotorTypeData.kindToField(records(i).kind)) = readtable(records(i).file);
            end
            u = MotorTypeData.normalizeAngles(u);
        end

        function u = normalizeAngles(u)
            %NORMALIZEANGLES  Shifts every absolute-angle column in this
            %   unit's tables so the identified min-pulse edge (the
            %   Summary table's smart/low row) sits at 0deg. Only
            %   columns literally named "*angle_deg" (angle_deg,
            %   target_angle_deg, actual_angle_deg) are absolute
            %   positions and get shifted -- error_deg and any
            %   "*_error_deg" summary statistic are differences,
            %   already offset-independent, and must not be touched (a
            %   constant shift on both sides of a subtraction cancels
            %   out, so leaving them alone is also the mathematically
            %   correct choice, not just a naming-convention one).
            if isempty(u.Summary)
                return
            end
            s = u.Summary;
            lowRow = s(strcmp(s.method, 'smart') & strcmp(s.side, 'low'), :);
            if isempty(lowRow) || lowRow.angle_deg(1) == 0
                return
            end
            refDeg = lowRow.angle_deg(1);

            fields = fieldnames(u);
            for i = 1:numel(fields)
                val = u.(fields{i});
                if istable(val)
                    u.(fields{i}) = MotorTypeData.shiftAngleColumns(val, refDeg);
                end
            end
        end

        function tbl = shiftAngleColumns(tbl, refDeg)
            varNames = tbl.Properties.VariableNames;
            for i = 1:numel(varNames)
                if endsWith(varNames{i}, 'angle_deg')
                    tbl.(varNames{i}) = tbl.(varNames{i}) - refDeg;
                end
            end
        end

        function fieldName = kindToField(kind)
            % naive_low -> NaiveLow, accuracy_trials -> AccuracyTrials, etc.
            words = strsplit(kind, '_');
            words = cellfun(@(w) [upper(w(1)), w(2:end)], words, 'UniformOutput', false);
            fieldName = strjoin(words, '');
        end

        function u = emptyUnit(allKinds)
            % Every unit gets the same field set (whatever CSV kinds
            % exist anywhere in the data folder), defaulted to [] --
            % lets calling code check isempty(unit.AccuracyTrials)
            % safely instead of needing isfield() everywhere, even for
            % a unit whose accuracy test hasn't finished yet.
            fieldNames = cellfun(@MotorTypeData.kindToField, allKinds, 'UniformOutput', false);
            args = [fieldNames; repmat({[]}, 1, numel(fieldNames))];
            u = struct('UnitNumber', [], 'Stamp', '', args{:});
        end
    end
end
