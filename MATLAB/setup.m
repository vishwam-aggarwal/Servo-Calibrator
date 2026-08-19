%SETUP  Force this folder and the ServoDAQ data folder onto the MATLAB
%   path, then parse every study_range.py CSV in the data folder into
%   one MotorTypeData object per motor type actually present, assigned
%   to `motors` in this workspace.
%
%   The number of motor types (and the units within each type) is
%   discovered from the filenames on disk -- nothing here assumes how
%   many there are or will ever be. Run again any time new study data
%   has been added; it re-scans from scratch every call.
%
%   Filenames matched: <kind>_type<N>_unit<M>_<YYYYMMDD-HHMMSS>.csv --
%   study_range.py's own naming convention (see ../ServoDAQ/README.md
%   and ../ServoDAQ/MOTOR_TYPES.md for what a type/unit number means).
%   Anything else sitting in the data folder (plots, .log files) is
%   silently skipped.

matlabDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(matlabDir, '..', 'ServoDAQ', 'data');
if ~isfolder(dataDir)
    error('setup:noDataDir', 'Data folder not found: %s', dataDir);
end
addpath(matlabDir);   % so MotorTypeData resolves regardless of the current folder
addpath(dataDir);     % forced every call, not just "if not already on path"

motors = loadMotorData(dataDir);
motors = attachTypeNames(motors);

fprintf('setup: %d motor type(s), %d unit(s) total\n', ...
    numel(motors), sum(arrayfun(@(m) numel(m.Units), motors)));
for i = 1:numel(motors)
    label = sprintf('type%d', motors(i).TypeNumber);
    if ~isempty(motors(i).TypeName)
        label = sprintf('%s (type%d)', motors(i).TypeName, motors(i).TypeNumber);
    end
    fprintf('  %s: unit%s\n', label, ...
        strjoin(arrayfun(@(u) string(u.UnitNumber), motors(i).Units), ', '));
end

function motors = attachTypeNames(motors)
    %ATTACHTYPENAMES  Fills in TypeName from motorTypeNames.m wherever
    %   that lookup has an entry for a type actually present -- a type
    %   with no entry (e.g. type0/unlabeled runs) just keeps TypeName
    %   as '', not a placeholder guess.
    names = motorTypeNames();
    for i = 1:numel(motors)
        if isKey(names, motors(i).TypeNumber)
            motors(i).TypeName = names(motors(i).TypeNumber);
        end
    end
end

function motors = loadMotorData(dataDir)
    files = dir(fullfile(dataDir, '*.csv'));
    pattern = '^(?<kind>.+)_type(?<type>\d+)_unit(?<unit>\d+)_(?<stamp>\d{8}-\d{6})\.csv$';

    records = struct('kind', {}, 'type', {}, 'unit', {}, 'stamp', {}, 'file', {});
    for i = 1:numel(files)
        tok = regexp(files(i).name, pattern, 'names');
        if isempty(tok)
            continue   % not a study_range.py output file -- skip silently
        end
        records(end+1) = struct( ...                                     %#ok<AGROW>
            'kind', tok.kind, 'type', str2double(tok.type), ...
            'unit', str2double(tok.unit), 'stamp', tok.stamp, ...
            'file', fullfile(dataDir, files(i).name));
    end

    if isempty(records)
        error('setup:noMatches', ...
            'No study_range.py-style CSVs found in %s', dataDir);
    end

    allKinds = unique({records.kind});
    typeNumbers = unique([records.type]);

    motors = MotorTypeData.empty(1, 0);
    for i = 1:numel(typeNumbers)
        typeRecords = records([records.type] == typeNumbers(i));
        motors(i) = MotorTypeData(typeNumbers(i), typeRecords, allKinds);
    end
end
