function T = gimbal_import_csv(inputFile, mapping, outputFile)
% Normalize EmberProbe/LinkScope exports with explicit columns and units.
% Does not interpolate, fill missing samples or assume a sample frequency.
% mapping.timeColumn = "time"; mapping.timeUnit = "iso8601"|"s"|"ms"|"us";
% mapping.signals.theta_rad = struct('column',"angle:f32",'scale',pi/180);
% Optional offset is applied AFTER scale. Units/signs must be verified first.
opts = detectImportOptions(inputFile, 'Delimiter', ',', 'VariableNamingRule', 'preserve');
opts.VariableNamesLine = 1;
opts.DataLines = [2, Inf];
opts = setvartype(opts, opts.VariableNames, 'string');
raw = readtable(inputFile, opts);
assert(ismember(string(mapping.timeColumn), string(raw.Properties.VariableNames)), ...
    'Time column does not exist. Inspect the original CSV header.');
timeText = raw.(char(mapping.timeColumn));
if string(mapping.timeUnit) == "iso8601"
    stamp = datetime(timeText, 'InputFormat', "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", ...
        'TimeZone', 'UTC');
    t = seconds(stamp - stamp(1));
else
    factors = struct('s',1,'ms',1e-3,'us',1e-6);
    assert(isfield(factors, char(mapping.timeUnit)), 'Unsupported time unit.');
    t = str2double(timeText) * factors.(char(mapping.timeUnit));
    t = t - t(1);
end
assert(all(isfinite(t)) && all(diff(t)>0), ...
    'Time must be finite and strictly increasing. Split resets/sessions first.');
T = table(t, 'VariableNames', {'t_s'});
names = fieldnames(mapping.signals);
for k = 1:numel(names)
    spec = mapping.signals.(names{k});
    assert(ismember(string(spec.column), string(raw.Properties.VariableNames)), ...
        'Mapped column does not exist: %s', char(spec.column));
    assert(isfield(spec,'scale') && isfinite(spec.scale) && isscalar(spec.scale), ...
        'Provide an explicit finite scale for each signal.');
    offset = 0;
    if isfield(spec,'offset'), offset = spec.offset; end
    assert(isscalar(offset) && isfinite(offset), 'Offset must be finite.');
    T.(names{k}) = str2double(raw.(char(spec.column))) * spec.scale + offset;
end
if nargin >= 3 && strlength(string(outputFile))>0
    writetable(T, outputFile);
end
fprintf('Imported %d rows; missing values remain missing.\n', height(T));
end
