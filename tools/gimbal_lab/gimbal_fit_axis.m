function model = gimbal_fit_axis(csvFile, options)
% Preliminary integral least squares, NOT a certified hardware identification.
% dot(omega) = gain*u - damping*omega - friction*tanh(omega/v0) + bias.
% CSV: t_s, omega_rad_s, u_cmd, all synchronized at MCU if used for identification.
% u_cmd must be the FINAL applied command after clipping/slew/sign conversion.
% options.torquePerCommandUnit: calibrated positive N*m/unit; omit if unknown.
% Closed-loop noise can bias ordinary least squares: validate on another run.
if nargin<2, options = struct; end
if ~isfield(options,'window_s'), options.window_s = 0.05; end
if ~isfield(options,'frictionSmoothRadS'), options.frictionSmoothRadS = 0.05; end
assert(options.window_s>0 && options.frictionSmoothRadS>0, 'Positive options required.');
T = readtable(csvFile,'VariableNamingRule','preserve');
assert(all(ismember({'t_s','omega_rad_s','u_cmd'},T.Properties.VariableNames)), ...
    'Need time, gyro rate and final command.');
t = T.t_s; w = T.omega_rad_s; u = T.u_cmd;
assert(numel(t)>=100 && all(isfinite(t)) && all(diff(t)>0), ...
    'Need at least 100 samples with increasing finite time.');
typicalDt = median(diff(t));
assert(options.window_s>=5*typicalDt, 'Too few samples per integration window.');
X = zeros(0,4); y = zeros(0,1);
i = 1;
while i<numel(t)
    j = find(t>=t(i)+options.window_s,1,'first');
    if isempty(j), break; end
    idx = i:j;
    if all(isfinite(w(idx))) && all(isfinite(u(idx))) && ...
            max(diff(t(idx)))<=3*typicalDt
        X(end+1,:) = [trapz(t(idx),u(idx)), ...
            -trapz(t(idx),w(idx)), ...
            -trapz(t(idx),tanh(w(idx)/options.frictionSmoothRadS)), t(j)-t(i)]; %#ok<AGROW>
        y(end+1,1) = w(j)-w(i); %#ok<AGROW>
    end
    i = j;
end
assert(size(X,1)>=30,'Need >=30 valid windows; collect longer, denser, gap-free data.');
split = floor(0.7*size(X,1));
trainX = X(1:split,:); trainY = y(1:split);
scales = sqrt(mean(trainX.^2,1));
assert(all(scales>1e-12),'Insufficient excitation: a regressor is zero.');
scaledX = trainX./scales;
condition = cond(scaledX);
assert(rank(scaledX)==4 && condition<1e5, ...
    'Insufficient independent excitation; current and speed are too correlated.');
p = (scaledX\trainY)./scales';
assert(p(1)>0,'Input sign disagrees with rate direction. Correct units/signs first.');
assert(p(2)>=0,'Negative damping: inspect synchronization, model or excitation.');
model = struct('gain_rad_s2_per_unit',p(1),'damping_per_s',p(2), ...
    'friction_rad_s2',p(3),'bias_rad_s2',p(4), ...
    'windows',size(X,1),'normalized_condition',condition, ...
    'validation_delta_omega_rmse_rad_s',sqrt(mean((X(split+1:end,:)*p-y(split+1:end)).^2)));
model.note = 'Effective command-to-motion fit. Closed-loop/noise/delay bias needs independent validation.';
if isfield(options,'torquePerCommandUnit')
    kt = options.torquePerCommandUnit;
    assert(isscalar(kt) && isfinite(kt) && kt>0,'Torque conversion must be calibrated and positive.');
    model.J_effective_kg_m2 = kt/p(1);
    model.B_effective_nms_rad = p(2)*model.J_effective_kg_m2;
    model.coulomb_effective_nm = p(3)*model.J_effective_kg_m2;
else
    model.torque_note = 'Torque conversion unknown: no inertia in kg*m^2 can be inferred.';
end
disp(model);
end
