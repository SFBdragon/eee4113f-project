%% sharcbuoy_hydrostatic_weight_sensitivity_ring_5pct_infill.m
% Hydrostatic check for the detachable SharcBuoy communication module.
%
% This is not trying to be a full ocean simulation.
% It answers the design questions we care about:
%
% 1) Does the module float?
% 2) Does the connector seam stay above the waterline?
% 3) Does the ballast improve self-righting?
% 4) Does a wider flotation ring improve the design?
% 5) Where is our current PLA model on those graphs?
% 6) Where would the same geometry sit if the body was HDPE?
% 7) How sensitive is the design to the weight of the unit?
% 8) How wide does the ring need to be as the unit gets heavier?
%
% Coordinate system:
% z goes upward.
% z = 0 is the bottom of the SharcBuoy body.
% The ballast hangs below this, so it goes into negative z.

clear; clc; close all;

%% ---------------- 1. BASIC ASSUMPTIONS ----------------
% Start with the constants.
% Keep these visible because they control the whole model.

inp.g = 9.81;                        % m/s^2

% Near-freezing seawater.
% We use rho directly in the buoyancy calculation.
% Temperature and salinity are kept for report traceability.
inp.fluid.rho = 1027;                % kg/m^3
inp.fluid.tempC = -2;                % degC
inp.fluid.salinity_gkg = 35;         % g/kg

%% ---------------- 2. MATERIAL ASSUMPTIONS ----------------
% PLA is the printed prototype material.
% HDPE is the real SharcBuoy body material.
% Sand is the ballast fill.
%
% Important:
% The PLA print is not treated as solid plastic.
% This is the one knob for the printed PLA parts.
% We are using 5% infill for the current model, so this is 0.05.
% Change it here if the slicer setting changes.
%
% This is a simple equivalent-density model.
% It is not a slicer mass estimate.
% It does not separate wall lines, top layers, bottom layers and infill.
% But for hydrostatics, it gives us the right first-order effect:
% lower infill -> lower printed mass -> more useful buoyancy.

inp.material.PLA.rhoSolid = 1240;        % kg/m^3, solid PLA estimate
inp.material.PLA.infillFraction = 0.05;  % 5% infill for the printed PLA model
inp.material.HDPE.rho = 950;             % kg/m^3, HDPE estimate
inp.material.sand.rhoBulk =2000;        % kg/m^3, bulk sea-sand estimate

% This is the density actually used for printed PLA parts in the model.
% So the PLA ring is no longer a solid ring.
% Same for the PLA body/pod/shell case.
inp.material.PLA.rhoEffective = inp.material.PLA.rhoSolid * ...
    inp.material.PLA.infillFraction;

%% ---------------- 3. DESIGN TARGETS ----------------
% These are the pass/fail checks.
% Keep them simple.
% We only need to prove that the design is above the waterline and stable.

inp.stability.targetHeelDeg = 10;              % deg
inp.stability.minGM_m = 0.000;                 % m, positive GM means initially stable
inp.stability.minConnectorClearance_m = 0.000; % m, connector seam must clear water

% Static release only.
% No drag help.
% If we later do a pull test, put the measured resisting force here.
inp.release.F_measuredRetention = 0;           % N

% Printer-bed reference.
% This is only a visual limit on the plots.
inp.printer.bedXY_m = 0.260;                   % m

%% ---------------- 4. GEOMETRY ----------------
% Build the physical stack from bottom to top:
% ballast -> SharcBuoy body -> flotation ring -> connector pod.
%
% The external volumes create buoyancy.
% The masses and zCG values decide KG.

% Existing SharcBuoy core body.
inp.geom.core.name = "coreBody";
inp.geom.core.z0 = 0.000;            % m
inp.geom.core.z1 = 0.170;            % m, connector plane
inp.geom.core.rOuter = 0.0875;       % m
inp.geom.core.wallThickness = 0.005; % m
inp.geom.core.internalMass = 3.9;    % kg, battery/electronics estimate
inp.geom.core.internalZCG = inp.geom.core.z0 + ...
    0.10*(inp.geom.core.z1 - inp.geom.core.z0);

% Connector pod / radome.
% The seam at z = 0.170 m is the point we care about.
% If that is above water, the pod connection is clear.
inp.geom.pod.name = "connectorPod";
inp.geom.pod.z0 = 0.170;             % m
inp.geom.pod.height = 0.240;         % m
inp.geom.pod.z1 = inp.geom.pod.z0 + inp.geom.pod.height;
inp.geom.pod.rOuter = 0.0875;        % m
inp.geom.pod.wallThickness = 0.002;  % m
inp.geom.pod.internalMass = 0.10;    % kg, antenna/electronics allowance
inp.geom.pod.internalZCG = 0.5*(inp.geom.pod.z0 + inp.geom.pod.z1);

% This is the clearance reference on the plots.
inp.zConnectorRef = 0.170;           % m

% Rectangular flotation ring.
% Inner radius stays fixed.
% Outer radius is what we scale.
% The ring is PLA in both cases, because that is the proposed collar material.
% But it is printed PLA. Not solid PLA.
% So its mass uses the 5% infill density set above.
inp.geom.ring.name = "buoyRing";
inp.geom.ring.z1 = inp.zConnectorRef;           % m, ring top aligns with connector plane
inp.geom.ring.height = 0.065;                   % m
inp.geom.ring.z0 = inp.geom.ring.z1 - inp.geom.ring.height;
inp.geom.ring.rInner = 0.095;                   % m
inp.geom.ring.rOuter = 0.120;                   % m, current model
inp.geom.ring.material = "PLA";
inp.geom.ring.zCG = 0.5*(inp.geom.ring.z0 + inp.geom.ring.z1);

% Ballast tube.
% The tube shape stays fixed.
% We only change how much sand we pour into it.
% Since the sand cavity and tube span the same height, the ballast zCG stays
% at the middle of the tube while its mass changes.
inp.ballast.name = "ballastTube";
inp.ballast.zTop = 0.000;            % m
inp.ballast.length = 0.150;          % m
inp.ballast.outerDiameter = 0.060;   % m
inp.ballast.wallThickness = 0.005;   % m
inp.ballast.baselineFillFraction = 1.00;

%% ---------------- 5. SWEEP SETUP ----------------
% Current ballast is whatever fill fraction we set above.
% Then we sweep from empty to full.
% This answers: what does adding sand actually buy us?

maxBallastFillMass_kg = ballastSandCapacity(inp);
baselineBallastFillMass_kg = inp.ballast.baselineFillFraction * ...
    maxBallastFillMass_kg;

inp.sweep.ballastFillMass_kg = linspace(0, maxBallastFillMass_kg, 31).';

% Current ring width.
% This is the point we mark on the plots.
baselineRingWidth_m = inp.geom.ring.rOuter - inp.geom.ring.rInner;

% Print-bed limit expressed as ring width.
% Outer radius cannot exceed half the bed width.
prototypeMaxRingWidth_m = inp.printer.bedXY_m/2 - inp.geom.ring.rInner;

% Sweep wider than the print-bed limit.
% This lets the same graph show the prototype and possible scale-up.
inp.sweep.ringWidth_m = linspace(0.005, 0.080, 41).';

% Now test the thing that surprised us.
% Make the unit heavier and see when the design stops passing.
% Here, unit mass means the internal mass inside the SharcBuoy body.
currentCoreInternalMass_kg = inp.geom.core.internalMass;
inp.sweep.coreInternalMass_kg = linspace(0.5, 4.0, 36).';

% For each unit mass, search for the ring width that just passes.
% This goes past the printer limit because the real collar can scale.
inp.sweep.ringWidthForMass_m = linspace(0.005, 0.120, 116).';

%% ---------------- 6. EXPORT SETTINGS ----------------

inp.exportCSV = true;
inp.csvPrefix = 'sharcbuoy_hydrostatic_ring_5pct_infill';

%% ---------------- 7. BUILD THE TWO CASES ----------------
% Case 1: PLA prototype.
% This uses the effective PLA density, so infill matters here.
%
% Case 2: HDPE same-geometry model.
% Same dimensions.
% Same PLA flotation ring.
% The body, pod and ballast shell are HDPE.

cases = buildDesignCases(inp);

%% ---------------- 8. CURRENT DESIGN POINT ----------------
% Before sweeping anything, calculate where the current model sits.
% These are the points that get marked on the graphs.

baselineTable = evaluateBaselineCases(cases, inp, ...
    baselineBallastFillMass_kg, baselineRingWidth_m);

fprintf('\n=== BASELINE SUMMARY ===\n');
disp(baselineTable);

%% ---------------- 9. PARAMETER SWEEPS ----------------
% Question A:
% If the ring stays the same, what happens as we add ballast sand?

ballastSweepTable = sweepBallastFillMass(cases, inp, ...
    inp.sweep.ballastFillMass_kg, baselineRingWidth_m);

% Question B:
% If the ballast stays the same, what happens as we make the ring wider?

ringSweepTable = sweepRingWidth(cases, inp, ...
    inp.sweep.ringWidth_m, baselineBallastFillMass_kg);

% Question C:
% Where is the safe region if both ballast mass and ring width change?

designMapTable = sweepBallastAndRingWidth(cases, inp, ...
    inp.sweep.ballastFillMass_kg, inp.sweep.ringWidth_m);

% Question D:
% What happens when the unit gets heavier?
% Keep the current ring and ballast. Only change the internal body mass.

unitMassSweepTable = sweepCoreInternalMass(cases, inp, ...
    inp.sweep.coreInternalMass_kg, baselineBallastFillMass_kg, baselineRingWidth_m);

% Question E:
% As the unit gets heavier, what ring width is needed to pass?

requiredRingWidthTable = findRequiredRingWidthVsCoreMass(cases, inp, ...
    inp.sweep.coreInternalMass_kg, inp.sweep.ringWidthForMass_m, ...
    baselineBallastFillMass_kg);

%% ---------------- 10. PLOTS ----------------
% Keep the plots report-useful.
% Trends are useful, but the current design point is the proof anchor.

caseNames = unique(ballastSweepTable.caseName, 'stable');

% Plot 1: restoring moment vs ballast fill mass.
figure('Name','Restoring moment vs ballast fill mass');
hold on;

for i = 1:numel(caseNames)
    idx = ballastSweepTable.caseName == caseNames(i);
    plot(ballastSweepTable.ballastFillMass_g(idx), ...
        ballastSweepTable.restoringMomentTarget_Nm(idx), ...
        'LineWidth', 1.8, ...
        'DisplayName', caseNames(i));
end

markBaselinePoints(baselineTable, ...
    "ballastFillMass_g", "restoringMomentTarget_Nm", true);

yline(0, '--', 'Zero restoring moment', 'HandleVisibility','off');
grid on;
xlabel('Sand fill mass in ballast [g]');
ylabel('Restoring moment at 10 deg heel [N.m]');
title('Restoring moment as ballast is added');
legend('Location', 'best');

% Plot 2: GM vs ballast fill mass.
figure('Name','GM vs ballast fill mass');
hold on;

for i = 1:numel(caseNames)
    idx = ballastSweepTable.caseName == caseNames(i);
    plot(ballastSweepTable.ballastFillMass_g(idx), ...
        ballastSweepTable.GM_mm(idx), ...
        'LineWidth', 1.8, ...
        'DisplayName', caseNames(i));
end

markBaselinePoints(baselineTable, ...
    "ballastFillMass_g", "GM_mm", true);

yline(0, '--', 'Neutral stability', 'HandleVisibility','off');
grid on;
xlabel('Sand fill mass in ballast [g]');
ylabel('GM [mm]');
title('Metacentric height as ballast is added');
legend('Location', 'best');

% Plot 3: connector seam clearance vs ring width.
caseNames = unique(ringSweepTable.caseName, 'stable');

figure('Name','Connector seam clearance vs ring width');
hold on;

for i = 1:numel(caseNames)
    idx = ringSweepTable.caseName == caseNames(i);
    plot(ringSweepTable.ringWidth_mm(idx), ...
        ringSweepTable.connectorClearance_mm(idx), ...
        'LineWidth', 1.8, ...
        'DisplayName', caseNames(i));
end

markBaselinePoints(baselineTable, ...
    "ringWidth_mm", "connectorClearance_mm", true);

yline(0, '--', 'Connection plane reaches waterline', 'HandleVisibility','off');
xline(1e3*prototypeMaxRingWidth_m, '--', ...
    '260 mm printer-bed limit', 'HandleVisibility','off');
grid on;
xlabel('Flotation ring radial width [mm]');
ylabel('Clearance from waterline to connector seam [mm]');
title('Connector seam clearance as ring width increases');
legend('Location', 'best');

% Plot 4: net static uplift vs ring width.
figure('Name','Static uplift vs ring width');
hold on;

for i = 1:numel(caseNames)
    idx = ringSweepTable.caseName == caseNames(i);
    plot(ringSweepTable.ringWidth_mm(idx), ...
        ringSweepTable.netUpliftFullySubmerged_N(idx), ...
        'LineWidth', 1.8, ...
        'DisplayName', caseNames(i));
end

markBaselinePoints(baselineTable, ...
    "ringWidth_mm", "netUpliftFullySubmerged_N", true);

yline(0, '--', 'Neutral fully-submerged uplift', 'HandleVisibility','off');
xline(1e3*prototypeMaxRingWidth_m, '--', ...
    '260 mm printer-bed limit', 'HandleVisibility','off');
grid on;
xlabel('Flotation ring radial width [mm]');
ylabel('Net uplift when fully submerged [N]');
title('Static release uplift as ring width increases');
legend('Location', 'best');

% Plot 5: pass/fail design maps.
% This is the useful design-region plot.
% It shows whether each ballast + ring combination passes the screen.
figure('Name','Design pass-fail maps');
caseNames = unique(designMapTable.caseName, 'stable');

for i = 1:numel(caseNames)
    subplot(1, numel(caseNames), i);

    idx = designMapTable.caseName == caseNames(i);
    scatter(designMapTable.ringWidth_mm(idx), ...
        designMapTable.ballastFillMass_g(idx), ...
        28, double(designMapTable.passesScreen(idx)), 'filled');
    hold on;

    xline(1e3*prototypeMaxRingWidth_m, '--', '260 mm printer limit');

    idxBase = baselineTable.caseName == caseNames(i);
    plot(baselineTable.ringWidth_mm(idxBase), ...
        baselineTable.ballastFillMass_g(idxBase), ...
        'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y', ...
        'LineWidth', 1.4);
    text(baselineTable.ringWidth_mm(idxBase), ...
        baselineTable.ballastFillMass_g(idxBase), ...
        "  Current design", ...
        'FontSize', 8, 'VerticalAlignment', 'bottom');

    grid on;
    xlabel('Ring radial width [mm]');
    ylabel('Sand fill mass [g]');
    title(caseNames(i) + " pass/fail map");

    cb = colorbar;
    cb.Label.String = '0 = fail, 1 = pass';
    caxis([0 1]);
end

% Plot 6: current ring and ballast, but increasing unit mass.
% This explains why changing the 2.5 kg estimate changed everything.
figure('Name','Current design margins vs unit mass');
tiledlayout(3,1);
caseNames = unique(unitMassSweepTable.caseName, 'stable');

nexttile;
hold on;
for i = 1:numel(caseNames)
    idx = unitMassSweepTable.caseName == caseNames(i);
    plot(unitMassSweepTable.coreInternalMass_kg(idx), ...
        unitMassSweepTable.GM_mm(idx), ...
        'LineWidth', 1.8, ...
        'DisplayName', caseNames(i));
end
yline(0, '--', 'Neutral stability', 'HandleVisibility','off');
xline(currentCoreInternalMass_kg, '--', 'Current mass', 'HandleVisibility','off');
grid on;
xlabel('Internal unit mass [kg]');
ylabel('GM [mm]');
title('Self-righting margin as unit mass increases');
legend('Location','best');

nexttile;
hold on;
for i = 1:numel(caseNames)
    idx = unitMassSweepTable.caseName == caseNames(i);
    plot(unitMassSweepTable.coreInternalMass_kg(idx), ...
        unitMassSweepTable.connectorClearance_mm(idx), ...
        'LineWidth', 1.8, ...
        'DisplayName', caseNames(i));
end
yline(0, '--', 'Seam reaches waterline', 'HandleVisibility','off');
xline(currentCoreInternalMass_kg, '--', 'Current mass', 'HandleVisibility','off');
grid on;
xlabel('Internal unit mass [kg]');
ylabel('Connector clearance [mm]');
title('Connector seam clearance as unit mass increases');
legend('Location','best');

nexttile;
hold on;
for i = 1:numel(caseNames)
    idx = unitMassSweepTable.caseName == caseNames(i);
    plot(unitMassSweepTable.coreInternalMass_kg(idx), ...
        unitMassSweepTable.netUpliftFullySubmerged_N(idx), ...
        'LineWidth', 1.8, ...
        'DisplayName', caseNames(i));
end
yline(0, '--', 'Neutral submerged uplift', 'HandleVisibility','off');
xline(currentCoreInternalMass_kg, '--', 'Current mass', 'HandleVisibility','off');
grid on;
xlabel('Internal unit mass [kg]');
ylabel('Net uplift [N]');
title('Static release margin trend as unit mass increases');
legend('Location','best');

% Plot 7: required ring width as the unit gets heavier.
% This is the sizing graph.
% It says how much ring is needed to keep passing.
figure('Name','Required ring width vs unit mass');
caseNames = unique(requiredRingWidthTable.caseName, 'stable');
tiledlayout(1, numel(caseNames));

for i = 1:numel(caseNames)
    nexttile;
    idx = requiredRingWidthTable.caseName == caseNames(i);

    hold on;
    plot(requiredRingWidthTable.coreInternalMass_kg(idx), ...
        requiredRingWidthTable.minRingWidthForGM_mm(idx), ...
        '--', 'LineWidth', 1.5, ...
        'DisplayName', 'GM only');
    plot(requiredRingWidthTable.coreInternalMass_kg(idx), ...
        requiredRingWidthTable.minRingWidthForConnector_mm(idx), ...
        ':', 'LineWidth', 1.8, ...
        'DisplayName', 'Connector clearance only');
    plot(requiredRingWidthTable.coreInternalMass_kg(idx), ...
        requiredRingWidthTable.minRingWidthForAllChecks_mm(idx), ...
        '-', 'LineWidth', 2.0, ...
        'DisplayName', 'All checks');

    yline(1e3*baselineRingWidth_m, '--', ...
        'Current ring width', 'HandleVisibility','off');
    yline(1e3*prototypeMaxRingWidth_m, '--', ...
        '260 mm printer limit', 'HandleVisibility','off');
    xline(currentCoreInternalMass_kg, '--', ...
        'Current mass', 'HandleVisibility','off');

    grid on;
    xlabel('Internal unit mass [kg]');
    ylabel('Required ring radial width [mm]');
    title(caseNames(i));
    legend('Location','best');
end

%% ---------------- 11. CSV EXPORT ----------------

if inp.exportCSV
    writetable(baselineTable, [inp.csvPrefix '_baseline_summary.csv']);
    writetable(ballastSweepTable, [inp.csvPrefix '_ballast_fill_sweep.csv']);
    writetable(ringSweepTable, [inp.csvPrefix '_ring_width_sweep.csv']);
    writetable(designMapTable, [inp.csvPrefix '_design_pass_fail_map.csv']);
    writetable(unitMassSweepTable, [inp.csvPrefix '_unit_mass_sweep.csv']);
    writetable(requiredRingWidthTable, [inp.csvPrefix '_required_ring_width_vs_unit_mass.csv']);

    fprintf('\nCSV files written with prefix: %s\n', inp.csvPrefix);
end

%% =============== LOCAL FUNCTIONS ===============

function cases = buildDesignCases(inp)
    % Build the two material cases.
    % PLA gets the infill correction.
    % HDPE does not.

    cases(1).name = "PLA prototype";
    cases(1).bodyMaterial = "PLA";
    cases(1).bodyDensity = inp.material.PLA.rhoEffective;

    cases(2).name = "HDPE SharcBuoy case";
    cases(2).bodyMaterial = "HDPE";
    cases(2).bodyDensity = inp.material.HDPE.rho;
end

function T = evaluateBaselineCases(cases, inp, ballastFillMass_kg, ringWidth_m)
    % Calculate the current model for each case.
    % This table is what we use for the highlighted plot markers.

    rows = numel(cases);

    caseName = strings(rows,1);
    bodyMaterial = strings(rows,1);
    plaInfill_percent = zeros(rows,1);
    bodyDensity_kgm3 = zeros(rows,1);
    coreInternalMass_kg = zeros(rows,1);
    ballastFillMass_g = zeros(rows,1);
    ringWidth_mm = zeros(rows,1);
    totalMass_kg = zeros(rows,1);
    waterlineZ_mm = zeros(rows,1);
    connectorClearance_mm = zeros(rows,1);
    KG_mm = zeros(rows,1);
    KB_mm = zeros(rows,1);
    BM_mm = zeros(rows,1);
    GM_mm = zeros(rows,1);
    restoringMomentTarget_Nm = zeros(rows,1);
    netUpliftFullySubmerged_N = zeros(rows,1);
    staticReleaseMargin_N = zeros(rows,1);
    passesScreen = false(rows,1);

    for i = 1:rows
        seg = buildSegmentsForCase(inp, cases(i), ballastFillMass_kg, ringWidth_m);
        out = evaluateDesign(seg, inp);

        caseName(i) = cases(i).name;
        bodyMaterial(i) = cases(i).bodyMaterial;
        plaInfill_percent(i) = 100*inp.material.PLA.infillFraction;
        bodyDensity_kgm3(i) = cases(i).bodyDensity;
        coreInternalMass_kg(i) = inp.geom.core.internalMass;
        ballastFillMass_g(i) = 1e3*ballastFillMass_kg;
        ringWidth_mm(i) = 1e3*ringWidth_m;
        totalMass_kg(i) = out.totalMass_kg;
        waterlineZ_mm(i) = 1e3*out.waterlineZ_m;
        connectorClearance_mm(i) = 1e3*out.connectorClearance_m;
        KG_mm(i) = 1e3*out.KG_m;
        KB_mm(i) = 1e3*out.KB_m;
        BM_mm(i) = 1e3*out.BM_m;
        GM_mm(i) = 1e3*out.GM_m;
        restoringMomentTarget_Nm(i) = out.restoringMomentTarget_Nm;
        netUpliftFullySubmerged_N(i) = out.netUpliftFullySubmerged_N;
        staticReleaseMargin_N(i) = out.staticReleaseMargin_N;
        passesScreen(i) = out.passesScreen;
    end

    T = table(caseName, bodyMaterial, plaInfill_percent, bodyDensity_kgm3, ...
        coreInternalMass_kg, ballastFillMass_g, ringWidth_mm, totalMass_kg, waterlineZ_mm, ...
        connectorClearance_mm, KG_mm, KB_mm, BM_mm, GM_mm, ...
        restoringMomentTarget_Nm, netUpliftFullySubmerged_N, ...
        staticReleaseMargin_N, passesScreen);
end

function T = sweepBallastFillMass(cases, inp, ballastFillMassSweep_kg, ringWidth_m)
    % Keep the ring fixed.
    % Fill the ballast from empty to full.

    rows = numel(cases)*numel(ballastFillMassSweep_kg);

    caseName = strings(rows,1);
    bodyMaterial = strings(rows,1);
    ballastFillMass_g = zeros(rows,1);
    ballastTotalMass_kg = zeros(rows,1);
    totalMass_kg = zeros(rows,1);
    connectorClearance_mm = zeros(rows,1);
    GM_mm = zeros(rows,1);
    restoringMomentTarget_Nm = zeros(rows,1);
    netUpliftFullySubmerged_N = zeros(rows,1);
    staticReleaseMargin_N = zeros(rows,1);
    passesScreen = false(rows,1);

    row = 0;

    for c = 1:numel(cases)
        for k = 1:numel(ballastFillMassSweep_kg)
            row = row + 1;

            seg = buildSegmentsForCase(inp, cases(c), ...
                ballastFillMassSweep_kg(k), ringWidth_m);
            out = evaluateDesign(seg, inp);

            idxBallast = getSegmentIndex(seg, inp.ballast.name);

            caseName(row) = cases(c).name;
            bodyMaterial(row) = cases(c).bodyMaterial;
            ballastFillMass_g(row) = 1e3*ballastFillMassSweep_kg(k);
            ballastTotalMass_kg(row) = seg(idxBallast).mass;
            totalMass_kg(row) = out.totalMass_kg;
            connectorClearance_mm(row) = 1e3*out.connectorClearance_m;
            GM_mm(row) = 1e3*out.GM_m;
            restoringMomentTarget_Nm(row) = out.restoringMomentTarget_Nm;
            netUpliftFullySubmerged_N(row) = out.netUpliftFullySubmerged_N;
            staticReleaseMargin_N(row) = out.staticReleaseMargin_N;
            passesScreen(row) = out.passesScreen;
        end
    end

    T = table(caseName, bodyMaterial, ballastFillMass_g, ...
        ballastTotalMass_kg, totalMass_kg, connectorClearance_mm, ...
        GM_mm, restoringMomentTarget_Nm, netUpliftFullySubmerged_N, ...
        staticReleaseMargin_N, passesScreen);
end

function T = sweepRingWidth(cases, inp, ringWidthSweep_m, ballastFillMass_kg)
    % Keep the ballast fixed.
    % Make the ring wider by increasing outer radius only.

    rows = numel(cases)*numel(ringWidthSweep_m);

    caseName = strings(rows,1);
    bodyMaterial = strings(rows,1);
    ringWidth_mm = zeros(rows,1);
    ringOuterRadius_mm = zeros(rows,1);
    ringMass_kg = zeros(rows,1);
    totalMass_kg = zeros(rows,1);
    connectorClearance_mm = zeros(rows,1);
    GM_mm = zeros(rows,1);
    netUpliftFullySubmerged_N = zeros(rows,1);
    staticReleaseMargin_N = zeros(rows,1);
    passesScreen = false(rows,1);

    row = 0;

    for c = 1:numel(cases)
        for k = 1:numel(ringWidthSweep_m)
            row = row + 1;

            seg = buildSegmentsForCase(inp, cases(c), ...
                ballastFillMass_kg, ringWidthSweep_m(k));
            out = evaluateDesign(seg, inp);

            idxRing = getSegmentIndex(seg, inp.geom.ring.name);

            caseName(row) = cases(c).name;
            bodyMaterial(row) = cases(c).bodyMaterial;
            ringWidth_mm(row) = 1e3*ringWidthSweep_m(k);
            ringOuterRadius_mm(row) = 1e3*seg(idxRing).rOuter;
            ringMass_kg(row) = seg(idxRing).mass;
            totalMass_kg(row) = out.totalMass_kg;
            connectorClearance_mm(row) = 1e3*out.connectorClearance_m;
            GM_mm(row) = 1e3*out.GM_m;
            netUpliftFullySubmerged_N(row) = out.netUpliftFullySubmerged_N;
            staticReleaseMargin_N(row) = out.staticReleaseMargin_N;
            passesScreen(row) = out.passesScreen;
        end
    end

    T = table(caseName, bodyMaterial, ringWidth_mm, ringOuterRadius_mm, ...
        ringMass_kg, totalMass_kg, connectorClearance_mm, GM_mm, ...
        netUpliftFullySubmerged_N, staticReleaseMargin_N, passesScreen);
end

function T = sweepBallastAndRingWidth(cases, inp, ballastFillMassSweep_kg, ringWidthSweep_m)
    % This is the design-region sweep.
    % It asks: which combinations pass all checks?

    rows = numel(cases)*numel(ballastFillMassSweep_kg)*numel(ringWidthSweep_m);

    caseName = strings(rows,1);
    bodyMaterial = strings(rows,1);
    ballastFillMass_g = zeros(rows,1);
    ringWidth_mm = zeros(rows,1);
    connectorClearance_mm = zeros(rows,1);
    GM_mm = zeros(rows,1);
    netUpliftFullySubmerged_N = zeros(rows,1);
    passesScreen = false(rows,1);

    row = 0;

    for c = 1:numel(cases)
        for b = 1:numel(ballastFillMassSweep_kg)
            for r = 1:numel(ringWidthSweep_m)
                row = row + 1;

                seg = buildSegmentsForCase(inp, cases(c), ...
                    ballastFillMassSweep_kg(b), ringWidthSweep_m(r));
                out = evaluateDesign(seg, inp);

                caseName(row) = cases(c).name;
                bodyMaterial(row) = cases(c).bodyMaterial;
                ballastFillMass_g(row) = 1e3*ballastFillMassSweep_kg(b);
                ringWidth_mm(row) = 1e3*ringWidthSweep_m(r);
                connectorClearance_mm(row) = 1e3*out.connectorClearance_m;
                GM_mm(row) = 1e3*out.GM_m;
                netUpliftFullySubmerged_N(row) = out.netUpliftFullySubmerged_N;
                passesScreen(row) = out.passesScreen;
            end
        end
    end

    T = table(caseName, bodyMaterial, ballastFillMass_g, ringWidth_mm, ...
        connectorClearance_mm, GM_mm, netUpliftFullySubmerged_N, passesScreen);
end

function T = sweepCoreInternalMass(cases, inp, coreInternalMassSweep_kg, ballastFillMass_kg, ringWidth_m)
    % Keep the current ring and ballast.
    % Make the unit heavier.
    % This shows why the 2.5 kg estimate matters.

    rows = numel(cases)*numel(coreInternalMassSweep_kg);

    caseName = strings(rows,1);
    bodyMaterial = strings(rows,1);
    coreInternalMass_kg = zeros(rows,1);
    totalMass_kg = zeros(rows,1);
    connectorClearance_mm = zeros(rows,1);
    GM_mm = zeros(rows,1);
    netUpliftFullySubmerged_N = zeros(rows,1);
    staticReleaseMargin_N = zeros(rows,1);
    passesScreen = false(rows,1);

    row = 0;

    for c = 1:numel(cases)
        for k = 1:numel(coreInternalMassSweep_kg)
            row = row + 1;

            inpTmp = inp;
            inpTmp.geom.core.internalMass = coreInternalMassSweep_kg(k);

            seg = buildSegmentsForCase(inpTmp, cases(c), ...
                ballastFillMass_kg, ringWidth_m);
            out = evaluateDesign(seg, inpTmp);

            caseName(row) = cases(c).name;
            bodyMaterial(row) = cases(c).bodyMaterial;
            coreInternalMass_kg(row) = coreInternalMassSweep_kg(k);
            totalMass_kg(row) = out.totalMass_kg;
            connectorClearance_mm(row) = 1e3*out.connectorClearance_m;
            GM_mm(row) = 1e3*out.GM_m;
            netUpliftFullySubmerged_N(row) = out.netUpliftFullySubmerged_N;
            staticReleaseMargin_N(row) = out.staticReleaseMargin_N;
            passesScreen(row) = out.passesScreen;
        end
    end

    T = table(caseName, bodyMaterial, coreInternalMass_kg, totalMass_kg, ...
        connectorClearance_mm, GM_mm, netUpliftFullySubmerged_N, ...
        staticReleaseMargin_N, passesScreen);
end

function T = findRequiredRingWidthVsCoreMass(cases, inp, coreInternalMassSweep_kg, ringWidthSweep_m, ballastFillMass_kg)
    % For each unit mass, search ring widths from small to large.
    % Save the first width that passes each check.
    % NaN means even the largest swept ring was not enough.

    rows = numel(cases)*numel(coreInternalMassSweep_kg);

    caseName = strings(rows,1);
    bodyMaterial = strings(rows,1);
    coreInternalMass_kg = zeros(rows,1);
    minRingWidthForGM_mm = NaN(rows,1);
    minRingWidthForConnector_mm = NaN(rows,1);
    minRingWidthForStaticUplift_mm = NaN(rows,1);
    minRingWidthForAllChecks_mm = NaN(rows,1);
    limitingCheck = strings(rows,1);

    row = 0;

    for c = 1:numel(cases)
        for m = 1:numel(coreInternalMassSweep_kg)
            row = row + 1;

            inpTmp = inp;
            inpTmp.geom.core.internalMass = coreInternalMassSweep_kg(m);

            passGM = false(numel(ringWidthSweep_m),1);
            passConnector = false(numel(ringWidthSweep_m),1);
            passStatic = false(numel(ringWidthSweep_m),1);
            passAll = false(numel(ringWidthSweep_m),1);

            for r = 1:numel(ringWidthSweep_m)
                seg = buildSegmentsForCase(inpTmp, cases(c), ...
                    ballastFillMass_kg, ringWidthSweep_m(r));
                out = evaluateDesign(seg, inpTmp);

                passGM(r) = out.passesGM;
                passConnector(r) = out.passesConnectorClearance;
                passStatic(r) = out.passesStaticRelease;
                passAll(r) = out.passesScreen;
            end

            caseName(row) = cases(c).name;
            bodyMaterial(row) = cases(c).bodyMaterial;
            coreInternalMass_kg(row) = coreInternalMassSweep_kg(m);

            minRingWidthForGM_mm(row) = firstPassingWidth_mm(ringWidthSweep_m, passGM);
            minRingWidthForConnector_mm(row) = firstPassingWidth_mm(ringWidthSweep_m, passConnector);
            minRingWidthForStaticUplift_mm(row) = firstPassingWidth_mm(ringWidthSweep_m, passStatic);
            minRingWidthForAllChecks_mm(row) = firstPassingWidth_mm(ringWidthSweep_m, passAll);

            limitingCheck(row) = findLimitingCheck( ...
                minRingWidthForGM_mm(row), ...
                minRingWidthForConnector_mm(row), ...
                minRingWidthForStaticUplift_mm(row), ...
                minRingWidthForAllChecks_mm(row));
        end
    end

    T = table(caseName, bodyMaterial, coreInternalMass_kg, ...
        minRingWidthForGM_mm, minRingWidthForConnector_mm, ...
        minRingWidthForStaticUplift_mm, minRingWidthForAllChecks_mm, ...
        limitingCheck);
end

function w_mm = firstPassingWidth_mm(ringWidthSweep_m, passVector)
    % First width that passes.

    idx = find(passVector, 1, 'first');

    if isempty(idx)
        w_mm = NaN;
    else
        w_mm = 1e3*ringWidthSweep_m(idx);
    end
end

function label = findLimitingCheck(wGM_mm, wConnector_mm, wStatic_mm, wAll_mm)
    % Which check forces the largest ring width?

    if isnan(wAll_mm)
        label = "No passing width in sweep";
        return;
    end

    widths = [wGM_mm, wConnector_mm, wStatic_mm];
    names = ["GM", "connector clearance", "static uplift"];

    [~, idx] = max(widths);
    label = names(idx);
end

function seg = buildSegmentsForCase(inp, designCase, ballastFillMass_kg, ringWidth_m)
    % Put the module together.
    % Each part becomes one segment with:
    % external volume, mass, and zCG.

    bodyRho = designCase.bodyDensity;

    core = makeShellCylinderSegment( ...
        inp.geom.core.name, ...
        inp.geom.core.z0, inp.geom.core.z1, ...
        inp.geom.core.rOuter, inp.geom.core.wallThickness, ...
        bodyRho, inp.geom.core.internalMass, inp.geom.core.internalZCG);

    pod = makeShellCylinderSegment( ...
        inp.geom.pod.name, ...
        inp.geom.pod.z0, inp.geom.pod.z1, ...
        inp.geom.pod.rOuter, inp.geom.pod.wallThickness, ...
        bodyRho, inp.geom.pod.internalMass, inp.geom.pod.internalZCG);

    ring = makeRingSegment(inp, ringWidth_m);

    ballast = makeBallastSegment(inp, bodyRho, ballastFillMass_kg);

    seg = [ballast, core, ring, pod];
end

function s = makeShellCylinderSegment(name, z0, z1, rOuter, wallThickness, materialRho, internalMass, internalZCG)
    % Shell mass plus whatever is inside it.
    % Buoyancy still uses the outside shape.
    % That is the important separation.

    height = z1 - z0;
    shellVolume = closedCylinderShellVolume(rOuter, height, wallThickness);
    shellMass = materialRho * shellVolume;
    shellZCG = 0.5*(z0 + z1);

    mass = shellMass + internalMass;

    if mass > 0
        zCG = (shellMass*shellZCG + internalMass*internalZCG)/mass;
    else
        zCG = shellZCG;
    end

    s = makeSeg(name, z0, z1, rOuter, 0, mass, zCG);
end

function s = makeRingSegment(inp, ringWidth_m)
    % Rectangular PLA collar.
    % Wider ring means more displaced volume.
    % It also means more printed mass.
    % But not solid-PLA mass.
    % This is the important correction.
    % The ring mass uses the 5% infill effective density.

    rInner = inp.geom.ring.rInner;
    rOuter = rInner + ringWidth_m;

    if rOuter <= rInner
        error('Ring outer radius must be larger than ring inner radius.');
    end

    height = inp.geom.ring.height;
    externalVolume = pi*(rOuter^2 - rInner^2)*height;

    % Printed ring mass.
    % Not solid PLA.
    % Effective density already includes the infill fraction.
    mass = inp.material.PLA.rhoEffective * externalVolume;

    s = makeSeg(inp.geom.ring.name, ...
        inp.geom.ring.z0, inp.geom.ring.z1, ...
        rOuter, rInner, mass, inp.geom.ring.zCG);
end

function s = makeBallastSegment(inp, shellMaterialRho, ballastFillMass_kg)
    % Fixed tube.
    % Variable sand.
    % The tube zCG does not move because the fill cavity is centred in the tube.

    L = inp.ballast.length;
    rOuter = inp.ballast.outerDiameter/2;
    wall = inp.ballast.wallThickness;

    z1 = inp.ballast.zTop;
    z0 = inp.ballast.zTop - L;
    zCG = 0.5*(z0 + z1);

    maxFillMass = ballastSandCapacity(inp);

    if ballastFillMass_kg < -1e-12 || ballastFillMass_kg > maxFillMass + 1e-12
        error('Ballast fill mass %.4f kg is outside the physical range 0 to %.4f kg.', ...
            ballastFillMass_kg, maxFillMass);
    end

    shellVolume = closedCylinderShellVolume(rOuter, L, wall);
    shellMass = shellMaterialRho * shellVolume;

    totalMass = shellMass + ballastFillMass_kg;

    s = makeSeg(inp.ballast.name, z0, z1, rOuter, 0, totalMass, zCG);
end

function s = makeSeg(name, z0, z1, rOuter, rInner, mass, zCentroid)
    % One part of the buoy.
    % Keep it boring.
    % Bad geometry should fail early.

    if z1 <= z0
        error('Segment "%s" has z1 <= z0. Check its height.', string(name));
    end

    if rOuter < rInner
        error('Segment "%s" has rOuter < rInner. Check the radii.', string(name));
    end

    if mass < 0
        error('Segment "%s" has negative mass. Check the mass input.', string(name));
    end

    s.name = string(name);
    s.z0 = z0;
    s.z1 = z1;
    s.rOuter = rOuter;
    s.rInner = rInner;
    s.mass = mass;
    s.zCentroid = zCentroid;
end

function out = evaluateDesign(seg, inp)
    % This is the hydrostatic engine.
    %
    % First we find mass and KG.
    % Then we find how much water must be displaced.
    % Then we solve for the waterline.
    % Then we calculate KB, BM and GM.
    %
    % GM is the stability check.
    % Connector clearance is the usefulness check.
    % Static uplift is the release check.

    rho = inp.fluid.rho;
    g = inp.g;

    %% Mass and centre of gravity

    mTot = sum([seg.mass]);
    KG = sum([seg.mass].*[seg.zCentroid]) / mTot;

    %% Full displaced volume

    Vfull = 0;

    for k = 1:numel(seg)
        segmentHeight = seg(k).z1 - seg(k).z0;
        segmentArea = areaOfSeg(seg(k));
        Vfull = Vfull + segmentArea*segmentHeight;
    end

    Vreq = mTot / rho;

    %% Static release budget

    FBfull = rho*g*Vfull;
    W = mTot*g;

    netUpliftFull = FBfull - W;
    staticReleaseMargin = netUpliftFull - inp.release.F_measuredRetention;

    %% Floating waterline

    out.isPositivelyBuoyant = Vreq <= Vfull;

    if out.isPositivelyBuoyant
        zMin = min([seg.z0]) - 1e-3;
        zMax = max([seg.z1]) + 1e-3;

        f = @(zw) submergedVolumeAtWaterline(seg, zw) - Vreq;
        zW = fzero(f, [zMin, zMax]);

        [Vsub, M1, Awp, Iwp] = submergedProps(seg, zW);

        KB = M1 / Vsub;
        BM = Iwp / Vsub;
        GM = KB + BM - KG;

        connectorClearance = inp.zConnectorRef - zW;
    else
        zW = NaN;
        Vsub = NaN;
        Awp = NaN;
        Iwp = NaN;
        KB = NaN;
        BM = NaN;
        GM = NaN;
        connectorClearance = NaN;
    end

    %% Restoring moment and checks

    thetaTarget = deg2rad(inp.stability.targetHeelDeg);
    restoringMomentTarget = W * GM * sin(thetaTarget);

    passesGM = out.isPositivelyBuoyant && GM >= inp.stability.minGM_m;
    passesConnectorClearance = out.isPositivelyBuoyant && ...
        connectorClearance >= inp.stability.minConnectorClearance_m;
    passesStaticRelease = staticReleaseMargin > 0;

    out.totalMass_kg = mTot;
    out.fullDisplacedVolume_m3 = Vfull;
    out.requiredFloatVolume_m3 = Vreq;

    out.waterlineZ_m = zW;
    out.connectorClearance_m = connectorClearance;

    out.KG_m = KG;
    out.KB_m = KB;
    out.BM_m = BM;
    out.GM_m = GM;

    out.waterplaneArea_m2 = Awp;
    out.waterplaneAreaMoment_m4 = Iwp;

    out.weight_N = W;
    out.FBfull_N = FBfull;
    out.netUpliftFullySubmerged_N = netUpliftFull;
    out.staticReleaseMargin_N = staticReleaseMargin;

    out.restoringMomentTarget_Nm = restoringMomentTarget;

    out.passesGM = passesGM;
    out.passesConnectorClearance = passesConnectorClearance;
    out.passesStaticRelease = passesStaticRelease;
    out.passesScreen = passesGM && passesConnectorClearance && passesStaticRelease;
end

function A = areaOfSeg(s)
    % Horizontal area seen by the waterline.
    A = pi*(s.rOuter^2 - s.rInner^2);
end

function V = submergedVolumeAtWaterline(seg, zW)
    % Small wrapper so fzero stays readable.
    [V, ~, ~, ~] = submergedProps(seg, zW);
end

function [Vsub, M1, Awp, Iwp] = submergedProps(seg, zW)
    % Volume below the waterline.
    % Also gets the first moment and the waterplane inertia.
    %
    % Vsub gives buoyancy.
    % M1/Vsub gives KB.
    % Iwp/Vsub gives BM.

    Vsub = 0;
    M1 = 0;
    Awp = 0;
    Iwp = 0;

    for k = 1:numel(seg)
        z0 = seg(k).z0;
        z1 = seg(k).z1;
        A = areaOfSeg(seg(k));

        if zW <= z0
            hSub = 0;
        elseif zW >= z1
            hSub = z1 - z0;
        else
            hSub = zW - z0;
        end

        if hSub > 0
            V = A*hSub;
            zc = z0 + 0.5*hSub;

            Vsub = Vsub + V;
            M1 = M1 + V*zc;
        end

        if (zW > z0) && (zW < z1)
            Awp = Awp + A;
            Iwp = Iwp + (pi/4)*(seg(k).rOuter^4 - seg(k).rInner^4);
        end
    end
end

function V = closedCylinderShellVolume(rOuter, height, wallThickness)
    % A simple closed tube/cylinder shell volume.
    % We subtract the hollow inside from the outside.
    %
    % For PLA, this volume is multiplied by the effective PLA density.
    % So infill is handled by density, not by changing geometry.

    if wallThickness <= 0
        error('Wall thickness must be positive.');
    end

    if wallThickness >= rOuter
        error('Wall thickness is larger than the cylinder radius.');
    end

    if 2*wallThickness >= height
        error('Wall thickness is too large for the cylinder height.');
    end

    rInner = rOuter - wallThickness;
    hInner = height - 2*wallThickness;

    Vouter = pi*rOuter^2*height;
    Vinner = pi*rInner^2*hInner;

    V = Vouter - Vinner;
end

function V = ballastCavityVolume(inp)
    % Space available for sand inside the ballast tube.

    rOuter = inp.ballast.outerDiameter/2;
    wall = inp.ballast.wallThickness;
    L = inp.ballast.length;

    if wall >= rOuter
        error('Ballast wall thickness is larger than the outer radius.');
    end

    if 2*wall >= L
        error('Ballast wall thickness is too large for the ballast length.');
    end

    rCavity = rOuter - wall;
    hCavity = L - 2*wall;

    V = pi*rCavity^2*hCavity;
end

function m = ballastSandCapacity(inp)
    % Maximum sand mass if the ballast cavity is full.
    m = inp.material.sand.rhoBulk * ballastCavityVolume(inp);
end

function markBaselinePoints(baselineTable, xField, yField, addLabels)
    % Mark the current model on a trend plot.
    % Same marker each time.
    % Easy to see.

    xVals = baselineTable.(char(xField));
    yVals = baselineTable.(char(yField));

    for b = 1:height(baselineTable)
        x = xVals(b);
        y = yVals(b);

        plot(x, y, ...
            'kp', 'MarkerSize', 11, 'MarkerFaceColor', 'y', ...
            'LineWidth', 1.4, ...
            'DisplayName', baselinePlotLabel(baselineTable.caseName(b)));

        if addLabels
            text(x, y, ...
                "  " + baselinePlotLabel(baselineTable.caseName(b)), ...
                'FontSize', 8, 'VerticalAlignment', 'bottom');
        end
    end
end

function label = baselinePlotLabel(caseName)
    % Short labels for the current-state markers.

    if string(caseName) == "PLA prototype"
        label = "Current PLA model";
    elseif string(caseName) == "HDPE SharcBuoy case"
        label = "HDPE same-geometry model";
    else
        label = string(caseName) + " baseline";
    end
end

function idx = getSegmentIndex(seg, targetName)
    % Find the part we need.
    % Fail clearly if the name is wrong.

    names = strings(1, numel(seg));

    for k = 1:numel(seg)
        names(k) = string(seg(k).name);
    end

    idx = find(names == string(targetName), 1, 'first');

    if isempty(idx)
        availableNames = strjoin(cellstr(names), ', ');
        error('Could not find segment "%s". Available segment names are: %s', ...
            string(targetName), availableNames);
    end
end
