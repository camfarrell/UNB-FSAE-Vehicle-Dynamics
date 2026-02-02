%% UNB FSAE | Wheel Loading and Weight Transfer
clear; clc; close all

VP = Vehicle_Params();

%Acceleration sweeps
ax_vec = linspace(-0.8,  1.1, 100) * VP.g; %longitudinal accel (- = braking)
ay_vec = linspace(-1.4,  1.4, 100) * VP.g; %lateral accel (+ = left turn)

bump_mult = 1.00; %bump mulitplier (1.00 = no bump, 1.10 = +10% load from bump)

%% Print Example 

%Load Case
ax_print = 1.1 * VP.g; % + forward accel, - braking
ay_print = 1.4 * VP.g; % + cornering left

%Contribution breakdown print
C = wheelLoads_components(ax_print, ay_print, bump_mult, VP);

%assemble rows: [Static, Longitudinal, Elastic, Geometric, Offset]
FL_terms = [C.Wf_static/2, -C.dW_L/2, -C.dW_F_el, -C.dW_F_geo, C.dW_F_stat];
FR_terms = [C.Wf_static/2, -C.dW_L/2, C.dW_F_el, C.dW_F_geo, -C.dW_F_stat];
RL_terms = [C.Wr_static/2, C.dW_L/2, -C.dW_R_el, -C.dW_R_geo, C.dW_R_stat];
RR_terms = [C.Wr_static/2, C.dW_L/2, C.dW_R_el, C.dW_R_geo, -C.dW_R_stat];

labels = {'Static','Longitudinal','Elastic lat','Geom lat','CG offset'};

fprintf('\nWHEEL LOAD BREAKDOWN @ ax=%.2f g (+: forward), ay=%.2f g (+: left turn)\n', ax_print / VP.g, ay_print / VP.g);
fprintf('%-18s  %10s  %10s  %10s  %10s\n', '','FL','FR','RL','RR');
fmt = '%-18s  %10.1f  %10.1f  %10.1f  %10.1f\n';
for k = 1:numel(labels)
    fprintf(fmt, labels{k}, FL_terms(k), FR_terms(k), RL_terms(k), RR_terms(k));
end

Totals = [sum(FL_terms), sum(FR_terms), sum(RL_terms), sum(RR_terms)];
fprintf('%-18s  %10.1f  %10.1f  %10.1f  %10.1f\n', 'TOTAL (calc)', Totals(1), Totals(2), Totals(3), Totals(4));
fprintf('%-18s  %10.1f  %10.1f  %10.1f  %10.1f\n', 'TOTAL (C.*) ', C.W_FL, C.W_FR, C.W_RL, C.W_RR);

fprintf('Sum of wheels: %.1f N\n', C.W_sum);

%Roll gradient print
fprintf('\nROLL:\n');
fprintf('Roll gradient: %.2f deg/g \n', VP.phi_grad_deg);
fprintf('Front roll stiffness" %.2f Nm/rad \n', VP.Kphi_f);
fprintf('Rear roll stiffness" %.2f Nm/rad \n', VP.Kphi_r);

%3D-Forces Print
forces = calc3DForces(ax_print, ay_print, bump_mult, VP);

fprintf('\n--- 3D WHEEL FORCES (N) ---\n');
fprintf('Case: ax = %.2fg, ay = %.2fg\n', ax_print/VP.g, ay_print/VP.g);
fprintf('%-10s %12s %12s %12s\n', 'Wheel', 'Fx (Long)', 'Fy (Lat)', 'Fz (Vert)');
fields = {'FL','FR','RL','RR'};
for i = 1:4
    f = fields{i};
    fprintf('%-10s %12.1f %12.1f %12.1f\n', f, forces.(['Fx_',f]), forces.(['Fy_',f]), forces.(['Fz_',f]));
end

%% Plots 
[AX, AY] = meshgrid(ax_vec, ay_vec);

[W_FL, W_FR, W_RL, W_RR] = wheelLoads(AX, AY, bump_mult, VP);

% Store in cell array for looping
WheelData = {W_FL, 'Front Left'; W_FR, 'Front Right'; W_RL, 'Rear Left'; W_RR, 'Rear Right'};

figure('Name', 'Wheel Loads & Lift Check', 'Color', 'w'); 
tiledlayout(2,2, 'Padding', 'compact', 'TileSpacing', 'compact');

for i = 1:4
    nexttile;
    % CONTOUR PLOT
    contourf(AX / VP.g, AY / VP.g, WheelData{i,1}, 20, 'LineStyle', 'none'); 
    hold on;
    %Plot the "Wheel Lift" boundary (0 Newtons) in thick black
    [C, h_cont] = contour(AX / VP.g, AY / VP.g, WheelData{i,1}, [0 0], 'k', 'LineWidth', 2);
    clabel(C, h_cont, 'FontSize', 8, 'Color', 'w', 'FontWeight', 'bold');
    
    colorbar; grid on;
    title(WheelData{i,2}); 
    xlabel('Longitudinal (g)'); ylabel('Lateral (g) ''+'' = left turn');
end
sgtitle('Wheel Loads (N)');
%% Functions 

function [W_FL, W_FR, W_RL, W_RR] = wheelLoads(ax, ay, bump_mult, VP)

    Wf_stat = (VP.W * VP.wf) * bump_mult;   
    Wr_stat = (VP.W * VP.wr) * bump_mult;   
    dW_F_stat = Wf_stat .* (VP.dy ./ VP.tf); %accounting for CG offset
    dW_R_stat = Wr_stat .* (VP.dy ./ VP.tr);  

    % +ax = forward accel (rear gains, front loses)
    dW_L = (VP.W * VP.h / (VP.g * VP.L)) .* ax;    

    %roll moment and elastic distribution
    M_phi = (VP.W/VP.g) .* ay .* VP.H;  % (Nm)      
    frac_f = VP.Kphi_f / (VP.Kphi_f + VP.Kphi_r); %fraction of roll moment on front axle (ref. Milliken)
    frac_r = VP.Kphi_r / (VP.Kphi_f + VP.Kphi_r); %fraction of roll moment on rear axle

    dW_F_el = (frac_f .* M_phi) ./ VP.tf;   
    dW_R_el = (frac_r .* M_phi) ./ VP.tr;   

    %geometric weight transfer (RC)
    dW_F_geo = (VP.W / VP.g) .* ay .* (VP.b / VP.L) .* (VP.Zrf ./ VP.tf);
    dW_R_geo = (VP.W / VP.g) .* ay .* (VP.a / VP.L) .* (VP.Zrr ./ VP.tr);

    %total lateral per axle
    dW_F = dW_F_el + dW_F_geo; %+ adds to right, sub from left
    dW_R = dW_R_el + dW_R_geo;   

    %total longitudinal per axle
    Wf_total = Wf_stat - dW_L;   
    Wr_total = Wr_stat + dW_L;   

    %split to each wheel
    W_FL = (Wf_total/2) - dW_F + dW_F_stat;   
    W_FR = (Wf_total/2) + dW_F - dW_F_stat;   
    W_RL = (Wr_total/2) - dW_R + dW_R_stat;   
    W_RR = (Wr_total/2) + dW_R - dW_R_stat;   

end

%This is the same as function above just using scalar values to work for
%print output instead of a vector range of acceleration values
function C = wheelLoads_components(ax, ay, bump_mult, VP)

    %Static loads (with bump)
    C.Wf_static = (VP.W * VP.wf) * bump_mult;   
    C.Wr_static = (VP.W * VP.wr) * bump_mult;  

    %Load transfer from CG offset
    C.dW_F_stat = C.Wf_static * (VP.dy / VP.tf); 
    C.dW_R_stat = C.Wr_static * (VP.dy / VP.tr);     
    
    %Load transfer from Longitudinal accel
    C.dW_L = (VP.W * VP.h / (VP.g * VP.L)) * ax;    
   
    %Laterl components (geometric and elastic)
    C.M_phi = (VP.W / VP.g) * ay * VP.H;             
    frac_f = VP.Kphi_f / (VP.Kphi_f + VP.Kphi_r);
    frac_r = VP.Kphi_r / (VP.Kphi_f + VP.Kphi_r);

    C.dW_F_el = (frac_f * C.M_phi) / VP.tf;  
    C.dW_R_el = (frac_r * C.M_phi) / VP.tr;  

    C.dW_F_geo = (VP.W / VP.g) * ay * (VP.b / VP.L) * (VP.Zrf / VP.tf); 
    C.dW_R_geo = (VP.W / VP.g) * ay * (VP.a / VP.L) * (VP.Zrr / VP.tr); 

    %Total load each wheel
    C.W_FL = C.Wf_static/2 - C.dW_L/2 - C.dW_F_el - C.dW_F_geo + C.dW_F_stat;
    C.W_FR = C.Wf_static/2 - C.dW_L/2 + C.dW_F_el + C.dW_F_geo - C.dW_F_stat;
    C.W_RL = C.Wr_static/2 + C.dW_L/2 - C.dW_R_el - C.dW_R_geo + C.dW_R_stat;
    C.W_RR = C.Wr_static/2 + C.dW_L/2 + C.dW_R_el + C.dW_R_geo - C.dW_R_stat;

    C.W_sum = C.W_FL + C.W_FR + C.W_RL + C.W_RR;
end

function F = calc3DForces(ax, ay, bump, VP)
    % 1. VERTICAL FORCES (Fz)
    Wf_stat = (VP.W * VP.wf) * bump;   
    Wr_stat = (VP.W * VP.wr) * bump;   
    dW_L = (VP.W * VP.h / (VP.g * VP.L)) * ax; % Longitudinal load transfer
    
    % Lateral load transfer components
    M_phi = (VP.W/VP.g) * ay * VP.H;
    frac_f = VP.Kphi_f / (VP.Kphi_f + VP.Kphi_r);
    frac_r = 1 - frac_f;
    
    dW_F_lat = ((frac_f * M_phi) + (VP.W/VP.g * ay * (VP.b/VP.L) * VP.Zrf)) / VP.tf;
    dW_R_lat = ((frac_r * M_phi) + (VP.W/VP.g * ay * (VP.a/VP.L) * VP.Zrr)) / VP.tr;
    
    % CG offset compensation
    dW_F_off = Wf_stat * (VP.dy / VP.tf);
    dW_R_off = Wr_stat * (VP.dy / VP.tr);

    F.Fz_FL = max(0, (Wf_stat/2) - (dW_L/2) - dW_F_lat + dW_F_off);
    F.Fz_FR = max(0, (Wf_stat/2) - (dW_L/2) + dW_F_lat - dW_F_off);
    F.Fz_RL = max(0, (Wr_stat/2) + (dW_L/2) - dW_R_lat + dW_R_off);
    F.Fz_RR = max(0, (Wr_stat/2) + (dW_L/2) + dW_R_lat - dW_R_off);

    % 2. LONGITUDINAL FORCES (Fx)
    total_Fx = (VP.W / VP.g) * ax;
    if ax >= 0 % Accelerating
        F.Fx_FL = 0;
        F.Fx_FR = 0;
        F.Fx_RL = total_Fx / 2;
        F.Fx_RR = total_Fx / 2;
    else % Braking
        F.Fx_FL = (total_Fx * VP.BB) / 2;
        F.Fx_FR = (total_Fx * VP.BB) / 2;
        F.Fx_RL = (total_Fx * (1-VP.BB)) / 2;
        F.Fx_RR = (total_Fx * (1-VP.BB)) / 2;
    end

    % 3. LATERAL FORCES (Fy)
    total_Fy = (VP.W / VP.g) * ay;
    % Distributed by vertical load ratio
    F.Fy_FL = total_Fy * (F.Fz_FL / VP.W);
    F.Fy_FR = total_Fy * (F.Fz_FR / VP.W);
    F.Fy_RL = total_Fy * (F.Fz_RL / VP.W);
    F.Fy_RR = total_Fy * (F.Fz_RR / VP.W);
end