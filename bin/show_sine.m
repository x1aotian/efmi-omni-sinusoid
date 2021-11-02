clc;
clear;

idx = textread('idxs.txt');
us = textread('delta_xs.txt');
vs = textread('delta_ys.txt');
thetas = textread('thetas.txt');

[row,~] = size(idx);

sine_u = [];
sine_v = [];
W = 1100;
lamda = W/(2*pi)*0.1;

% wtrans_wrot
param_u = [-0.00978632 0.28456 2.61541];
param_v = [-0.0784 -1.21051 8.85841];
for i = 1:W
    sine_u = [sine_u; ...
        W/(2*pi)*param_u(1)*sin(2*pi/W*i+param_u(2)) + param_u(3)];
    sine_v = [sine_v; ...
        W/(2*pi)*param_v(1)*sin(2*pi/W*i+param_v(2)) + param_v(3) ...
        - lamda*param_v(1)*cos(2*pi/W*i+param_v(2))];
end

% wotrans_wrot
% param_u = [2.79716];
% param_v = [-0.0495703 -0.771952];
% for i = 1:W
%     sine_u = [sine_u; param_u(1)];
%     sine_v = [sine_v; ...
%         W/(2*pi)*param_v(1)*sin(2*pi/W*i+param_v(2))...
%         - lamda*param_v(1)*cos(2*pi/W*i+param_v(2))];
% end

figure;
plot(idx, us, 'o');
hold on;
plot(idx, vs + lamda * thetas, '*');
hold on;
plot(sine_u);
hold on;
plot(sine_v);
grid on;
hold off;
legend('\Delta u', '\Delta v + \eta * \Delta\theta',...
    'Fitting with \Delta u', 'Fitting with \Delta v + \eta * \Delta\theta');
xlim([0, 1100]);
ylim([-10, 30]);
xlabel('Column Index');
ylabel('Pixel Deviation');
title('Fitting with Translation');
set(gca,'FontSize',12); 
set(get(gca,'XLabel'),'FontSize',14);
set(get(gca,'YLabel'),'FontSize',14);



