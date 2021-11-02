clear;
close all;
clc;

data_u = importdata('22_23/new_results/du_data.txt');
data_v = importdata('22_23/new_results/dv_data.txt');

% do rotation to make data easier to analyse
data_u0 = rot90(data_u);
data_v0 = rot90(data_v);

save('22_23/new_results/dus_wave.mat','data_u0');
save('22_23/new_results/dvs_wave.mat','data_v0');

[x_len, y_len] = size(data_v);

data_plot = data_v; % choose u or v to plot

% plot p-u/v
x = zeros(x_len*y_len, 1);
y = x; val = x;

for xi = 1:x_len
    for yi = 1:y_len
        x( (xi-1)*y_len + yi) = xi;
        y( (xi-1)*y_len + yi) = yi-(y_len+1)/2;
        if data_plot(xi, yi) > 0.05     % threshold to set
            val( (xi-1)*y_len + yi) = data_plot(xi, yi);
        end
    end
end

gscatter(x, y, val, gray)
legend('off');
saveas(gcf,'22_23/new_results/dvs_wave_grays','jpg');
