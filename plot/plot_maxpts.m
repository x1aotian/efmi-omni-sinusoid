clear;
close all;
clc;

data_u = importdata('22_23/new_results/du_data.txt');
data_v = importdata('22_23/new_results/dv_data.txt');

[x_len, y_len] = size(data_u);
x = zeros(x_len, 1);
y = x; val = x;

data_plot = data_v; % choose u or v to plot

for xi = 1:x_len
    x(xi) = xi;
    max = 0; max_idx = 0;
    
    for yi = 1:y_len
        if data_plot(xi,yi) > max
            max = data_plot(xi,yi);
            max_idx = yi - (y_len+1)/2;
        end
    
    end
    
    y(xi) = max_idx;
    val(xi) = max;

    % in case one column is all zero
    if max == 0
        y(xi) = 0;
        val(xi) = 1;
    end
end

gscatter(x, y, val, gray);
legend('off');
saveas(gcf,'22_23/new_results/dvs_wave_max','jpg');
