%%
vals = importdata('22_23/original_have/values-rot-scale.txt');

[X,Y]=meshgrid(1:251,1:251);

mesh(X,Y,vals);
saveas(gcf,'22_23/original_have/shift_rot_scale','jpg');

%%
trans = importdata('22_23/original_have/afterFilter.txt');

[X,Y]=meshgrid(1:251,1:251);
mesh(X,Y,trans);
saveas(gcf,'22_23/original_have/shift_trans','jpg');

%%
xs = importdata('22_23/original_have/delta_xs.txt');
ys = importdata('22_23/original_have/delta_ys.txt');
idxs = 1:1:117;

plot(idxs,xs,'o');
saveas(gcf,'22_23/original_have/dxs','jpg');

plot(idxs,ys,'o');
saveas(gcf,'22_23/original_have/dys','jpg');