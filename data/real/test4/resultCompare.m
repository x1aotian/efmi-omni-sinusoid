clc;
clear;

gt = textread('gt.txt');
gt(:,1) = gt(:,1) - gt(1,1);
gt(:,2) = gt(:,2) - gt(1,2);
gt(:,3) = gt(:,3) - gt(1,3);

r1 = textread('results3.txt');
r2 = textread('results4.txt');

% scaling = norm(gt(15,1),gt(15,2))/norm(r1(14,1:2));
scaling = 0.0002;
r1(:,1) = scaling * r1(:,1);
r1(:,2) = scaling * r1(:,2);
r2(:,1) = scaling * r2(:,1);
r2(:,2) = scaling * r2(:,2);

figure;
plot(-gt(2:end,2)+gt(2,2), 'r-o','MarkerSize',6);
grid on;
hold on;
plot(r1(:,1)-r1(1,1), 'g-^', 'MarkerSize',8);
hold on;
plot(r2(:,1)-r2(1,1), 'b-*');
hold off;
legend('Groundtruth' , 'FMT', 'eFMT');
xlabel('frames');
ylabel('positions (m) in x axis');

% figure;
% plot(-gt(2:end,2)+gt(2,2), -gt(2:end,1)+gt(2,1), 'r-o','MarkerSize',6);
% grid on;
% hold on;
% plot(r1(:,1)-r1(1,1), - r1(:,2)+r1(1,2), 'g-^', 'MarkerSize',8);
% hold on;
% plot(r2(:,1)-r2(1,1), -r2(:,2)+r2(1,2), 'b-*');
% hold off;
% legend('Groundtruth' , 'FMT', 'eFMT');
% xlim([0, 0.25]);
% ylim([-0.01, 0.01]);
% xlabel('x');
% ylabel('y');
