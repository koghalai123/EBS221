clc, clear
close all

load("plant_bed.mat")
load("test_wpts.mat")

start_state = [2.0, -5.5, 1.57];

hold on
scatter(start_state(1), start_state(2), 30, "*b")

for i=1:length(PLANT_BED_D)
    plot([PLANT_BED_D(1, i),PLANT_BED_F(1, i)], [PLANT_BED_D(2, i),PLANT_BED_F(2, i)], 'r', 'LineWidth', 1.5)
end

plot(wpts(1,:), wpts(2,:), '-og')
hold off