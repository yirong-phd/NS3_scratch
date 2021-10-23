%% This draft plots the NS3-simulated MSE & SR / Collision rates with 3-link
clear; clc;
file = './three_link_rand.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x)),T)); % get rid of sentances
sim = str2num(char(T_clear)); % numbers

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,10);

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,3);

c_rate = zeros(Nsim,3);
c_rate_N = zeros(Nsim,3);
sr_rate = zeros(Nsim,3);

for i = 1:Nsim
    for j = 1:3
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,8+j) - sim(run_time*(i-1)+1:run_time*i,11+j)).^2)/run_time;
        error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,8+j).^2)/run_time);

        c_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,2+j))/run_time;
        sr_rate(i,j) = sum(1 - sim(run_time*(i-1)+1:run_time*i,5+j) )/run_time;
    end
end

figure;
plot(error(:,1))
hold on
for id = 2:3
    plot(error(:,id));
end

figure;
plot(sr_rate(:,3),c_rate(:,3))
%%
means = logspace(-3,-1,30)';
%means = linspace(0.001,0.05,20)';
writematrix(means,'./ia_means_3l.txt');