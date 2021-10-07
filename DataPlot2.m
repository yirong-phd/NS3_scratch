%% This script plots the NS3-simulated Data on the server (MSE vs. T)
clear; clc;
file = './ten_link_T.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x)),T)); % get rid of sentances
sim = str2num(char(T_clear)); % numbers

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,10);

sim_time = zeros(Nsim,1);

for i = 1:Nsim
    sim_time(i) = sim(run_time*i,1);
    for j = 1:10
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,3+j) - sim(run_time*(i-1)+1:run_time*i,33+j)).^2)/run_time;
        error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j).^2)/run_time);
    end
    
end

figure;
plot(sim_time,error(:,1),'-.*','LineWidth',1)
hold on
for i=2:9
    plot(sim_time,error(:,i),'-.*','LineWidth',1)
end