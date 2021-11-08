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
%%
clear; clc;
file = './three_link_T.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x(1:2))),T)); % get rid of sentances
T_clear = T_clear(3:end);
sim = str2num(char(T_clear)); % numbers

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,3);

sim_time = zeros(Nsim,1);

for i = 1:Nsim
    sim_time(i) = sim(run_time*i,1);
    for j = 1:3
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,8+j) - sim(run_time*(i-1)+1:run_time*i,11+j)).^2)/run_time;
        error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,8+j).^2)/run_time);
    end
    
end


figure;
plot(log(sim_time),log(error(:,1)),'-.*','LineWidth',1)
hold on
P = polyfit(log(sim_time(25:end)),log(error(25:end,1)),1);
yfit = P(1)*log(sim_time)+P(2); display(P(1))
plot(log(sim_time),yfit,'r-.');

for i=2:3
    plot(log(sim_time),log(error(:,i)),'-.*','LineWidth',1)
    P = polyfit(log(sim_time(25:end)),log(error(25:end,i)),1); display(P(1))
    yfit = P(1)*log(sim_time)+P(2);
    plot(log(sim_time),yfit,'r-.');
end