%% This script plots the algo's performance vs. the "distance from ideal non-HN topology"
clear; clc;
file = './five_link_deltaHN.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x(1:2))),T)); % get rid of sentances
%T_clear = T_clear(3:end);
sim = str2num(char(T_clear)); % numbers

run_time = 50;
Nsim = round((length(sim))/run_time);
p_rate = zeros(Nsim,5);
p_rate_est = zeros(Nsim,5);
error = zeros(Nsim,5);
c_rate = zeros(Nsim,5);

sim_time = zeros(Nsim,1);
KL = zeros(Nsim,1);

for i = 1:Nsim
    sim_time(i) = sim(run_time*i,1);
    for j = 1:5
        p_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,3+j))/run_time;
        p_rate_est(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,18+j))/run_time;
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,3+j) - sim(run_time*(i-1)+1:run_time*i,18+j)).^2)/run_time;
        error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j).^2)/run_time);
        c_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,13+j))/run_time;
        c_rate(i,j) = c_rate(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j))/run_time);
    end
    
end

figure;
subplot(2,1,1)
plot(p_rate)
subplot(2,1,2)
plot(c_rate)

for i = 1:Nsim
    p_rate(i,:) = p_rate(i,:)./sum(p_rate(i,:));
    p_rate_est(i,:) = p_rate_est(i,:)./sum(p_rate_est(i,:));

    KL(i) = sum(p_rate(i,:).*log(p_rate(i,:)./p_rate_est(i,:)));
end

figure;
semilogy(sum(c_rate,2),KL,'-.*')

figure;
plot(c_rate(:,1),(error(:,1)),'-.*','LineWidth',1)
hold on
for i=2:3
    plot(c_rate(:,i),(error(:,i)),'-.*','LineWidth',1)
end