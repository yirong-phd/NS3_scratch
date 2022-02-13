%% This script plots the algo's performance vs. the "distance from ideal non-HN topology"
clear; clc;
%file = './five_link_HN_final.txt';
file = './five_link_HN_ex.txt';
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
c_rate_N = zeros(Nsim,1);
c_rate_sum = zeros(Nsim,1);

sim_time = zeros(Nsim,1); delta = zeros(Nsim,1);
KL = zeros(Nsim,1);

for i = 1:Nsim
    sim_time(i) = sim(run_time*i,1); delta(i) = sim(run_time*i,2);
    for j = 1:5
        p_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,3+j))/run_time;
        p_rate_est(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,18+j))/run_time;
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,3+j) - sim(run_time*(i-1)+1:run_time*i,18+j)).^2)/run_time;
        error(i,j) = error(i,j)/(sum(sim(run_time*(i-1)+1:run_time*i,3+j).^2)/run_time);
        c_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,13+j))/run_time;
        c_rate(i,j) = c_rate(i,j)/(sum(sim(run_time*(i-1)+1:run_time*i,3+j))/run_time);
    end  
end

for i = 1:Nsim
    p_rate(i,:) = p_rate(i,:)./sum(p_rate(i,:));
    p_rate_est(i,:) = p_rate_est(i,:)./sum(p_rate_est(i,:));

    KL(i) = sum(p_rate(i,:).*log(p_rate(i,:)./p_rate_est(i,:)));
    c_rate_N(i) = sum(c_rate(i,:).*p_rate(i,:))/sum(p_rate(i,:));
end

figure;
subplot(2,1,1)
plot(KL)
subplot(2,1,2)
plot(c_rate_N)

c_rate_neighbor = zeros(Nsim,5);
for i = 1:Nsim
    for id = 1:5
        c_rate_neighbor(i,id) = sum(c_rate(i,:).*p_rate(i,:)) - c_rate(i,id).*p_rate(i,id);
    end
end

figure;
plot(c_rate_N,KL,'-.*')

figure;
plot(c_rate_N,sqrt(error(:,1)),'-.*')
hold on
for i=2:5
    plot(c_rate_N,sqrt(error(:,i)),'-.*')
end