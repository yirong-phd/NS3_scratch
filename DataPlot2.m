%% This script plots the NS3-simulated Data on the server (MSE vs. T)
clear; clc;
file = './five_link_T_result.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x(1:2))),T)); % get rid of sentances
T_clear = T_clear(3:end);
sim = str2num(char(T_clear)); % numbers

run_time = 100;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,5);
p_rate = zeros(Nsim,5);

sim_time = zeros(Nsim,1);

for i = 1:Nsim
    sim_time(i) = sim(run_time*i,1);
    for j = 1:5
        p_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,3+j))/run_time;
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,3+j) - sim(run_time*(i-1)+1:run_time*i,18+j)).^2)/run_time;
        %error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j).^2)/run_time);
    end
    
end

n_error = sqrt(error);

figure;
semilogy(sim_time-1, sqrt(error(:,1)),'-.*','LineWidth',5)
hold on
for i=2:5
    plot(sim_time-1, sqrt(error(:,i)),'-.*','LineWidth',5)
end
grid on
xlabel("Observation Time (s)",'FontSize',34)
ylabel("Normalized Error",'FontSize',34)
title("Normalized Error v.s. Observation Time",'FontSize',36)
lgd = legend("flow 1","flow 2","flow 3","flow 4","flow 5");
lgd.FontSize = 34;
set(gca,'FontSize',32)

figure;
loglog(sim_time-1,error(:,1),'-.*','LineWidth',5,'DisplayName','flow 1')
hold on
%P = polyfit(log(sim_time(30:end)),log(error(30:end,1)),1);
%yfit = P(1)*log(sim_time)+P(2); display(P(1))
%plot(log(sim_time),yfit,'-.');

loglog(sim_time-1,error(:,2),'-.*','LineWidth',5,'DisplayName','flow 2')
%P = polyfit(log(sim_time(40:end)),log(error(40:end,2)),1); display(P(1))
%yfit = P(1)*log(sim_time)+P(2);
%plot(log(sim_time),yfit,'-.');

loglog(sim_time-1,error(:,3),'-.*','LineWidth',5,'DisplayName','flow 3')
%P = polyfit(log(sim_time(10:end)),log(error(10:end,3)),1); display(P(1))
%yfit = P(1)*log(sim_time)+P(2);
%plot(log(sim_time),yfit,'-.');

loglog(sim_time-1,error(:,4),'-.*','LineWidth',5,'DisplayName','flow 4')
%P = polyfit(log(sim_time(40:end)),log(error(40:end,4)),1); display(P(1))
%yfit = P(1)*log(sim_time)+P(2);
%plot(log(sim_time),yfit,'-.');

loglog(sim_time-1,error(:,5),'-.*','LineWidth',5,'DisplayName','flow 5')
%P = polyfit(log(sim_time(1:end)),log(error(1:end,5)),1); display(P(1))
%yfit = P(1)*log(sim_time)+P(2);
%plot(log(sim_time),yfit,'-.');
grid on
xlabel("log(T)",'FontSize',34)
ylabel("log(MSE)",'FontSize',34)
title("MSE v.s. Observation Time",'FontSize',28)
lgd = legend("flow 1","flow 2","flow 3","flow 4","flow 5");
lgd.FontSize = 34;
set(gca,'FontSize',32)
% Testing the slope of best-fit linear curve in log-log scale:
%P = polyfit(log(sim_time(23:end)),log((error(23:end,1))),1);
P = polyfit(log(sim_time(1:end)),log((error(1:end,1))),1);
yfit = P(1)*log(sim_time(1:end))+P(2); display(P(1));

for i=2:5
    P = polyfit(log(sim_time(1:end)),log((error(1:end,i))),1);
    if i == 2
        P = polyfit(log(sim_time(1:end)),log((error(1:end,i))),1);
    end
    yfit = P(1)*log(sim_time(1:end))+P(2); display(P(1));
    %plot(log(sim_time(1:end)),yfit,'r-.');
end
%% plot the concatenated MSE v.s T figure:
clear all; clc;
load error_1_100.mat
error1 = error;
load time1.mat
time1 = sim_time;
load error_100_200.mat
error2 = error;
load time2.mat
time2 = sim_time;
load nerror1.mat
nerror1 = n_error;
load nerror2.mat
nerror2 = n_error;

time_cat = [time1-1; time2-1];
error_cat = [error1; error2];
nerror_cat = [nerror1; nerror2];

figure;
semilogy(time_cat,nerror_cat,'-.*','LineWidth',5)
grid on
xlabel("Observation Duration T (s)",'FontSize',34)
ylabel("Normalized Error Rate",'FontSize',34)
title("Normalized Error Rate v.s. Observation Time",'FontSize',36)
lgd = legend("flow 1","flow 2","flow 3","flow 4","flow 5");
lgd.FontSize = 34;
set(gca,'FontSize',32)

figure;
hold on
plot(log10(time_cat(1:end,:)),log10(error_cat(1:end,:)),'-.*','LineWidth',3)

for i=1:5
    P = polyfit(log10(time_cat(1:72)),log10(error_cat(1:72,i)),1);
    if i == 2
        P = polyfit(log10(time_cat(1:54)),log10((error_cat(1:54,i))),1);
    end
    yfit = P(1)*log10(time_cat(1:end))+P(2); display(P(1));
    plot(log10(time_cat(1:end)),yfit,'r-.');
end
grid on
xlabel("log(T)",'FontSize',34)
ylabel("log(MSE)",'FontSize',34)
title("MSE v.s. Observation Time (log-log plot)",'FontSize',28)
lgd = legend("flow 1","flow 2","flow 3","flow 4","flow 5");
lgd.FontSize = 34;
set(gca,'FontSize',32)
