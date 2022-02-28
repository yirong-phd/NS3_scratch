%% This script plots the NS3-simulated Data on the server (MSE vs. T)
clear; clc;
file = './ten_link_T.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x)),T)); % get rid of sentances
sim = str2num(char(T_clear)); % numbers

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,10);
Npkt = zeros(Nsim,10);
sim_time = zeros(Nsim,1);

for i = 1:Nsim
    sim_time(i) = sim(run_time*i,1);
    for j = 1:10
        Npkt(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,3+j)*(sim_time(i)-1))/run_time;
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,3+j) - sim(run_time*(i-1)+1:run_time*i,33+j)).^2)/run_time;
        %error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j).^2)/run_time);
    end
    
end

figure;
plot(Npkt(:,1),error(:,1))
hold on
for i=2:9
    plot(Npkt(:,i),error(:,i))
end

figure;
plot(log(sim_time),log(error(:,1)),'-.*','LineWidth',1)
hold on
hold on
P = polyfit(log(sim_time),log((error(:,1))),1);
yfit = P(1)*log(sim_time)+P(2); display(P(1));
plot(log(sim_time),yfit,'r-.');
for i=2:9
    plot(log(sim_time),log(error(:,i)),'-.*','LineWidth',1)
end
%%
clear; clc;
file = './five_link_T.txt';
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
        error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j).^2)/run_time);
    end
    
end

figure;
plot(sim_time, sqrt(error(:,1)),'-.*','LineWidth',5)
hold on
for i=2:5
    plot(sim_time, sqrt(error(:,i)),'-.*','LineWidth',5)
end
grid on
xlabel("Observation Time (s)",'FontSize',34)
ylabel("Mean Square Error",'FontSize',34)
title("MSE v.s. Observation Time",'FontSize',36)
lgd = legend("flow 1","flow 2","flow 3","flow 4","flow 5");
lgd.FontSize = 34;
set(gca,'FontSize',32)

figure;
plot(log(sim_time),log(error(:,1)),'-.*','LineWidth',5,'DisplayName','flow 1')
hold on
%P = polyfit(log(sim_time(30:end)),log(error(30:end,1)),1);
%yfit = P(1)*log(sim_time)+P(2); display(P(1))
%plot(log(sim_time),yfit,'-.');

plot(log(sim_time),log(error(:,2)),'-.*','LineWidth',5,'DisplayName','flow 2')
%P = polyfit(log(sim_time(40:end)),log(error(40:end,2)),1); display(P(1))
%yfit = P(1)*log(sim_time)+P(2);
%plot(log(sim_time),yfit,'-.');

plot(log(sim_time),log(error(:,3)),'-.*','LineWidth',5,'DisplayName','flow 3')
%P = polyfit(log(sim_time(10:end)),log(error(10:end,3)),1); display(P(1))
%yfit = P(1)*log(sim_time)+P(2);
%plot(log(sim_time),yfit,'-.');

plot(log(sim_time),log(error(:,4)),'-.*','LineWidth',5,'DisplayName','flow 4')
%P = polyfit(log(sim_time(40:end)),log(error(40:end,4)),1); display(P(1))
%yfit = P(1)*log(sim_time)+P(2);
%plot(log(sim_time),yfit,'-.');

plot(log(sim_time),log(error(:,5)),'-.*','LineWidth',5,'DisplayName','flow 5')
%P = polyfit(log(sim_time(1:end)),log(error(1:end,5)),1); display(P(1))
%yfit = P(1)*log(sim_time)+P(2);
%plot(log(sim_time),yfit,'-.');
grid on
xlabel("log(T)",'FontSize',34)
ylabel("log(MSE)",'FontSize',34)
%title("MSE v.s. Observation Time (log-log plot)",'FontSize',28)
lgd = legend("flow 1","flow 2","flow 3","flow 4","flow 5");
lgd.FontSize = 34;
set(gca,'FontSize',32)
% Testing the slope of best-fit linear curve in log-log scale:
P = polyfit(log(sim_time(23:end)),log((error(23:end,1))),1);
yfit = P(1)*log(sim_time(23:end))+P(2); display(P(1));

for i=2:5
    %plot(log(sim_time(53:end)),log((error(53:end,i))),'-.*','LineWidth',1)
    P = polyfit(log(sim_time(23:end)),log((error(23:end,i))),1);
    if i == 2
        P = polyfit(log(sim_time(13:end)),log((error(13:end,i))),1);
    end
    yfit = P(1)*log(sim_time(13:end))+P(2); display(P(1));
    plot(log(sim_time(13:end)),yfit,'r-.');
end
%%
plot(yfit)
%%
clear; clc;
file = './indep_5l_T.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x)),T)); % get rid of sentances
sim = str2num(char(T_clear)); % numbers

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,3);
Npkt = zeros(Nsim,3);
sim_time = zeros(Nsim,1);

for i = 1:Nsim
    sim_time(i) = sim(run_time*i,1);
    for j = 1:3
        Npkt(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,3+j)*(sim_time(i)-1))/run_time;
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,3+j) - sim(run_time*(i-1)+1:run_time*i,6+j)).^2)/run_time;
        %error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j).^2)/run_time);
    end    
end

figure;
plot(Npkt(:,1),error(:,1))
hold on
plot(Npkt(:,2),error(:,2))
plot(Npkt(:,3),error(:,3))

figure;
plot(sim_time,(error(:,1)),'-.*','LineWidth',1)
hold on
for i=2:3
    plot(sim_time,(error(:,i)),'-.*','LineWidth',1)
end
grid on
lgd = legend("link 1","link 2","link 3");
lgd.FontSize = 24;
xlabel("Sim Time (s)",'FontSize',24)
ylabel("Normalized Error","FontSize",24)

figure;
plot(log(sim_time), log((error(:,1))),'-.*','LineWidth',1)
hold on
P = polyfit(log(sim_time),log((error(:,1))),1);
yfit = P(1)*log(sim_time)+P(2); display(P(1));
plot(log(sim_time),yfit,'r-.');

for i=2:3
    plot(log(sim_time),log((error(:,i))),'-.*','LineWidth',1)
    P = polyfit(log(sim_time),log((error(:,i))),1);
    if i==3
        P = polyfit(log(sim_time(1:25)),log((error(1:25,i))),1);
    end
    yfit = P(1)*log(sim_time)+P(2); display(P(1));
    plot(log(sim_time),yfit,'r-.');
end