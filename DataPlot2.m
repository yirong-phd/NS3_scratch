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
%T_clear = T_clear(3:end);
sim = str2num(char(T_clear)); % numbers

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,5);

sim_time = zeros(Nsim,1);

for i = 1:Nsim
    sim_time(i) = sim(run_time*i,1);
    for j = 1:5
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,3+j) - sim(run_time*(i-1)+1:run_time*i,18+j)).^2)/run_time;
        %error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j).^2)/run_time);
    end
    
end


figure;
plot(sim_time, error(:,1),'-.*','LineWidth',1)
hold on
for i=2:5
    plot(sim_time,error(:,i),'-.*','LineWidth',1)
end

figure;
plot(log(sim_time),log(error(:,1)),'-.*','LineWidth',1)
hold on
P = polyfit(log(sim_time(1:end)),log(error(1:end,1)),1);
yfit = P(1)*log(sim_time)+P(2); display(P(1))
plot(log(sim_time),yfit,'r-.');

for i=2:5
    plot(log(sim_time),log(error(:,i)),'-.*','LineWidth',1)
    P = polyfit(log(sim_time(1:end)),log(error(1:end,i)),1); display(P(1))
    yfit = P(1)*log(sim_time)+P(2);
    plot(log(sim_time),yfit,'r-.');
end
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