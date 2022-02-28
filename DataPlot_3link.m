%% This draft plots the NS3-simulated MSE & SR / Collision rates with 3-link
clear; clc;
file = './three_link_noHN.txt';
T = textread(file,'%s','delimiter','\n');
%T_clear = T(~cellfun(@(x) any(isletter(x)),T)); % get rid of sentances
T_clear = T(~cellfun(@(x) any(isletter(x(1:2))),T)); % get rid of sentances (while preserves the NaN's)
%T_clear = T_clear(3:end);
sim = str2num(char(T_clear)); % numbers

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,10);

run_time = 30;
Nsim = round((length(sim))/run_time);

p_rate = zeros(Nsim,3);
error = zeros(Nsim,3);

c_rate = zeros(Nsim,3);
c_rate_N = zeros(Nsim,3);
sr_rate = zeros(Nsim,3);

C_rate = zeros(Nsim,1);

for i = 1:Nsim

    row_idx = [];
    for k = run_time*(i-1)+1:run_time*i
        if ~isnan(sim(k,:))
            row_idx = [row_idx,k];
        end
    end

    avg_N = size(row_idx,2);

    for j = 1:3
        p_rate(i,j) = sum(sim(row_idx,8+j))/avg_N;
  
        error(i,j) = sum((sim(row_idx,8+j) - sim(row_idx,11+j)).^2)/avg_N;
        error(i,j) = error(i,j) / (sum(sim(row_idx,8+j).^2)/avg_N);

        c_rate(i,j) = sum( sim(row_idx,2+j).*sim(row_idx,8+j)./sum(sim(row_idx,9:11),2) )/avg_N;
        sr_rate(i,j) = sum(1 - sim(row_idx,5+j) )/avg_N;
    end
    C_rate(i) = sum( sum(sim(row_idx,3:5).*sim(row_idx,9:11),2) ./sum(sim(row_idx,9:11),2) )/avg_N;
    
end

figure;
plot(p_rate(:,1))
hold on
for id = 2:3
    plot(p_rate(:,id));
end

figure;
plot(sr_rate(:,1),sqrt(error(:,1)))
hold on
for id = 2:3
    plot(sr_rate(:,id),sqrt(error(1:end,id)));
end


figure;
semilogy(sqrt(error(1:end,1)))
hold on
for id = 2:3
    plot(sqrt(error(1:end,id)));
end

figure;
plot(sr_rate(:,1))
hold on
for id = 2:3
    plot(sr_rate(:,id));
end
%%
clear all;
clc;

load e_asymHN.mat;
e_asymHN = error;
load e_symHN.mat;
e_symHN = error;
load e_symHN_fixedCW.mat
e_symHN_fixedCW = error;

load e_symHNstrong.mat
e_symHNstrong = error;
load e_noHN.mat;
e_noHN = error;
load crate_symHNstrong.mat;
crate_symHNstrong = C_rate;
load crate_symHN.mat;
crate_symHN = C_rate;

load prate_symHN.mat;

figure;
subplot(2,3,1)
semilogy(e_noHN(1:end,1),'-.*','LineWidth',1)
hold on
plot(e_symHN(1:end,1))
lgd = legend("baseline","with weak HN");
ylabel('Normalized Error')
xlabel('Input index')
for id = 2:3
    subplot(2,3,id)
    semilogy(e_noHN(1:end,id),'-.*','LineWidth',1);
    hold on
    plot(e_symHN(1:end,id))
    lgd = legend("baseline","with weak HN");
end

subplot(2,3,4)
plot(crate_symHN(1:end));
ylabel('Network HN Collision Rate')
xlabel('Input index')
for id = 2:3
    subplot(2,3,id+3)
    plot(crate_symHN(1:end));
    ylabel('Network HN Collision Rate')
xlabel('Input index')
end

figure;
subplot(2,3,1)
semilogy(e_noHN(1:end,1),'-.*','LineWidth',1)
hold on
plot(e_symHNstrong(1:end,1))
lgd = legend("baseline","with strong HN");
ylabel('Normalized Error')
xlabel('Input index')

for id = 2:3
    subplot(2,3,id)
    semilogy(e_noHN(1:end,id),'-.*','LineWidth',1);
    hold on
    plot(e_symHNstrong(1:end,id))
    lgd = legend("baseline","with strong HN");
ylabel('Normalized Error')
xlabel('Input index')
end

subplot(2,3,4)
plot(crate_symHNstrong(1:end));
ylabel('Network HN Collision Rate')
xlabel('Input index')
for id = 2:3
    subplot(2,3,id+3)
    plot(crate_symHNstrong(1:end));
    ylabel('Network HN Collision Rate')
    xlabel('Input index')
end


figure;
subplot(1,3,1)
plot(e_noHN(1:end,1),'-.*','LineWidth',1)
hold on
plot(e_asymHN(1:end,1))

for id = 2:3
    subplot(1,3,id)
    plot(e_noHN(1:end,id),'-.*','LineWidth',1);
    hold on
    plot(e_asymHN(1:end,id))
end

%{
figure;
plot(e_noHN(1:end,1),'-.*','LineWidth',1)
hold on
plot(e_symHN(1:end,1))
plot(e_symHN_fixedCW(1:end,1),'-.o','LineWidth',1)
%}
%%
clear; clc;
file = './three_link_T.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x(1:2))),T)); % get rid of sentances
%T_clear = T_clear(3:end);
sim = str2num(char(T_clear)); % numbers

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,10);

sim_time = zeros(Nsim,1);

for i = 1:Nsim

    row_idx = [];
    for k = run_time*(i-1)+1:run_time*i
        if ~isnan(sim(k,:))
            row_idx = [row_idx,k];
        end
    end

    avg_N = size(row_idx,2);

    for j = 1:3
  
        error(i,j) = sum((sim(row_idx,8+j) - sim(row_idx,11+j)).^2)/avg_N;
        error(i,j) = error(i,j) / (sum(sim(row_idx,8+j).^2)/avg_N);

    end
    sim_time(i) = sum(sim(row_idx,1))/avg_N;
    
end

figure;
plot(sim_time,sqrt(error(:,1)),'-.*','LineWidth',1)
hold on
for i=2:3
    plot(sim_time,sqrt(error(:,i)),'-.*','LineWidth',1)
end
%%
clear all
clc;
means = logspace(-3,-1,30)';
%means = linspace(0.001,0.05,20)';
%writematrix(means,'./ia_means_3l.txt');
delta1 = linspace(3.1,3.4,50)';
delta2 = linspace(3.4,4.2,31)';
delta = [delta1(1:end-1);delta2];
idx = randperm(length(delta));
delta_shuffle = delta(idx);
Delta = [delta,delta_shuffle];
writematrix(Delta,"./deltaHN.txt");
%%
delta1 = linspace(4.1,4.3,30)';
delta2 = linspace(3.1,3.23,20)';
delta_ex = [delta2;delta1];
writematrix(delta_ex,"./deltaHN.txt");
%%
time = logspace(2,2.3,50)';
time = time + 1;
%time = linspace(20,100,50)';
writematrix(time,'./time2.txt');
%means = logspace(-3,-1,30)';
%writematrix(means,'./ia_means_3l.txt');