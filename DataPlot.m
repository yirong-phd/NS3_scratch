%% This Matlab Script plots the NS3-simlated data on server
clear; clc;
file = './ten_link_rand_p2.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x)),T)); % get rid of sentances
sim = str2num(char(T_clear)); % numbers

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,10);

c_rate = zeros(Nsim,10);
sr_rate = zeros(Nsim,10);
%sr_rate = zeros(Nsim,1);

for i = 1:Nsim
    for j = 1:10
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,3+j) - sim(run_time*(i-1)+1:run_time*i,33+j)).^2)/run_time;
        error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j))/run_time);

        c_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,23+j)./sim(run_time*(i-1)+1:run_time*i,3+j))/run_time;
        sr_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,3+j)./(sim(run_time*(i-1)+1:run_time*i,13+j)) - 1)/run_time;
    end
    %sr_rate(i) = sum(1 - sum(sim(run_time*(i-1)+1:run_time*i,14:23),2)./sum(sim(run_time*(i-1)+1:run_time*i,4:13),2))/run_time;
end

figure;
scatter(sr_rate(:,1),error(:,1));
hold on
for id = 2:10
    scatter(sr_rate(:,id),error(:,id));
end

figure;
scatter(c_rate(:,1),error(:,1));
hold on
for id = 2:10
    scatter(c_rate(:,id),error(:,id));
end


figure;
plot3(c_rate(1:30,1),sr_rate(1:30,1),error(1:30,1),'-.*','LineWidth',5);
grid on
hold on
for id = 2:10
    plot3(c_rate(1:30,id),sr_rate(1:30,id),error(1:30,id),'-.*','LineWidth',5);
end
lgd = legend("link 1","link 2","link 3","link 4","link 5","link 6","link 7","link 8","link 9","link 10");
lgd.FontSize = 24;

figure;
plot3(c_rate(31:60,1),sr_rate(31:60,1),error(31:60,1),'-.*','LineWidth',5);
hold on
grid on
for id = 2:10
    plot3(c_rate(31:60,id),sr_rate(31:60,id),error(31:60,id),'-.*','LineWidth',5);
end
lgd = legend("link 1","link 2","link 3","link 4","link 5","link 6","link 7","link 8","link 9","link 10");
lgd.FontSize = 24;

%% Plot for 5-link cases:
clear; clc;
file = './ten_link_rand.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x)),T)); % get rid of sentances
sim = str2num(char(T_clear)); % numbers

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,5);

c_rate = zeros(Nsim,5);
sr_rate = zeros(Nsim,5);

for i = 1:Nsim
    for j = 1:5
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,3+j) - sim(run_time*(i-1)+1:run_time*i,18+j)).^2)/run_time;
        error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j))/run_time);

        c_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,13+j)./sim(run_time*(i-1)+1:run_time*i,3+j))/run_time;
        sr_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,3+j)./(sim(run_time*(i-1)+1:run_time*i,8+j)) - 1)/run_time;
    end
    %sr_rate(i) = sum(1 - sum(sim(run_time*(i-1)+1:run_time*i,14:23),2)./sum(sim(run_time*(i-1)+1:run_time*i,4:13),2))/run_time;
end

figure;
scatter(sr_rate(:,1),error(:,1));
hold on
for id = 2:5
    scatter(sr_rate(:,id),error(:,id));
end

figure;
scatter(c_rate(:,1),error(:,1));
hold on
for id = 2:5
    scatter(c_rate(:,id),error(:,id));
end


figure;
plot3(c_rate(1:30,1),sr_rate(1:30,1),error(1:30,1),'-.*','LineWidth',5);
grid on
hold on
for id = 2:5
    plot3(c_rate(1:30,id),sr_rate(1:30,id),error(1:30,id),'-.*','LineWidth',5);
end
lgd = legend("link 1","link 2","link 3","link 4","link 5");
lgd.FontSize = 24;

figure;
plot3(c_rate(31:60,1),sr_rate(31:60,1),error(31:60,1),'-.*','LineWidth',5);
hold on
grid on
for id = 2:5
    plot3(c_rate(31:60,id),sr_rate(31:60,id),error(31:60,id),'-.*','LineWidth',5);
end
lgd = legend("link 1","link 2","link 3","link 4","link 5");
lgd.FontSize = 24;