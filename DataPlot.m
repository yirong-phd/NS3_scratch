%% This Matlab Script plots the NS3-simlated data on server
clear; clc;
%file = './ten_link_rand_p1.txt';
file = './ten_link_rand_R.txt';
T = textread(file,'%s','delimiter','\n');
T_clear = T(~cellfun(@(x) any(isletter(x)),T)); % get rid of sentances
sim = str2num(char(T_clear)); % numbers

fileCG = './topology_cg.txt';
TCG = textread(fileCG,'%s','delimiter','\n');
TCG_clear = TCG(~cellfun(@(x) any(isletter(x)), TCG));
CG = str2num(char(TCG_clear));

run_time = 30;
Nsim = round((length(sim))/run_time);
error = zeros(Nsim,10);

c_rate = zeros(Nsim,10);
c_rate_N = zeros(Nsim,10);
sr_rate = zeros(Nsim,10);

%sr_rate = zeros(Nsim,1);
C_rate = zeros(Nsim,1);

r_error = zeros(Nsim,10);
x_metric = zeros(Nsim,10);

cg = zeros(10);
cg(1,1)=1; cg(1,4)=1; cg(1,8)=1; cg(1,9)=1;
cg(2,2)=1; cg(2,6)=1; cg(2,7)=1;
cg(3,3)=1; cg(3,6)=1; 
cg(4,1)=1; cg(4,4)=1; cg(4,5)=1; cg(4,8)=1;
cg(5,4)=1; cg(5,5)=1;
cg(6,2)=1; cg(6,3)=1; cg(6,6)=1;
cg(7,2)=1; cg(7,7)=1;
cg(8,1)=1; cg(8,4)=1; cg(8,8)=1;
cg(9,1)=1; cg(9,9)=1;
cg(10,10)=1;


for i = 1:Nsim
    for j = 1:10
        error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,3+j) - sim(run_time*(i-1)+1:run_time*i,33+j)).^2)/run_time;
        error(i,j) = error(i,j) / (sum(sim(run_time*(i-1)+1:run_time*i,3+j).^2)/run_time);

        c_rate(i,j) = sum(sim(run_time*(i-1)+1:run_time*i,23+j)./sim(run_time*(i-1)+1:run_time*i,3+j))/run_time;
        c_rate_N(i,j) = sum( sum(sim(run_time*(i-1)+1:run_time*i,23+find(cg(j,:)==0)),2) ./ sum(sim(run_time*(i-1)+1:run_time*i,3+find(cg(j,:)==0)),2) )/run_time;

        sr_rate(i,j) = sum(1 - sim(run_time*(i-1)+1:run_time*i,13+j)./(sim(run_time*(i-1)+1:run_time*i,3+j)))/run_time;

        r_error(i,j) = sum((sim(run_time*(i-1)+1:run_time*i,43+j) - sim(run_time*(i-1)+1:run_time*i,53+j)).^2)/run_time;
        r_error(i,j) = r_error(i,j)/(sum(sim(run_time*(i-1)+1:run_time*i,53+j).^2)/run_time);
    end
    %sr_rate(i) = sum(1 - sum(sim(run_time*(i-1)+1:run_time*i,14:23),2)./sum(sim(run_time*(i-1)+1:run_time*i,4:13),2))/run_time;
    C_rate(i) = sum( sum(sim(run_time*(i-1)+1:run_time*i,24:33),2)./sum(sim(run_time*(i-1)+1:run_time*i,4:13),2) )/run_time;
end

for i = 1:Nsim
    for j = 1:10
        x_metric(i,j) = sum( r_error(i,find(cg(j,:)==0)) );
    end
end

figure;
subplot(1,2,1)
plot(error(31:60,1));
hold on
for id = 2:10
    plot(error(31:60,id));
end
subplot(1,2,2)
plot(c_rate(31:60,1));
hold on
for id = 2:10
    plot(c_rate(31:60,id));
end

figure;
plot(c_rate(1:30,1),r_error(1:30,1))
hold on
for id = 2:10
    plot(c_rate(1:30,id),r_error(1:30,id));
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
xlabel('The collision rate within legitimate network','FontSize',24);
ylabel('The collision rate at the observer','FontSize',24);
zlabel('The normalized MSE','FontSize',24);

figure;
plot3(c_rate(31:60,1),sr_rate(31:60,1),error(31:60,1),'-.*','LineWidth',5);
hold on
grid on
for id = 2:10
    plot3(c_rate(31:60,id),sr_rate(31:60,id),error(31:60,id),'-.*','LineWidth',5);
end
lgd = legend("link 1","link 2","link 3","link 4","link 5","link 6","link 7","link 8","link 9","link 10");
lgd.FontSize = 24;
xlabel('The collision rate within legitimate network','FontSize',24);
ylabel('The collision rate at the observer','FontSize',24);
zlabel('The normalized MSE','FontSize',24);
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