%% PROGRAM TO FANCY RESULTS
clc, clear all, close all;
load("h_mpc");
load("obj_3d_mpc");
load("time_mpc");
load("uv_mpc");
load("uv_d_mpc");
load("he_mpc");
load("U_mpc");

%% Norm 2 pixels error

for k=1:length(he_mpc)
   he1_mpc(k) = norm(he_mpc(1:2,k),2); 
   he2_mpc(k) = norm(he_mpc(3:4,k),2); 
   he3_mpc(k) = norm(he_mpc(5:6,k),2);
   he4_mpc(k) = norm(he_mpc(7:8,k),2);
    
end
obj_3d_conec_mpc = [obj_3d_mpc(1,1), obj_3d_mpc(4,1), obj_3d_mpc(7,1), obj_3d_mpc(10,1), obj_3d_mpc(1,1);...
                     obj_3d_mpc(2,1), obj_3d_mpc(5,1), obj_3d_mpc(8,1), obj_3d_mpc(11,1), obj_3d_mpc(2,1);...
                     obj_3d_mpc(3,1), obj_3d_mpc(6,1), obj_3d_mpc(9,1), obj_3d_mpc(12,1), obj_3d_mpc(3,1)];
N = 10;
uo = 2.4597651153505402e+02;
vo = 1.9955473351505680e+02;

%% Fancy Pictures
Drone_Parameters(0.008);
%% Parameters fancy plots
% define plot properties
lw = 2; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 13; %11
fontsizeLabel1 = 15; %11
fontsizeLegend = 13;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 600; % size figure
sizeY = 900; % size figure

% color propreties
C1 = [246 170 141]/255;
C2 = [51 187 238]/255;
C3 = [0 153 136]/255;
C4 = [238 119 51]/255;
C5 = [204 51 17]/255;
C6 = [238 51 119]/255;
C7 = [187 187 187]/255;
C8 = [80 80 80]/255;
C9 = [140 140 140]/255;
C10 = [0 128 255]/255;
C11 = [234 52 89]/255;
C12 = [39 124 252]/255;
C13 = [40 122 125]/255;
%C14 = [86 215 219]/255;
C14 = [252 94 158]/255;
C15 = [244 171 39]/255;
C16 = [100 121 162]/255;
C17 = [255 0 0]/255;
C18 = [242 144 182]/255;
C19 = [229 23 101]/255;
C20 = [129 231 174]/255;
C21 = [42 142 86]/255;
C22 = [139 177 221]/255;
C23 = [64 109 159]/255;
C24 = [233 230 129]/255;
C25 = [178 173 12]/255;

figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
%set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');
box on
subplot(2,1,1)
%view(20,15);

plot3(h_mpc(1,:),h_mpc(2,:),h_mpc(3,:),'-.','Color',C9,'LineWidth',lw*1.2); hold on
plot3(obj_3d_conec_mpc(1,:),obj_3d_conec_mpc(2,:),obj_3d_conec_mpc(3,:),'--','Color',C16,'LineWidth',lw);
plot3(obj_3d_mpc(1,1),obj_3d_mpc(2,1),obj_3d_mpc(3,1),'o','Color',C16,'LineWidth',lw);
plot3(obj_3d_mpc(4,1),obj_3d_mpc(5,1),obj_3d_mpc(6,1),'o','Color',C16,'LineWidth',lw);
plot3(obj_3d_mpc(7,1),obj_3d_mpc(8,1),obj_3d_mpc(9,1),'o','Color',C16,'LineWidth',lw);
plot3(obj_3d_mpc(10,1),obj_3d_mpc(11,1),obj_3d_mpc(12,1),'o','Color',C16,'LineWidth',lw);
G2=Drone_Plot_3D(h_mpc(1,end),h_mpc(2,end),h_mpc(3,end),0,0,h_mpc(4,end));hold on
%plot3(t,ul,'-','Color',C11,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$\textrm{Y}[m]$','interpreter','latex','fontsize',fontsizeLabel)
xlabel('$\textrm{X}[m]$','interpreter','latex','fontsize',fontsizeLabel)
zlabel('$\textrm{Z}[m]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\eta$','$^a\xi$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;
axis([-0.5 0.5 -0.5 0.5 -0.05 2])

subplot(2,1,2)
xlim([-uo uo])
ylim([-vo vo])
set(gca,'Ydir','reverse')
plot(uv_mpc(1,:),uv_mpc(2,:),'--','Color',C19,'LineWidth',lw*1); hold on
plot(uv_mpc(3,:),uv_mpc(4,:),'--','Color',C21,'LineWidth',lw*1);
plot(uv_mpc(5,:),uv_mpc(6,:),'--','Color',C23,'LineWidth',lw*1); hold on
plot(uv_mpc(7,:),uv_mpc(8,:),'--','Color',C25,'LineWidth',lw*1);

plot(uv_d_mpc(1,1),uv_d_mpc(2,1),'o','Color',C9,'LineWidth',lw*1.5);
plot(uv_d_mpc(3,1),uv_d_mpc(4,1),'o','Color',C9,'LineWidth',lw*1.5);
plot(uv_d_mpc(5,1),uv_d_mpc(6,1),'o','Color',C9,'LineWidth',lw*1.5);
plot(uv_d_mpc(7,1),uv_d_mpc(8,1),'o','Color',C9,'LineWidth',lw*1.5);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
    'fontsize',fontsizeTicks)
title({'(b)'},'fontsize',fontsizeTitel,'interpreter','latex')
xlabel('$^p u~[pixels]$','interpreter','latex','fontsize',fontsizeLabel1)
ylabel('$^p v~[pixels]$','interpreter','latex','fontsize',fontsizeLabel1)
%title({'a) SEIR system identification: DMD vs. SINDy';''},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$^p\xi_{1}$','$^p\xi_{2}$','$^p\xi_{3}$','$^p\xi_{4}$','$^p\xi_{ref}$'},'interpreter','latex','fontsize',fontsizeLegend)
xlim([-uo uo])
ylim([-vo vo])
set(gca,'Ydir','reverse')
print -dpng System_movement_ibvs
print -depsc System_movement_ibvs

figure
set(gcf, 'PaperUnits', 'inches');
%set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');
box on
plot(t(1:length(he1_mpc)),he1_mpc(1,:),'-','Color',C18,'LineWidth',lw*2.7); hold on
plot(t(1:length(he1_mpc)),he2_mpc(1,:),'-','Color',C20,'LineWidth',lw*2.7); hold on
plot(t(1:length(he1_mpc)),he3_mpc(1,:),'-','Color',C22,'LineWidth',lw*2.7); hold on
plot(t(1:length(he1_mpc)),he4_mpc(1,:),'-','Color',C24,'LineWidth',lw*2.7); hold on

plot(t(1:length(he1_mpc)),he1_mpc(1,:),'-','Color',C19,'LineWidth',lw*0.8); hold on
plot(t(1:length(he1_mpc)),he2_mpc(1,:),'-','Color',C21,'LineWidth',lw*0.8); hold on
plot(t(1:length(he1_mpc)),he3_mpc(1,:),'-','Color',C23,'LineWidth',lw*0.8); hold on
plot(t(1:length(he1_mpc)),he4_mpc(1,:),'-','Color',C25,'LineWidth',lw*0.8); hold on
xlim([0 89])
ylim([0 140])


grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[pixels]$','interpreter','latex','fontsize',fontsizeLabel)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$||^p\tilde{\xi}_{1}||$','$||^p\tilde{\xi}_{2}||$','$||^p\tilde{\xi}_{3}||$','$||^p\tilde{\xi}_{4}||$'},'interpreter','latex','fontsize',fontsizeLegend)
print -dpng error_movement_ibvs
print -depsc error_movement_ibvs

figure
set(gcf, 'PaperUnits', 'inches');
%set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');
box on
plot(t(1:length(U_mpc)),U_mpc(1,:),'-','Color',C19,'LineWidth',lw*1.2); hold on
plot(t(1:length(U_mpc)),U_mpc(2,:),'-','Color',C21,'LineWidth',lw*1.2); hold on
plot(t(1:length(U_mpc)),U_mpc(3,:),'-','Color',C23,'LineWidth',lw*1.2); hold on
plot(t(1:length(U_mpc)),U_mpc(4,:),'-','Color',C25,'LineWidth',lw*1.2); hold on
xlim([0 89])
ylim([-0.4 0.3])
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s][rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(a)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\mu_{lref}$','$\mu_{mref}$','$\mu_{nref}$','$\omega_{ref}$'},'interpreter','latex','fontsize',fontsizeLegend)
print -dpng control_ibvs
print -depsc control_ibvs