% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % |Roberto Oliveira%%%%%%%%%%%%%%%%%
% % |Simula navegação de um robot móvel. Utilizador define ganhos do
% % |controlador e de repulsão/atracção assim como pose inicial e final
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;clear;close all;
global xview yview POD
%% Window dialog requesting parameters
prompt = {'[Kv,Ki,Ks]','[X,Y,Theta] actual','[X,Y] objectivo','FOV','POD'};
window_title = 'Escolha de parâmetros';
dims = [1 100]; % Dialog window size
defaultinput = {'[10.6,0.1,19.8]';'[20,150,0]';'[200,150]';'[315,60]';'[115]'}; % Default Input Values
inputs = inputdlg(prompt, window_title, dims, defaultinput);
K =  str2num(cell2mat(inputs(1))); % convert to string array and to lower case
pose_actual = str2num(cell2mat(inputs(2)));
pose_objectivo = str2num(cell2mat(inputs(3))); 
view = str2num(cell2mat(inputs(4))); 
POD = str2num(cell2mat(inputs(5))); 
xview = view(1);
yview = view(2);
global alfa
alfa = 36;
sector=zeros(1,alfa);
figure;
h=circle_det(0,0,7);
circle_params=[h.XData h.XData;
               h.YData h.YData;
               zeros(1,length(h.XData)) ones(1,length(h.XData))*7.5;
               ones(1,2*length(h.XData))];
hold on
grid on
h=fill3(circle_params(1,:),circle_params(2,:),circle_params(3,:),'r');

[x_n,y_n,t_n,h]=robot_goto(pose_actual,pose_objectivo,K,h,circle_params);


[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[20,100],K,h,circle_params);
% disp('done')
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[30 170],K,h,circle_params);
% disp('done')
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[50 180],K,h,circle_params);
% disp('done')
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[20 160],K,h,circle_params);
% disp('done')
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[80 90],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[160 40],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[20 165],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[200 365],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[80 80],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[350 350],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[120 350],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[380 50],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[50 250],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[150 300],K,h,circle_params);
% [x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[150 300],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[110 200],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[375 315],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[80 90],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[50 75],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[365 40],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[70 350],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[225 75],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[170 370],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[345 85],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[20 140],K,h,circle_params);
[x_n,y_n,t_n,h]=robot_goto([x_n,y_n,t_n],[140 90],K,h,circle_params);
function [x,y,theta_plot,h] = robot_goto(pose_actual,pose_objectivo,K,h,circle_params)
%% Ganhos
kv=K(1);
ki=K(2); 
ks=K(3); 


%% Preparação de variáveis
i=1;
x_store(i)=0;
y_store(i)=0;
erro_actual=0;
sample_time=0.05;
ponto_objectivo_x_world=pose_objectivo(1);
ponto_objectivo_y_world=pose_objectivo(2);
[obj_x,obj_y]=world2cell(ponto_objectivo_x_world,ponto_objectivo_y_world);
F=0;
Fr_x=0;
Fr_y=0;
x_actual=pose_actual(1);
y_actual=pose_actual(2);
theta_actual=pose_actual(3);

%% Construção de matriz mapa
M=zeros(80,80);
M=imcomplement(M);

%% Criação das paredes
for j=1:80
    M(j,1)=~1;
    M(j,80)=~1;
    M(1,j)=~1;
    M(80,j)=~1;
end

%% Criação de duas caixas com origem em (X,Y)=(70,30) com dimensões 20x20 e (X,Y)=(30,10) com dimensões 10x30
ob_x=[110 50 40 200 300 350 160  95 175 250 145 315];
ob_y=[70  0 120 100 0   250 210 255 325 200 150 100];
% ob_x=[50 300 300 300 300 300 300  300 300 300 300 300];
% ob_y=[85  300 300 300 300 300 300 300 300 300 300 300];
ob_dimx=[20 20 20 20 30 20 30 25 45 30 15 30];
ob_dimy=[20 30 20 20 50 40 40 30 25 60 15 20];
[M,xvec,yvec]=create_obstacle([ob_x(1) ob_y(1)],ob_dimx(1),ob_dimy(1),M);
[M,xvec2,yvec2]=create_obstacle([ob_x(2) ob_y(2)],ob_dimx(2),ob_dimy(2),M);
[M,xvec3,yvec3]=create_obstacle([ob_x(3) ob_y(3)],ob_dimx(3),ob_dimy(3),M);
[M,xvec4,yvec4]=create_obstacle([ob_x(4) ob_y(4)],ob_dimx(4),ob_dimy(4),M);
[M,xvec5,yvec5]=create_obstacle([ob_x(5) ob_y(5)],ob_dimx(5),ob_dimy(5),M);
[M,xvec6,yvec6]=create_obstacle([ob_x(6) ob_y(6)],ob_dimx(6),ob_dimy(6),M);
[M,xvec7,yvec7]=create_obstacle([ob_x(7) ob_y(7)],ob_dimx(7),ob_dimy(7),M);
[M,xvec8,yvec8]=create_obstacle([ob_x(8) ob_y(8)],ob_dimx(8),ob_dimy(8),M);
[M,xvec9,yvec9]=create_obstacle([ob_x(9) ob_y(9)],ob_dimx(9),ob_dimy(9),M);
[M,xvec10,yvec10]=create_obstacle([ob_x(10) ob_y(10)],ob_dimx(10),ob_dimy(10),M);
[M,xvec11,yvec11]=create_obstacle([ob_x(11) ob_y(11)],ob_dimx(11),ob_dimy(11),M);
[M,xvec12,yvec12]=create_obstacle([ob_x(12) ob_y(12)],ob_dimx(12),ob_dimy(12),M);

ob1_vertices=[ob_x(1) ob_x(1)+ob_dimx(1) ob_x(1)+ob_dimx(1) ob_x(1);
              ob_y(1) ob_y(1)            ob_y(1)+ob_dimy(1) ob_y(1)+ob_dimy(1);
              0 0 0 0];
ob2_vertices=[ob_x(2) ob_x(2)+ob_dimx(2) ob_x(2)+ob_dimx(2) ob_x(2);
              ob_y(2) ob_y(2)            ob_y(2)+ob_dimy(2) ob_y(2)+ob_dimy(2);
              0 0 0 0];
ob3_vertices=[ob_x(3) ob_x(3)+ob_dimx(3) ob_x(3)+ob_dimx(3) ob_x(3);
              ob_y(3) ob_y(3)            ob_y(3)+ob_dimy(3) ob_y(3)+ob_dimy(3);
              0 0 0 0];     
ob4_vertices=[ob_x(4) ob_x(4)+ob_dimx(4) ob_x(4)+ob_dimx(4) ob_x(4);
              ob_y(4) ob_y(4)            ob_y(4)+ob_dimy(4) ob_y(4)+ob_dimy(4);
              0 0 0 0];
ob5_vertices=[ob_x(5) ob_x(5)+ob_dimx(5) ob_x(5)+ob_dimx(5) ob_x(5);
              ob_y(5) ob_y(5)            ob_y(5)+ob_dimy(5) ob_y(5)+ob_dimy(5);
              0 0 0 0];
ob6_vertices=[ob_x(6) ob_x(6)+ob_dimx(6) ob_x(6)+ob_dimx(6) ob_x(6);
              ob_y(6) ob_y(6)            ob_y(6)+ob_dimy(6) ob_y(6)+ob_dimy(6);
              0 0 0 0]; 
ob7_vertices=[ob_x(7) ob_x(7)+ob_dimx(7) ob_x(7)+ob_dimx(7) ob_x(7);
              ob_y(7) ob_y(7)            ob_y(7)+ob_dimy(7) ob_y(7)+ob_dimy(7);
              0 0 0 0];
ob8_vertices=[ob_x(8) ob_x(8)+ob_dimx(8) ob_x(8)+ob_dimx(8) ob_x(8);
              ob_y(8) ob_y(8)            ob_y(8)+ob_dimy(8) ob_y(8)+ob_dimy(8);
              0 0 0 0];
ob9_vertices=[ob_x(9) ob_x(9)+ob_dimx(9) ob_x(9)+ob_dimx(9) ob_x(9);
              ob_y(9) ob_y(9)            ob_y(9)+ob_dimy(9) ob_y(9)+ob_dimy(9);
              0 0 0 0];  
ob10_vertices=[ob_x(10) ob_x(10)+ob_dimx(10) ob_x(10)+ob_dimx(10) ob_x(10);
              ob_y(10) ob_y(10)            ob_y(10)+ob_dimy(10) ob_y(10)+ob_dimy(10);
              0 0 0 0];      
ob11_vertices=[ob_x(11) ob_x(11)+ob_dimx(11) ob_x(11)+ob_dimx(11) ob_x(11);
              ob_y(11) ob_y(11)            ob_y(11)+ob_dimy(11) ob_y(11)+ob_dimy(11);
              0 0 0 0];      
ob12_vertices=[ob_x(12) ob_x(12)+ob_dimx(12) ob_x(12)+ob_dimx(12) ob_x(12);
              ob_y(12) ob_y(12)            ob_y(12)+ob_dimy(12) ob_y(12)+ob_dimy(12);
              0 0 0 0];      
for p=1:100
clear sector
alfa=36;
sector=zeros(1,alfa);
P_atual=[x_actual y_actual 1]';

%Transformação de coordenadas do mundo para coordenadas célula
[Cx,Cy]=world2cell(x_actual,y_actual);

%% Força de repulsão
r=1;
for i = Cx-5:1:Cx+5%Zona activa de 25cm em cada direcção do robot, 25cm->5 células
    for j = Cy-5:1:Cy+5%Percorre zona active
        if i < 1 || i > 80 || j < 1 || j > 80
        else
            if M(i,j)==0 % = a 0 se houver uma célula com obstáculo na zona activa
                d=sqrt((Cx-i)^2+(Cy-j)^2);
                beta=atan2((j-Cy),(i-Cx));
                beta=rad2deg(wrapTo2Pi(beta-pi/2));
                %dmax=sqrt(50)
                b=2; a=2*sqrt(50);
                m=ceil((~M(i,j))^2*(a-b*d));
                k=ceil(beta/10);
                if k==0 
                    k=1;
                end
                sector(k)=sector(k)+m;
%                 r=r+1;
            end
        end
    end
end
sector = filter_sector(sector);
% figure;bar(sector)
% figure;bar([sector sector sector])%passar para 0 a 2pi??
beta=atan2((obj_y-Cy),(obj_x-Cx));% 2*pi - 36
beta=rad2deg(wrapTo2Pi(beta-pi/2));% x - 38
k=ceil(beta/10);%x=2*pi*38/36;x=wrapTo2Pi(x);->Nova regra de 3 simples                              VALOR A ALTERAR
if k==0                                      %2*pi-36                                               ^^
    k=1;                                     %x - xnovo ->>> xnovo=x*36/(2*pi); ->>> FORMULA: x=2*pi*37/36;x=wrapTo2Pi(x);final=x*36/(2*pi) -> ainda se tem de criar condição para zero!
end

chosen_sector(p) = sector_alg([sector sector sector],k+36);
chosen_sector(p)=2*pi*chosen_sector(p)/36;
chosen_sector(p)=wrapTo2Pi(chosen_sector(p));chosen_sector(p)=chosen_sector(p)*36/(2*pi);
[chosen_sector(p) k]

sector_d=chosen_sector(p)*10-5;
% chosen_sector
% sector_d
ponto_obj_x=x_actual+cosd(sector_d)*5;
ponto_obj_y=y_actual+sind(sector_d)*5;

%% Controlador
erro_anterior=erro_actual;
erro_actual=sqrt((ponto_obj_x-x_actual)^2+(ponto_obj_y-y_actual)^2)-0;%ultimo nmr é distancia
erro_integral = erro_anterior+erro_actual;
velocidade=kv*erro_actual+ki*erro_integral;
theta_desejado=atan2((ponto_obj_y-y_actual),(ponto_obj_x-x_actual));
theta_diff=theta_desejado-theta_actual;
theta_diff=atan2(sin(theta_diff),cos(theta_diff));
omega=ks*theta_diff;
omega_E=(velocidade-(14.1/2)*omega)/3.5;
omega_R=(velocidade+(14.1/2)*omega)/3.5;
%% Simulação de movimento
if (omega==0.0)
   x=x_actual+0.05*velocidade*cos(theta_actual+omega*0.05/2);
   y=y_actual+0.05*velocidade*sin(theta_actual+omega*0.05/2);
    
else
    x=x_actual+0.05*velocidade*sin(omega*0.05/2)*cos(theta_actual+omega*0.05/2)/(omega*0.05/2);
    y=y_actual+0.05*velocidade*sin(omega*0.05/2)*sin(theta_actual+omega*0.05/2)/(omega*0.05/2);
end
theta=theta_actual+omega*0.05;
theta_store(p)=theta;
theta_actual=theta;
x_store(p)=x;
y_store(p)=y;
x_actual=x;
y_actual=y;
omega_e_rec(p,1)=omega_E;
omega_d_rec(p,1)=omega_R;
t(p)=p*0.05;
if Cx==obj_x && Cy==obj_y
    break
end
end
% disp('Velocidades enviadas para rodas')
% [omega_e_rec omega_d_rec]
% figure;plot(x_store,y_store);
% hold on;plot(xvec,yvec);plot(xvec2,yvec2);plot(xvec3,yvec3);
x=x_store(p-1);
y=y_store(p-1);
theta_plot=theta_store(p-1);
robot_graph(x_store(1,1:length(x_store)-1),y_store(1,1:length(y_store)-1),theta_store(1,1:length(theta_store)-1),h,ob1_vertices,ob2_vertices,ob3_vertices,ob4_vertices,ob5_vertices,ob6_vertices,ob7_vertices,ob8_vertices,ob9_vertices,ob10_vertices,ob11_vertices,ob12_vertices,circle_params);
% robot_graph(x_store(1,1:length(x_store)-1),y_store(1,1:length(y_store)-1),theta_store(1,1:length(theta_store)-1),h,ob1_vertices,ob2_vertices,ob3_vertices,circle_params);
end

function [Cx,Cy] = world2cell(Cx,Cy)
T= [0 -1 80;
    1 0 1;
    0 0 1];
robot_P=[floor(Cx/5) floor(Cy/5) 1]';

Zona_activa=T*robot_P;
Cx=Zona_activa(1,1);
Cy=Zona_activa(2,1);
end

function [M,xvec,yvec] = create_obstacle(point,xdim,ydim,M)
xmin=point(1);
ymin=point(2);
xmax=xmin+xdim;
ymax=ymin+ydim;

[xmin,ymin]=world2cell(xmin,ymin);
[xmax,ymax]=world2cell(xmax,ymax);
 for i=xmax:xmin
     M(i,ymin)=~1;
     M(i,ymax)=~1;
 end
  for i=ymin:ymax
     M(xmin,i)=~1;
     M(xmax,i)=~1;
  end

yvec=ones(1,xdim)*point(2);
xvec=(point(1)+1):(point(1)+xdim);
yvec=[yvec (point(2)+1):(point(2)+ydim)];
xvec=[xvec ones(1,ydim)*(point(1)+xdim)];
yvec=[yvec ones(1,xdim)*(point(2)+ydim)];
xvec=[xvec (point(1)+1):(point(1)+xdim)];
yvec=[yvec point(2):(point(2)+ydim-1)];
xvec=[xvec ones(1,ydim)*point(1)];

end

function h = robot_graph(x,y,theta,h,ob1_vertices,ob2_vertices,ob3_vertices,ob4_vertices,ob5_vertices,ob6_vertices,ob7_vertices,ob8_vertices,ob9_vertices,ob10_vertices,ob11_vertices,ob12_vertices,circle_params)
% function h = robot_graph(x,y,theta,h,ob1_vertices,ob2_vertices,ob3_vertices,circle_params)
global xview yview
% h=circle(0,0,8);
% figure;
hold on;view(xview,yview)
obstacle1=fill3(ob1_vertices(1,:),ob1_vertices(2,:),ob1_vertices(3,:),'b');
obstacle2=fill3(ob2_vertices(1,:),ob2_vertices(2,:),ob2_vertices(3,:),'y');
obstacle3=fill3(ob3_vertices(1,:),ob3_vertices(2,:),ob3_vertices(3,:),'y');
obstacle4=fill3(ob4_vertices(1,:),ob4_vertices(2,:),ob4_vertices(3,:),'b');
obstacle5=fill3(ob5_vertices(1,:),ob5_vertices(2,:),ob5_vertices(3,:),'y');
obstacle6=fill3(ob6_vertices(1,:),ob6_vertices(2,:),ob6_vertices(3,:),'y');
obstacle7=fill3(ob7_vertices(1,:),ob7_vertices(2,:),ob7_vertices(3,:),'b');
obstacle8=fill3(ob8_vertices(1,:),ob8_vertices(2,:),ob8_vertices(3,:),'y');
obstacle9=fill3(ob9_vertices(1,:),ob9_vertices(2,:),ob9_vertices(3,:),'y');
obstacle10=fill3(ob10_vertices(1,:),ob10_vertices(2,:),ob10_vertices(3,:),'b');
obstacle11=fill3(ob11_vertices(1,:),ob11_vertices(2,:),ob11_vertices(3,:),'b');
obstacle12=fill3(ob12_vertices(1,:),ob12_vertices(2,:),ob12_vertices(3,:),'b');
axis([0 400 0 400 0 50])

for i=1:length(x)
P = position_calc(x(i),y(i),0,theta(i));

circle_params_aux=P*circle_params;
set(h,'XData',circle_params_aux(1,:));
set(h,'YData',circle_params_aux(2,:));
set(h,'ZData',circle_params_aux(3,:));
pause(0.05)
end

end

function h = circle_det(x,y,r)
hold on
th = 0:0.2:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
hold off
end

function M = position_calc(x,y,z,theta)
T=[cos(theta) -sin(theta) 0 x;
   sin(theta) cos(theta) 0 y;
   0 0 1 z;
   0 0 0 1];
M=eye(4)*T;
end


function sector_filtered = filter_sector(sector)
sector_filtered=zeros(1,length(sector));
for k = 1 : length(sector)
    if k == 1
        sector_filtered(k) = sector(33)+2*sector(34)+3*sector(35)+4*sector(36)+5*sector(k)+4*sector(k+1)+3*sector(k+2)+2*sector(k+3)+sector(k+4);
    elseif k == 2
         sector_filtered(k) = sector(34)+2*sector(35)+3*sector(36)+4*sector(k-1)+5*sector(k)+4*sector(k+1)+3*sector(k+2)+2*sector(k+3)+sector(k+4);
    elseif k == 3
         sector_filtered(k) = sector(35)+2*sector(36)+3*sector(k-2)+4*sector(k-1)+5*sector(k)+4*sector(k+1)+3*sector(k+2)+2*sector(k+3)+sector(k+4);
    elseif k == 4
         sector_filtered(k) = sector(36)+2*sector(k-3)+3*sector(k-2)+4*sector(k-1)+5*sector(k)+4*sector(k+1)+3*sector(k+2)+2*sector(k+3)+sector(k+4); 
    elseif k == 33 
         sector_filtered(k) = sector(k-4)+2*sector(k-3)+3*sector(k-2)+4*sector(k-1)+5*sector(k)+4*sector(k+1)+3*sector(k+2)+2*sector(k+3)+sector(1);      
    elseif k == 34 
         sector_filtered(k) = sector(k-4)+2*sector(k-3)+3*sector(k-2)+4*sector(k-1)+5*sector(k)+4*sector(k+1)+3*sector(k+2)+2*sector(1)+sector(2);      
    elseif k == 35
         sector_filtered(k) = sector(k-4)+2*sector(k-3)+3*sector(k-2)+4*sector(k-1)+5*sector(k)+4*sector(k+1)+3*sector(1)+2*sector(2)+sector(3);      
    elseif k == 36
         sector_filtered(k) = sector(k-4)+2*sector(k-3)+3*sector(k-2)+4*sector(k-1)+5*sector(k)+4*sector(1)+3*sector(2)+2*sector(3)+sector(4);      
    else
         sector_filtered(k) = sector(k-4)+2*sector(k-3)+3*sector(k-2)+4*sector(k-1)+5*sector(k)+4*sector(k+1)+3*sector(k+2)+2*sector(k+3)+sector(k+4);      

    end
        
end

end
% 
function chosen_sector = sector_alg(sector,sot)%sot -> sector of target
% pass_check=0;
Smax_check=1;
Smax = 14;%wide valley
narrow_min=8;
% POD = 35;
global POD
i=1;
check_neg=0;
check_pos=0;
check_free=0;
kv1=0;
kv2=0;
chosen_sector=-1;

%is path to sector free?
for i=sot-2:sot+2
    if sector(i)<POD
        check_free=check_free+1;
    end
end

if check_free == 5 %target sector is clear of obstacles
    chosen_sector = sot;
    return
end

% i=1;
% while(Smax_check==1) %% wide valley verification
%     %target sector is not free of obstacles
%     
%     if sector(sot-i)<POD && check_neg ~= -1 %will check if neighbouring sectors are clear of obstacles and wide valley
%         check_neg=check_neg+1;
%     else
%         check_neg=-1;
%     end
%     
%     if sector(sot+i)<POD && check_pos ~= -1
%         check_pos=check_pos+1;
%     else 
%         check_pos =-1;
%     end
%         
%     if check_pos >= Smax %|| check_neg >= Smax 
%         chosen_sector = sot + Smax/2;
%         break
%     end
%     
%     if check_neg >= Smax %|| check_neg >= Smax 
%         chosen_sector = sot - Smax/2;
%         break
%     end 
%     
%     if check_pos == -1 && check_neg == -1
%         Smax_check = 0;
%         break
%     end
%     i=i+1;
% end
% if chosen_sector~=-1
%     return
% end
%if program gets to this point means sector of target isn't clear and there
%is no wide valley

for i=0:length(sector)
    kv1=0;
    kv2=0;
    if sector(sot+i)<=POD %candidate sector
        kv1=sot+i;
        for j = i+1:i+narrow_min
            if sector(sot+j)>POD%n encontrou sector medio
                break
            elseif j == i+narrow_min
                kv2 = sot+j;
                while sector(sot+j)<=POD
                    kv2 = sot+j;
                    if j == i+Smax
                        kv2=sot+j;
%                         pass_check=1;
                        break
                    end
                    j=j+1;
                end
                    
            end
        end
    end
     
    if kv1~=0 && kv2~=0
        chosen_sector_pos=(kv1+kv2)/2;
        break
    end
end

for i=0:length(sector)
    kv1=0;
    kv2=0;
    if sector(sot-i)<=POD %candidate sector
        kv1=sot-i;
        for j = i+1:i+narrow_min
            if sector(sot-j)>POD%n encontrou sector medio
                break
            elseif j == i+narrow_min
                kv2 = sot-j;
                while sector(sot-j) <= POD
                    kv2=sot-j; 
                   if j==i+Smax
                      kv2=sot-j; 
%                       pass_check=1;
                      break
                   end
                    j=j+1;
                end
            end
        end
    end
     
    if kv1~=0 && kv2~=0
        chosen_sector_neg=(kv1+kv2)/2;
        break
    end
end

% if pass_check==0
%     disp('not wide')
% % else 
% %     disp('wide')
% end


if abs(chosen_sector_neg-sot)<abs(chosen_sector_pos-sot)
    chosen_sector=chosen_sector_neg;
else
    chosen_sector=chosen_sector_pos;
end

return
end
