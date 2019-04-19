% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % |Roberto Oliveira%%%%%%%%%%%%%%%%%
% % |Simula navegação de um robot móvel. Utilizador define ganhos do
% % |controlador e de repulsão/atracção assim como pose inicial e final
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;clear;close all;

%% Window dialog requesting parameters
prompt = {'[Kv,Ki,Ks]', '[Fr,Fa]','[X,Y,Theta] actual','[X,Y] objectivo'};
window_title = 'Escolha de parâmetros';
dims = [1 100]; % Dialog window size
defaultinput = {'[4.1,0.02,13.56]'; '[60,14.5]';'[30,30,0]';'[80,80]'}; % Default Input Values
inputs = inputdlg(prompt, window_title, dims, defaultinput);
K =  str2num(cell2mat(inputs(1))); % convert to string array and to lower case
Fcra =  str2num(cell2mat(inputs(2)));
pose_actual = str2num(cell2mat(inputs(3)));
pose_objectivo = str2num(cell2mat(inputs(4))); 
figure;
robot_goto(pose_actual,pose_objectivo,K,Fcra);
robot_goto([pose_objectivo pi],pose_actual,K,Fcra);
robot_goto([pose_actual pi],[150 150],K,Fcra);
% robot_goto([pose_objectivo pi],pose_actual,K,Fcra);
% robot_goto([100,35,0],[20 140],K,Fcra);
% robot_goto([20 140 pi/2],[80 90],K,Fcra);
% robot_goto([80 90 0],[160 40],K,Fcra);
% robot_goto([160 40 0],[20 165],K,Fcra);
function robot_goto(pose_actual,pose_objectivo,K,Fcra)
%% Ganhos
kv=K(1);
ki=K(2); 
ks=K(3); 
Fcr=Fcra(1); %Força constante repulsiva
Fca=Fcra(2);

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
ob_x=[50 100 200];
ob_y=[50 100 200];
ob_dimx=[20 20 0];
ob_dimy=[20 20 0];
[M,xvec,yvec]=create_obstacle([ob_x(1) ob_y(1)],ob_dimx(1),ob_dimy(1),M);
[M,xvec2,yvec2]=create_obstacle([ob_x(2) ob_y(2)],ob_dimx(2),ob_dimy(2),M);
[M,xvec3,yvec3]=create_obstacle([ob_x(3) ob_y(3)],ob_dimx(3),ob_dimy(3),M);
ob1_vertices=[ob_x(1) ob_x(1)+ob_dimx(1) ob_x(1)+ob_dimx(1) ob_x(1);
              ob_y(1) ob_y(1)            ob_y(1)+ob_dimy(1) ob_y(1)+ob_dimy(1);
              0 0 0 0];
ob2_vertices=[ob_x(2) ob_x(2)+ob_dimx(2) ob_x(2)+ob_dimx(2) ob_x(2);
              ob_y(2) ob_y(2)            ob_y(2)+ob_dimy(2) ob_y(2)+ob_dimy(2);
              0 0 0 0];
ob3_vertices=[ob_x(3) ob_x(3)+ob_dimx(3) ob_x(3)+ob_dimx(3) ob_x(3);
              ob_y(3) ob_y(3)            ob_y(3)+ob_dimy(3) ob_y(3)+ob_dimy(3);
              0 0 0 0];          
for p=1:100
clear Fr_y Fr_x F
F=0;Fr_y=0;Fr_x=0;
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
                F(r)=Fcr*(~M(i,j))/d^2;
                Fr_x(r)=F(r)*(Cx-i)/d;
                Fr_y(r)=F(r)*(Cy-j)/d;
                r=r+1;
            end
        end
    end
end
Fr_y_total=sum(Fr_y);
Fr_x_total=sum(Fr_x);

%% força atracção
d=sqrt((Cx-obj_x)^2+(Cy-obj_y)^2);
Fa_x=Fca*((obj_x-Cx)/d);
Fa_y=Fca*((obj_y-Cy)/d);

F_total_x=Fa_x+Fr_x_total;
F_total_y=Fa_y+Fr_y_total;

%% Forças no referencial mundo
aux=F_total_x;
F_total_x=F_total_y;
F_total_y=-aux;
ponto_obj_x=x_actual+F_total_x;
ponto_obj_y=y_actual+F_total_y;

%% Controlador
erro_anterior=erro_actual;
erro_actual=sqrt((ponto_obj_x-x_actual)^2+(ponto_obj_y-y_actual)^2)-0;%ultimo nmr é distancia
% erro_integral=sample_time*(erro_anterior+erro_actual)/2;
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
% Cx
% obj_x
% Cy
% obj_y
if Cx==obj_x && Cy==obj_y
    break
end
end
disp('Velocidades enviadas para rodas')
[omega_e_rec omega_d_rec]
% figure;plot(x_store,y_store);
% hold on;plot(xvec,yvec);plot(xvec2,yvec2);plot(xvec3,yvec3);

robot_graph(x_store(1,1:length(x_store)-1),y_store(1,1:length(y_store)-1),theta_store(1,1:length(theta_store)-1),ob1_vertices,ob2_vertices,ob3_vertices)
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

[xmin,ymin]=world2cell(xmin,ymin)
[xmax,ymax]=world2cell(xmax,ymax)
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

function robot_graph(x,y,theta,ob1_vertices,ob2_vertices,ob3_vertices)
h=circle(0,0,8);
% figure;
hold on;view(315,65)
obstacle1=fill3(ob1_vertices(1,:),ob1_vertices(2,:),ob1_vertices(3,:),'b');
obstacle2=fill3(ob2_vertices(1,:),ob2_vertices(2,:),ob2_vertices(3,:),'y');
obstacle3=fill3(ob3_vertices(1,:),ob3_vertices(2,:),ob3_vertices(3,:),'y');


circle_params=[h.XData h.XData;
               h.YData h.YData;
               zeros(1,length(h.XData)) ones(1,length(h.XData))*20;
               ones(1,2*length(h.XData))];
hold on
h=fill3(circle_params(1,:),circle_params(2,:),circle_params(3,:),'r');
axis([0 200 0 200 0 200])
% pause(2)
for i=1:length(x)
P = position_calc(x(i),y(i),0,theta(i));

circle_params_aux=P*circle_params;
set(h,'XData',circle_params_aux(1,:))
set(h,'YData',circle_params_aux(2,:))
set(h,'ZData',circle_params_aux(3,:))
pause(0.05)
end
function h = circle(x,y,r)
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
end
