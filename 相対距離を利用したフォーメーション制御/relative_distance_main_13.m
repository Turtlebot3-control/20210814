% relative_distance_main_13.m
% 202011
% 測定ノイズ

clear;
close all;

% number of vehicles
%  N = 5;
  N = 7;

%�?質点な�?0 移動ロボットな�?1
ver = 0; 

% initial state ワールド座�?
%  P=[100 200 10 -10 -20;
%     0 100 10*sqrt(3) 10*sqrt(3) 0];
%  x=P(1,:); 
%  y=P(2,:);
  rng(6); x = 200*rand(1,N);
  rng(7); y = 200*rand(1,N);
%  rng(6); x = 25*rand(1,N);
%  rng(7); y = -20*rand(1,N);

 x(1,1)=0;
 y(1,1)=0;
 theta=zeros(1,N);
%theta=[0 pi/3 -pi/4 2*pi/3 pi];
%theta=[pi/10 pi/5 -pi/4 -pi/3 pi];
% rng(4); theta = rand(1,N);
% theta(1)=0;



%隣接行�??
% A=[0 0 0 0 0;
%    1 0 0 0 0;
%    1 1 0 0 0;
%    1 0 1 0 0;
%    1 0 0 1 0];
% A=[0 0 0 0 0 0 0;
%    1 0 0 0 0 0 0;
%    1 1 0 0 0 0 0;
%    1 0 1 0 0 0 0;
%    1 0 0 1 0 0 0;
%    1 0 0 0 1 0 0;
%    1 0 0 0 0 1 0];

dd=zeros(N,N); %目標�?�相対距離
for i=2:N
   dd(i,1)=60;
   dd(i,i-1)=60;
end

sgn=1; %目標�?�隣接�?�? 符号を変えれ�?�反転する

%% 
rx=zeros(N,N);   %相対位置�?行が基�?  
ry=zeros(N,N);
dx=zeros(1,N);    %目標位置
dy=zeros(1,N);
ux=zeros(1,N);
uy=zeros(1,N);
K=2; % ゲイン
aex=0;
aey=0;
gamma=0.8;
phi=zeros(1,N); %変角�?�積�?
vex=zeros(1,N);
vey=zeros(1,N);
D21=sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
D31=sqrt((x(3)-x(1))^2+(y(3)-y(1))^2);
D41=sqrt((x(4)-x(1))^2+(y(4)-y(1))^2);
D51=sqrt((x(5)-x(1))^2+(y(5)-y(1))^2);
D61=sqrt((x(6)-x(1))^2+(y(6)-y(1))^2);
D71=sqrt((x(7)-x(1))^2+(y(7)-y(1))^2);

%% 目標�?�相対距離に収束するために
    ts = [0 8];
    if ver == 1
        XX0 = [x;y;theta;phi;vex;vey];
        initial = XX0(:);
        [T,XX] = ode45(@twdd_vehicle_w,ts,initial,[],N,rx,ry,dx,dy,dd,sgn,K,ver,aex,aey,gamma);
        X=XX(:,1:6:end);
        Y=XX(:,2:6:end);
        Theta= XX(:,3:6:end);
    elseif ver == 0
          vex(1)=10;
        XX0 = [x;y;theta;vex;vey];
        initial = XX0(:);
        [T,XX] = ode45(@twdd_vehicle_w,ts,initial,[],N,rx,ry,dx,dy,dd,sgn,K,ver,aex,aey,gamma);
        X=XX(:,1:5:end);
        Y=XX(:,2:5:end);
        Theta=XX(:,3:5:end);
    end
    
    
%% 本当に目標�?�相対距離に収束して�?るか確認するとき用
%  [dx,dy]=dxdy3(dx,dy,rx,ry,dd,sgn,N); %ローカル座標�?�目標位置を求める関数
 e=zeros(1,N);
 for i=1:N
 e(i)=sqrt((dx(i))^2+(dy(i))^2);
 end
 
 
%% アニメーションとグラ�?
%アニメーション
% figure(1)
% X(end,:)=[];
% Y(end,:)=[];
% Theta(end,:)=[];
% ii = 1; jj = 1; k=0;
% while ii <length(X(:,1))
%  plot(X(ii,:),Y(ii,:),'ko','MarkerFaceColor','g','MarkerSize',8,'LineWidth',2)
%  hold on
%  for k=1:N
%  p1x(k)=X(ii,k)+cos(Theta(ii,k))*10;
%  p1y(k)=Y(ii,k)+sin(Theta(ii,k))*10;
%  p2x(k)=X(ii,k)+cos(Theta(ii,k))*(-5)-sin(Theta(ii,k))*5;
%  p2y(k)=Y(ii,k)+sin(Theta(ii,k))*(-5)+cos(Theta(ii,k))*5;
%  p3x(k)=X(ii,k)+cos(Theta(ii,k))*(-5)-sin(Theta(ii,k))*(-5);
%  p3y(k)=Y(ii,k)+sin(Theta(ii,k))*(-5)+cos(Theta(ii,k))*(-5);
%  end
%  for k=1:N
%  plot([p1x(k),p2x(k)],[p1y(k),p2y(k)],'b')
%  plot([p1x(k),p3x(k)],[p1y(k),p3y(k)],'b')
%  plot([p2x(k),p3x(k)],[p2y(k),p3y(k)],'b')
%  end
%  hold off
%  
%  grid on
%  xlabel('x [cm]','Fontsize',20);
%  ylabel('y [cm]','Fontsize',20);
%   axis([-100 400 -100 200])
% set(gcf, 'Position', [300 30 500 300]);
%  if ii == 1
%   while jj < 30
%    drawnow;
%    plot_movie(jj) = getframe(gcf);
%    jj = jj+1;
%   end
%   pause(2);
%   ii = ii+1;
%  else
%   ii=ii+1;
%  end
%  drawnow;
%  plot_movie(jj) = getframe(gcf);
%  jj = jj+1;
%  end

%グラ�?
figure(2)
 for k=1:N
 f1x(k)=X(1,k)+cos(Theta(1,k))*10;
 f1y(k)=Y(1,k)+sin(Theta(1,k))*10;
 f2x(k)=X(1,k)+cos(Theta(1,k))*(-5)-sin(Theta(1,k))*5;
 f2y(k)=Y(1,k)+sin(Theta(1,k))*(-5)+cos(Theta(1,k))*5;
 f3x(k)=X(1,k)+cos(Theta(1,k))*(-5)-sin(Theta(1,k))*(-5);
 f3y(k)=Y(1,k)+sin(Theta(1,k))*(-5)+cos(Theta(1,k))*(-5);
 end
 for k=1:N
 p1x(k)=X(end,k)+cos(Theta(end,k))*10;
 p1y(k)=Y(end,k)+sin(Theta(end,k))*10;
 p2x(k)=X(end,k)+cos(Theta(end,k))*(-5)-sin(Theta(end,k))*5;
 p2y(k)=Y(end,k)+sin(Theta(end,k))*(-5)+cos(Theta(end,k))*5;
 p3x(k)=X(end,k)+cos(Theta(end,k))*(-5)-sin(Theta(end,k))*(-5);
 p3y(k)=Y(end,k)+sin(Theta(end,k))*(-5)+cos(Theta(end,k))*(-5);
 end

plot(X(:,1:N),Y(:,1:N),'LineWidth',1.5)
hold on
for k=1:N
 plot([f1x(k),f2x(k)],[f1y(k),f2y(k)],'k','LineWidth',1)
 plot([f1x(k),f3x(k)],[f1y(k),f3y(k)],'k','LineWidth',1)
 plot([f2x(k),f3x(k)],[f2y(k),f3y(k)],'k','LineWidth',1)
 plot([p1x(k),p2x(k)],[p1y(k),p2y(k)],'b','LineWidth',1.5)
 plot([p1x(k),p3x(k)],[p1y(k),p3y(k)],'b','LineWidth',1.5)
 plot([p2x(k),p3x(k)],[p2y(k),p3y(k)],'b','LineWidth',1.5)
 end
 hold off
axis([-100 800 -100 200])
set(gcf, 'Position', [300 30 900 300]);
 grid 
 legend('1','2','3','4','5','6','7');
 set(gca,'FontSize',20);
xlabel('x [cm]','Fontsize',20)
ylabel('y [cm]','Fontsize',20)

figure(3)
    d21=sqrt((X(:,2)-X(:,1)).^2+(Y(:,2)-Y(:,1)).^2);
    d31=sqrt((X(:,3)-X(:,1)).^2+(Y(:,3)-Y(:,1)).^2);
    d41=sqrt((X(:,4)-X(:,1)).^2+(Y(:,4)-Y(:,1)).^2);
    d51=sqrt((X(:,5)-X(:,1)).^2+(Y(:,5)-Y(:,1)).^2);
    d61=sqrt((X(:,6)-X(:,1)).^2+(Y(:,6)-Y(:,1)).^2);
    d71=sqrt((X(:,7)-X(:,1)).^2+(Y(:,7)-Y(:,1)).^2);

    DD=60*ones(length(T),1);
    plot(T,DD,'LineWidth',1.2)
    hold on
    plot(T,d21,'LineWidth',1.2)
    plot(T,d31,'LineWidth',1.2)
    plot(T,d41,'LineWidth',1.2)
    plot(T,d51,'LineWidth',1.2)
    plot(T,d61,'LineWidth',1.2)
    plot(T,d71,'LineWidth',1.2)
    hold off
    grid on
     legend('目標�?�距離 d_{12}','1,2間�?�距離 ||^{2}p_{1}||','1,3間�?�距離 ||^{3}p_{1}||','1,4間�?�距離 ||^{4}p_{1}||','1,5間�?�距離 ||^{5}p_{1}||','1,6間�?�距離 ||^{6}p_{1}||','1,7間�?�距離 ||^{7}p_{1}||');
    set(gca,'FontSize',13);
    xlabel('時間 t [s]','Fontsize',20);
    ylabel('相対距離 [cm]','Fontsize',20);
    

    