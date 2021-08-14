% relative_distance_main_12.m
% 202011
% 測定ノイズ

clear;
close all;

% number of vehicles
% N = 5;
  N = 7;

% initial state ワールド座標
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
rng(4); theta = rand(1,N);
theta(1)=0;



%隣接行列
% A=[0 0 0 0 0;
%    1 0 0 0 0;
%    1 1 0 0 0;
%    1 0 1 0 0;
%    1 0 0 1 0];
A=[0 0 0 0 0 0 0;
   1 0 0 0 0 0 0;
   1 1 0 0 0 0 0;
   1 0 1 0 0 0 0;
   1 0 0 1 0 0 0;
   1 0 0 0 1 0 0;
   1 0 0 0 0 1 0];

dd=zeros(N,N); %目標の相対距離
for i=2:N
   dd(i,1)=60;
   dd(i,i-1)=60;
end

sgn=1; %目標の隣接順序 符号を変えれば反転する

%% 
rx=zeros(N,N);   %相対位置　行が基準  
ry=zeros(N,N);
dx=zeros(1,N);    %目標位置
dy=zeros(1,N);
ux=zeros(1,N);
uy=zeros(1,N);
K=2; % ゲイン
%K=1;
tt=0.05; % ステップ時間
X=x;
Y=y;
a=1;
Theta=theta;
phi=zeros(1,N); %変角の積分
D21=sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
D31=sqrt((x(3)-x(1))^2+(y(3)-y(1))^2);
D41=sqrt((x(4)-x(1))^2+(y(4)-y(1))^2);
D51=sqrt((x(5)-x(1))^2+(y(5)-y(1))^2);
D61=sqrt((x(6)-x(1))^2+(y(6)-y(1))^2);
D71=sqrt((x(7)-x(1))^2+(y(7)-y(1))^2);

%% 目標の相対距離に収束するために
    
    for t=0:tt:30
    [rx,ry]=relative_position(x,y,theta,N,rx,ry); %相対位置を求める関数 最初だけワールド座標の情報(初期位置)を利用する．
    wrx=rx+wgn(N,N,10);
    wry=ry+wgn(N,N,10);
%      ux(1)=a*t;
    ux(1)=10;
    [drx,dry]=relative_velocity(A,ux,uy,theta,N); %隣接するエージェントの相対速度の平均
%     for i=2:N                                     
%         dd(i,1)=40+(ux(1)^2+uy(1)^2)^(1/2);  %目標の相対距離を動かす
%         dd(i,i-1)=40+(ux(1)^2+uy(1)^2)^(1/2);
%     end
%     [dfx,dfy]=tformation(A,a,N,rx,ry);
%     [dx,dy]=dxdy3(dx,dy,rx,ry,dd,sgn,N); %ローカル座標の目標位置を求める関数
     [dx,dy]=dxdy3(dx,dy,wrx,wry,dd,sgn,N); %ローカル座標の目標位置を求める関数
%      ux=dx*K+drx-dfx; %速度
%      uy=dy*K+dry-dfy; 
    ux=dx*K+drx; %速度
    uy=dy*K+dry; 
%     ux=dx*K; %速度
%     uy=dy*K;
%      ux(1)=a*t;
     ux(1)=10;
    for i=1:N
        x(i)=x(i)+(ux(i)*cos(theta(i))-uy(i)*sin(theta(i)))*tt;
        y(i)=y(i)+(ux(i)*sin(theta(i))+uy(i)*cos(theta(i)))*tt;
    end
    
    d21=sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
    d31=sqrt((x(3)-x(1))^2+(y(3)-y(1))^2);
    d41=sqrt((x(4)-x(1))^2+(y(4)-y(1))^2);
    d51=sqrt((x(5)-x(1))^2+(y(5)-y(1))^2);
    d61=sqrt((x(6)-x(1))^2+(y(6)-y(1))^2);
    d71=sqrt((x(7)-x(1))^2+(y(7)-y(1))^2);
    D21=vertcat(D21,d21);
    D31=vertcat(D31,d31);
    D41=vertcat(D41,d41);
    D51=vertcat(D51,d51);
    D61=vertcat(D61,d61);
    D71=vertcat(D71,d71);
    
    X=vertcat(X,x);
    Y=vertcat(Y,y);
    Theta=vertcat(Theta,theta);   
   end

%% 本当に目標の相対距離に収束しているか確認するとき用
%  [dx,dy]=dxdy3(dx,dy,rx,ry,dd,sgn,N); %ローカル座標の目標位置を求める関数
 e=zeros(1,N);
 for i=1:N
 e(i)=sqrt((dx(i))^2+(dy(i))^2);
 end
 
 
%% アニメーションとグラフ
%アニメーション
figure(1)
X(end,:)=[];
Y(end,:)=[];
Theta(end,:)=[];
ii = 1; jj = 1; k=0;
while ii <length(X(:,1))
 plot(X(ii,:),Y(ii,:),'ko','MarkerFaceColor','g','MarkerSize',8,'LineWidth',2)
 hold on
 for k=1:N
 p1x(k)=X(ii,k)+cos(Theta(ii,k))*10;
 p1y(k)=Y(ii,k)+sin(Theta(ii,k))*10;
 p2x(k)=X(ii,k)+cos(Theta(ii,k))*(-5)-sin(Theta(ii,k))*5;
 p2y(k)=Y(ii,k)+sin(Theta(ii,k))*(-5)+cos(Theta(ii,k))*5;
 p3x(k)=X(ii,k)+cos(Theta(ii,k))*(-5)-sin(Theta(ii,k))*(-5);
 p3y(k)=Y(ii,k)+sin(Theta(ii,k))*(-5)+cos(Theta(ii,k))*(-5);
 end
 for k=1:N
 plot([p1x(k),p2x(k)],[p1y(k),p2y(k)],'b')
 plot([p1x(k),p3x(k)],[p1y(k),p3y(k)],'b')
 plot([p2x(k),p3x(k)],[p2y(k),p3y(k)],'b')
 end
 hold off
 
 grid on
 xlabel('x [cm]','Fontsize',20);
 ylabel('y [cm]','Fontsize',20);
  axis([-100 800 -100 300])
set(gcf, 'Position', [300 30 850 400]);
 if ii == 1
  while jj < 30
   drawnow;
   plot_movie(jj) = getframe(gcf);
   jj = jj+1;
  end
  pause(2);
  ii = ii+1;
 else
  ii=ii+1;
 end
 drawnow;
 plot_movie(jj) = getframe(gcf);
 jj = jj+1;
 end

%グラフ
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
 p1x(k)=X(end,k)+cos(Theta(ii,k))*10;
 p1y(k)=Y(end,k)+sin(Theta(ii,k))*10;
 p2x(k)=X(end,k)+cos(Theta(ii,k))*(-5)-sin(Theta(ii,k))*5;
 p2y(k)=Y(end,k)+sin(Theta(ii,k))*(-5)+cos(Theta(ii,k))*5;
 p3x(k)=X(end,k)+cos(Theta(ii,k))*(-5)-sin(Theta(ii,k))*(-5);
 p3y(k)=Y(end,k)+sin(Theta(ii,k))*(-5)+cos(Theta(ii,k))*(-5);
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
axis([-100 1000 -100 300])
set(gcf, 'Position', [300 30 1150 400]);
 grid 
 legend('1','2','3','4','5','6','7');
 set(gca,'FontSize',20);
xlabel('x [cm]','Fontsize',20)
ylabel('y [cm]','Fontsize',20)

figure(3)
    T=0:tt:t;
     DD=60*ones(length(T),1);
    plot(T,DD,'LineWidth',1.5)
    hold on
    D21(1,:)=[];
    D31(1,:)=[];
    D41(1,:)=[];
    D51(1,:)=[];
    D61(1,:)=[];
    D71(1,:)=[];
    plot(T,D21,'LineWidth',1.5)
    plot(T,D31,'LineWidth',1.5)
    plot(T,D41,'LineWidth',1.5)
    plot(T,D51,'LineWidth',1.5)
    plot(T,D61,'LineWidth',1.5)
    plot(T,D71,'LineWidth',1.5)
    hold off
    grid on
     legend('目標の距離 d_{12}','1,2間の距離 ||^{2}p_{1}||','1,3間の距離 ||^{3}p_{1}||','1,4間の距離 ||^{4}p_{1}||','1,5間の距離 ||^{5}p_{1}||','1,6間の距離 ||^{6}p_{1}||','1,7間の距離 ||^{7}p_{1}||');
    set(gca,'FontSize',13);
    xlabel('時間 t [s]','Fontsize',20);
    ylabel('相対距離 [cm]','Fontsize',20);
    
  e(1,2)=DD(end)-D21(end,1);
  e(1,3)=DD(end)-D31(end,1);
  e(1,4)=DD(end)-D41(end,1);
  e(1,5)=DD(end)-D51(end,1);
   
    