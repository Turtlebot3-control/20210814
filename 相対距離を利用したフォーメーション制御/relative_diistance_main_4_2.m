% relative_distance_201907_4.m
% 20190724
% relative_distance_201906.mを非ホロノミックロボットの動きにしてみる．
% 相対位置を求める関数がワールド座標を使ってるけど
% アニメーション工夫した
% 最終的な方位も考えよう

clear;
close all;

% number of vehicles
N = 5;

%　質点なら0 移動ロボットなら1
ver = 1; 

% initial state ワールド座標
% P=[0 30 15 -15 -30;
%    0 0 15*sqrt(3) 15*sqrt(3) 0];
% x=P(1,:); 
% y=P(2,:);
 rng(6); x = 25*rand(1,N);
 rng(7); y = -20*rand(1,N);
%theta=zeros(1,N);
theta=[pi/10 pi/5 -pi/4 -pi/3 pi];
%rng(4); theta = rand(1,N);

% desired state
dp21=[-30 0];
dp31=[-15*sqrt(3) 15];
dp32=[-15*sqrt(3) -15];
dp41=[-15*sqrt(3) -15];
dp43=[0 -30];
dp51=[-30 0];
dp54=[-15 -15*sqrt(3)];

dd=zeros(N,N); %目標の相対距離
for i=2:N
    
  dd=[0 0 0 0 0;
      norm(dp21) 0 0 0 0;
      norm(dp31) norm(dp32) 0 0 0;
      norm(dp41) 0 norm(dp43) 0 0;
      norm(dp51) 0 0 norm(dp54) 0];
end


sgn=1; %目標の隣接順序 符号を変えれば反転する

%% 
rx=zeros(N,N);   %相対位置　行が基準  
ry=zeros(N,N);
dx=zeros(1,N);    %目標位置
dy=zeros(1,N);
K=0.5; % ゲイン
phi=zeros(1,N); %変角の積分

%% 目標の相対距離に収束するために
    ts = [0 15];
    [T,X,Y,Theta] = vehicle_r(ts,N,x,y,theta,rx,ry,dx,dy,dd,sgn,K,ver,phi);
    
%% 本当に目標の相対距離に収束しているか確認するとき用
d=zeros(N,N);
for i=1:N
    for j=1:N
d(i,j)=sqrt((x(i)-x(j))^2+(y(i)-y(j))^2);
    end
end

%% グラフ
ii = 1; jj = 1; k=0;
count = 1;
while ii <length(X(:,1))
 plot(X(ii,:),Y(ii,:),'ko','MarkerFaceColor','g','MarkerSize',8,'LineWidth',2)
 hold on
 for k=1:N
 p1x(k)=X(ii,k)+cos(Theta(ii,k))*3;
 p1y(k)=Y(ii,k)+sin(Theta(ii,k))*3;
 p2x(k)=X(ii,k)+cos(Theta(ii,k))*(-1.5)-sin(Theta(ii,k))*1.5;
 p2y(k)=Y(ii,k)+sin(Theta(ii,k))*(-1.5)+cos(Theta(ii,k))*1.5;
 p3x(k)=X(ii,k)+cos(Theta(ii,k))*(-1.5)-sin(Theta(ii,k))*(-1.5);
 p3y(k)=Y(ii,k)+sin(Theta(ii,k))*(-1.5)+cos(Theta(ii,k))*(-1.5);
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
 axis([-20 60 -40 40])
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
 set(gcf, 'Position',[200 100 650 600]);
 jj = jj+1;
mov(count) = getframe(gcf);
count = count+1;
end
v = VideoWriter('relative_distance.avi');
 v.FrameRate = 10;
%  v.FrameRate = 40;
 open(v)
 %for count=2:length(t)
  writeVideo(v,mov)
 %end
 close(v)
%グラフ2
 for k=1:N
 f1x(k)=X(1,k)+cos(Theta(1,k))*3;
 f1y(k)=Y(1,k)+sin(Theta(1,k))*3;
 f2x(k)=X(1,k)+cos(Theta(1,k))*(-1.5)-sin(Theta(1,k))*1.5;
 f2y(k)=Y(1,k)+sin(Theta(1,k))*(-1.5)+cos(Theta(1,k))*1.5;
 f3x(k)=X(1,k)+cos(Theta(1,k))*(-1.5)-sin(Theta(1,k))*(-1.5);
 f3y(k)=Y(1,k)+sin(Theta(1,k))*(-1.5)+cos(Theta(1,k))*(-1.5);
 end
%plot(X(:,1:N),Y(:,1:N),X(end,:),Y(end,:),'ko','MarkerFaceColor','g','MarkerSize',10,'LineWidth',2)
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
axis([-20 60 -40 40])
set(gcf, 'Position', [200 100 650 600]);
 grid 
 legend('1','2','3','4','5');
 set(gca,'FontSize',20);
xlabel('x [cm]','Fontsize',20)
ylabel('y [cm]','Fontsize',20)
