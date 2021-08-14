% relative_distance_201906.m
% 20190604
% ���_�n

clear;
close all;

% number of vehicles
N = 5;

% initial state
 rng(6); x = 25*rand(1,N);
 rng(7); y = -20*rand(1,N);
 theta=[pi/10 pi/5 -pi/4 0 pi];

% desired state
dd=zeros(N,N); %�ڕW�̑��΋���
for i=2:N
   dd(i,1)=30;
   dd(i+1,i)=30;
end


sgn=1; %�ڕW�̗אڏ���

%% 
rx=zeros(N,N);   %���Έʒu�@�s���  
ry=zeros(N,N);
dx=zeros(1,N);    %�ڕW�ʒu
dy=zeros(1,N);
K=0.5; % �Q�C��
tt=0.1;
X=x;
Y=y;
   for t=0:tt:25
    [rx,ry]=relative_position(x,y,theta,N,rx,ry); %���Έʒu�����߂�֐�
    [dx,dy]=dxdy3(dx,dy,rx,ry,dd,sgn,N); %�ڕW�ʒu�����߂�֐�
    ux=dx*K; %���x
    uy=dy*K;
    for i=1:N
     x(i)=x(i)+(ux(i)*cos(theta(i))-uy(i)*sin(theta(i)))*tt;
     y(i)=y(i)+(ux(i)*sin(theta(i))+uy(i)*cos(theta(i)))*tt;
    end
    X=vertcat(X,x);
    Y=vertcat(Y,y);
   
   end

%% �O���t
figure(1)
ii = 1; jj = 1;
while ii <length(X(:,1))
 plot(X(ii,:),Y(ii,:),'ko','MarkerFaceColor','g','MarkerSize',20,'LineWidth',2)
 grid on
 xlabel('x [cm]','Fontsize',20);
 ylabel('y [cm]','Fontsize',20);
 axis([-20 60 -30 40])
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
figure(2)
plot(X(:,1:N),Y(:,1:N),X(end,:),Y(end,:),'ko','MarkerFaceColor','g','MarkerSize',10,'LineWidth',2)
axis([-20 60 -30 40])
grid
set(gcf, 'Position', [200 100 650 600]);
xlabel('x [cm]','Fontsize',10)
ylabel('y [cm]','Fontsize',10)
