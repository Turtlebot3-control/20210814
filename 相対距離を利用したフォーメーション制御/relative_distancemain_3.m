% relative_distance_201906.m
% 20190606
% �������������ǔ�z���m�~�b�N�̓����͍l���Ȃ��D
% 2���ڂɑ��Έʒu�����߂�Ƃ��̓O���[�o�����W���g�������Ȃ��p

clear;
close all;

% number of vehicles
N = 5;

% initial state

% P=[0 10 20 30 40;
%    0 10 20 30 40];
% x=P(1,:);
% y=P(2,:);
 rng(6); x = 25*rand(1,N);
 rng(7); y = 25*rand(1,N);
%  theta=zeros(1,N);
theta=[pi/10 pi/5 -pi/4 -pi/3 pi];


% desired state
dd=zeros(N,N); %�ڕW�̑��΋���
for i=2:N
   dd(i,1)=25;
   dd(i+1,i)=25;
end


sgn=1; %�ڕW�̗אڏ��� ������ς���Δ��]����

%% 
rx=zeros(N,N);   %���Έʒu�@
ry=zeros(N,N);
dx=zeros(1,N);    %�ڕW�ʒu(���[�J�����)
dy=zeros(1,N);
K=0.5; % �Q�C��
tt=0.1;
X=x;
Y=y;
Theta=theta;
phi=[0 0 0 0 0]; %���݂̕����ƖڕW�̕����Ƃ̊p�x���@��ɕ��i�^���Ȃ̂ō���̓[���@
   for t=0:tt:20
    [rx,ry]=relative_position(x,y,theta,N,rx,ry); %���Έʒu�����߂�֐� �ŏ��������[���h���W�̏��(�����ʒu)�𗘗p����D
    [dx,dy]=dxdy3(dx,dy,rx,ry,dd,sgn,N); %���[�J�����W�̖ڕW�ʒu�����߂�֐�
    ux=dx*K; %���x
    uy=dy*K;
    for i=1:5
    x(i)=x(i)+(ux(i)*cos(theta(i))-uy(i)*sin(theta(i)))*tt;
    y(i)=y(i)+(ux(i)*sin(theta(i))+uy(i)*cos(theta(i)))*tt;
    end
    X=vertcat(X,x);
    Y=vertcat(Y,y);
    Theta=vertcat(Theta,theta);
   end

%% �O���t
ii = 1; jj = 1;
while ii <length(X(:,1))
 plot(X(ii,:),Y(ii,:),'ko','MarkerFaceColor','g','MarkerSize',20,'LineWidth',2)
 grid on
 xlabel('x [cm]','Fontsize',20);
 ylabel('y [cm]','Fontsize',20);
 axis([-10 60 -30 30])
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
%�O���t2
ii=length(X(:,1));
for k=1:N
 p1x(k)=X(ii,k)+cos(Theta(ii,k))*3;
 p1y(k)=Y(ii,k)+sin(Theta(ii,k))*3;
 p2x(k)=X(ii,k)+cos(Theta(ii,k))*(-1.5)-sin(Theta(ii,k))*1.5;
 p2y(k)=Y(ii,k)+sin(Theta(ii,k))*(-1.5)+cos(Theta(ii,k))*1.5;
 p3x(k)=X(ii,k)+cos(Theta(ii,k))*(-1.5)-sin(Theta(ii,k))*(-1.5);
 p3y(k)=Y(ii,k)+sin(Theta(ii,k))*(-1.5)+cos(Theta(ii,k))*(-1.5);
 f1x(k)=X(1,k)+cos(Theta(1,k))*3;
 f1y(k)=Y(1,k)+sin(Theta(1,k))*3;
 f2x(k)=X(1,k)+cos(Theta(1,k))*(-1.5)-sin(Theta(1,k))*1.5;
 f2y(k)=Y(1,k)+sin(Theta(1,k))*(-1.5)+cos(Theta(1,k))*1.5;
 f3x(k)=X(1,k)+cos(Theta(1,k))*(-1.5)-sin(Theta(1,k))*(-1.5);
 f3y(k)=Y(1,k)+sin(Theta(1,k))*(-1.5)+cos(Theta(1,k))*(-1.5);
 end
%plot(X(:,1:N),Y(:,1:N),X(end,:),Y(end,:),'ko','MarkerFaceColor','g','MarkerSize',10,'LineWidth',2)
plot(X(:,1:N),Y(:,1:N),'LineWidth',1.5)
hold on;
for k=1:N
 plot([f1x(k),f2x(k)],[f1y(k),f2y(k)],'k','LineWidth',1)
 plot([f1x(k),f3x(k)],[f1y(k),f3y(k)],'k','LineWidth',1)
 plot([f2x(k),f3x(k)],[f2y(k),f3y(k)],'k','LineWidth',1)
 plot([p1x(k),p2x(k)],[p1y(k),p2y(k)],'b','LineWidth',1.5)
 plot([p1x(k),p3x(k)],[p1y(k),p3y(k)],'b','LineWidth',1.5)
 plot([p2x(k),p3x(k)],[p2y(k),p3y(k)],'b','LineWidth',1.5)
end
 hold off;
 legend('1','2','3','4','5');
axis([-10 60 -30 40])
xlabel('x [cm]','Fontsize',10)
ylabel('y [cm]','Fontsize',10)
