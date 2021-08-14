% relative_distance_201906_2.m
%20190621
% relative_distance_201906.m���z���m�~�b�N���{�b�g�̓����ɂ��Ă݂�D
% ���Έʒu�����߂�֐������[���h���W���g���Ă邯��
% �A�j���[�V�����H�v����

clear;
close all;

% number of vehicles
N = 5;

% initial state ���[���h���W
P=[0 10 20 30 40;
   0 10 20 30 40];
x=P(1,:); 
y=P(2,:);
 rng(6); x = 25*rand(1,N);
 rng(7); y = -20*rand(1,N);
theta=zeros(1,N);
theta=[pi/10 pi/5 -pi/4 -pi/3 pi];
%rng(4); theta = rand(1,N);
% desired state
dd=zeros(N,N); %�ڕW�̑��΋���
for i=2:N
   dd(i,1)=30;
   dd(i,i-1)=30;
end

sgn=1; %�ڕW�̗אڏ���

%% 
rx=zeros(N,N);   %���Έʒu�@�s���  
ry=zeros(N,N);
dx=zeros(1,N);    %�ڕW�ʒu
dy=zeros(1,N);
K=0.5; % �Q�C��
tt=0.1; % �X�e�b�v����
omega2=zeros(1,N);
v2=zeros(1,N);
X=x;
Y=y;
Theta=theta;
phi=zeros(1,N); %�ϊp�̐ϕ�
for t=0:tt:20
    [rx,ry]=relative_position(x,y,theta,N,rx,ry); %���Έʒu�����߂�֐�
    [dx,dy]=dxdy2(dx,dy,rx,ry,dd,sgn,N,phi); %�ڕW�ʒu�����߂�֐�
    ux=dx*K; %���x
    uy=dy*K;
    [omega,v] = cart2pol(ux,uy); %���x���ɍ��W�ϊ����Đ����x�Ɗp���x�����߂�
    [omega2,v2] = nonholonomic(omega,v,N,omega2,v2); %��z���m�~�b�N���{�b�g�̐����������邽�߂�
    theta=theta+omega2*tt;
    phi=phi+omega2*tt;
   for i=1:N
     x(i)=x(i)+v2(i)*cos(theta(i))*tt;
     y(i)=y(i)+v2(i)*sin(theta(i))*tt;
   end
    X=vertcat(X,x);
    Y=vertcat(Y,y);
    Theta=vertcat(Theta,theta);
    
    if v2(1)<10^-2&&v2(2)<10^-2&&v2(3)<10^-2&&v2(4)<10^-2&&v2(5)<10^-2 %�S�Ă̑��x���������Ȃ�����
      break;
    end
end

%% �{���ɖڕW�̑��΋����Ɏ������Ă��邩�m�F����Ƃ��p
d=zeros(N,N);
for i=1:N
    for j=1:N
d(i,j)=sqrt((x(i)-x(j))^2+(y(i)-y(j))^2);
    end
end

%% �O���t
ii = 1; jj = 1; k=0;
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
 set(gca,'FontSize',20);
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
%�O���t2
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
axis([-20 60 -30 40])
set(gcf, 'Position', [750 750 750 650]);
 grid 
 legend('1','2','3','4','5');
set(gca,'FontSize',20);
xlabel('x [cm]','Fontsize',20)
ylabel('y [cm]','Fontsize',20)
