% relative_distance_main_8.m
% 202005?
% ���[�_�𓮂���
% ��z���m�~�b�N�̓����̂Ƃ��͊�]�̒��΍��ɂȂ�Ȃ��D���_�n�̈ړ��̂Ƃ��͂��܂��������D
% ���䑥��ς����i�A�����_�n�̂Ƃ��j

clear;
close all;

% number of vehicles
N = 5;

% initial state ���[���h���W
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
% theta=zeros(1,N);
% theta(1)=pi/2;
theta=[0 pi/3 -pi/4 2*pi/3 pi];
%theta=[pi/10 pi/5 -pi/4 -pi/3 pi];
%rng(4); theta = rand(1,N);

% desired state
dp21=[-60 0];
dp31=[-30*sqrt(3) 30];
dp32=[-30*sqrt(3) -30];
dp41=[-30*sqrt(3) -30];
dp43=[0 -60];
dp51=[-60 0];
dp54=[-30 -30*sqrt(3)];

%�אڍs��
A=[0 0 0 0 0;
   1 0 0 0 0;
   1 1 0 0 0;
   1 0 1 0 0;
   1 0 0 1 0];

dd=zeros(N,N); %�ڕW�̑��΋���
for i=2:N
    
  dd=[0 0 0 0 0;
      norm(dp21) 0 0 0 0;
      norm(dp31) norm(dp32) 0 0 0;
      norm(dp41) 0 norm(dp43) 0 0;
      norm(dp51) 0 0 norm(dp54) 0];
end

sgn=1; %�ڕW�̗אڏ��� ������ς���Δ��]����

%% 
rx=zeros(N,N);   %���Έʒu�@�s���  
ry=zeros(N,N);
dx=zeros(1,N);    %�ڕW�ʒu
dy=zeros(1,N);
%K=0.5; % �Q�C��
K=1;
tt=0.1; % �X�e�b�v����
omega2=zeros(1,N); %��z���m�~�b�N���{�b�g�̐�����l�����p���x
%omega3=zeros(1,N); %�ڕW�̑��Ίp�x�Ɏ������邽�߂̊p���x
v2=zeros(1,N);
X=x;
Y=y;
Theta=theta;
phi=zeros(1,N); %�ϊp�̐ϕ�

%% �ڕW�̑��΋����Ɏ������邽�߂�  
    for t=0:tt:20
    [rx,ry]=relative_position(x,y,theta,N,rx,ry); %���Έʒu�����߂�֐� �ŏ��������[���h���W�̏��(�����ʒu)�𗘗p����D
    ux(1)=20;
    %uy(1)=20;
    [drx,dry]=relative_velocity(A,ux,uy,theta,N);
    [dx,dy]=dxdy2(dx,dy,rx,ry,dd,sgn,N,phi); %���[�J�����W�̖ڕW�ʒu�����߂�֐�
    ux=dx*K+drx; %���x
    uy=dy*K+dry; 
    [omega,v] = cart2pol(ux,uy); %���x���ɍ��W�ϊ����Đ����x�Ɗp���x�����߂�
    [omega2,v2] = nonholonomic(omega,v,N,omega2,v2); %��z���m�~�b�N���{�b�g�̐����������邽�߂�
     theta=theta+(omega2*tt);
     phi=phi+(omega2*tt);
    for i=1:N
     x(i)=x(i)+v2(i)*cos(theta(i))*tt;
     y(i)=y(i)+v2(i)*sin(theta(i))*tt;
   end
    X=vertcat(X,x);
    Y=vertcat(Y,y);
    Theta=vertcat(Theta,theta);   
   end

%% �{���ɖڕW�̑��΋����Ɏ������Ă��邩�m�F����Ƃ��p
 [dx,dy]=dxdy2(dx,dy,rx,ry,dd,sgn,N,phi); %���[�J�����W�̖ڕW�ʒu�����߂�֐�
 e=zeros(1,N);
 for i=1:N
 e(i)=sqrt((dx(i))^2+(dy(i))^2);
 end
 
%% �A�j���[�V�����ƃO���t
%�A�j���[�V����
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
  axis([-100 500 -100 300])
set(gcf, 'Position', [300 30 600 400]);
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

%ii=202; %�A�j���[�V����������Ȃ��Ƃ�
%�O���t
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
axis([-100 500 -100 300])
set(gcf, 'Position', [300 30 600 400]);
 grid 
 legend('1','2','3','4','5');
 set(gca,'FontSize',20);
xlabel('x [cm]','Fontsize',20)
ylabel('y [cm]','Fontsize',20)
