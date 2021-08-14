% relative_distance_main_8.m
% 20200528
% ���[�_�𓮂���
% ���[�_�̑��x�ɂ���ăt�H�[���[�V�����̑傫����ς���
% �G�[�W�F���g�̐��𑝂₵��
% ��肭�Ǐ]���Ȃ����Ƃ���������

clear;
close all;

% number of vehicles
% N = 5;
  N = 7;

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
%theta=[0 pi/3 -pi/4 2*pi/3 pi];
%theta=[pi/10 pi/5 -pi/4 -pi/3 pi];
rng(4); theta = rand(1,N);
theta(1)=0;



%�אڍs��
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

dd=zeros(N,N); %�ڕW�̑��΋���
for i=2:N
   dd(i,1)=60;
   dd(i,i-1)=60;
end

sgn=1; %�ڕW�̗אڏ��� ������ς���Δ��]����

%% 
rx=zeros(N,N);   %���Έʒu�@�s���  
ry=zeros(N,N);
dx=zeros(1,N);    %�ڕW�ʒu
dy=zeros(1,N);
ux=zeros(1,N);
uy=zeros(1,N);
K=0.5; % �Q�C��
%K=1;
tt=0.1; % �X�e�b�v����
X=x;
Y=y;
Theta=theta;
phi=zeros(1,N); %�ϊp�̐ϕ�
D21=sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
D31=sqrt((x(3)-x(1))^2+(y(3)-y(1))^2);
D41=sqrt((x(4)-x(1))^2+(y(4)-y(1))^2);
D51=sqrt((x(5)-x(1))^2+(y(5)-y(1))^2);

%% �ڕW�̑��΋����Ɏ������邽�߂�
    
    for t=0:tt:40
    [rx,ry]=relative_position(x,y,theta,N,rx,ry); %���Έʒu�����߂�֐� �ŏ��������[���h���W�̏��(�����ʒu)�𗘗p����D
     ux(1)=t;
    [drx,dry]=relative_velocity(A,ux,uy,theta,N);
    for i=2:N                                     
        dd(i,1)=40+(ux(1)^2+uy(1)^2)^(1/2);  %�ڕW�̑��΋����𓮂���
        dd(i,i-1)=40+(ux(1)^2+uy(1)^2)^(1/2);
%         dd(i,1)=40;  %�ڕW�̑��΋����𓮂���
%         dd(i,i-1)=40;
    end
    
    [dx,dy]=dxdy3(dx,dy,rx,ry,dd,sgn,N); %���[�J�����W�̖ڕW�ʒu�����߂�֐�
    ux=dx*K+drx; %���x
    uy=dy*K+dry; 
     ux(1)=t;
%     uy(1)=20*sin(((2*pi)/40)*t);
%     if t<20
%        ux(1)=20; 
%     else
%        ux(1)=10;
%     end
%     ux(1)=0;
    for i=1:N
        x(i)=x(i)+(ux(i)*cos(theta(i))-uy(i)*sin(theta(i)))*tt;
        y(i)=y(i)+(ux(i)*sin(theta(i))+uy(i)*cos(theta(i)))*tt;
    end
    
    d21=sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
    d31=sqrt((x(3)-x(1))^2+(y(3)-y(1))^2);
    d41=sqrt((x(4)-x(1))^2+(y(4)-y(1))^2);
    d51=sqrt((x(5)-x(1))^2+(y(5)-y(1))^2);
    D21=vertcat(D21,d21);
    D31=vertcat(D31,d31);
    D41=vertcat(D41,d41);
    D51=vertcat(D51,d51);
    
    X=vertcat(X,x);
    Y=vertcat(Y,y);
    Theta=vertcat(Theta,theta);   
   end

%% �{���ɖڕW�̑��΋����Ɏ������Ă��邩�m�F����Ƃ��p
%  [dx,dy]=dxdy3(dx,dy,rx,ry,dd,sgn,N); %���[�J�����W�̖ڕW�ʒu�����߂�֐�
 e=zeros(1,N);
 for i=1:N
 e(i)=sqrt((dx(i))^2+(dy(i))^2);
 end
 e
 
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
axis([-100 1000 -100 300])
set(gcf, 'Position', [300 30 1150 400]);
 grid 
 legend('1','2','3','4','5','6','7');
 set(gca,'FontSize',20);
xlabel('x [cm]','Fontsize',20)
ylabel('y [cm]','Fontsize',20)

figure(3)
    T=0:tt:t;
    DD=40+T;
    % DD=40;
    plot(T,DD)
    hold on;
    D21(end,:)=[];
    D31(end,:)=[];
    D41(end,:)=[];
    D51(end,:)=[];
    plot(T,D21)
%     plot(T,D31)
%     plot(T,D41)
%     plot(T,D51)
    grid on
%     legend('�ڕW�̋��� d_{12}','1,2�Ԃ̋��� ||^{2}p_{1}||','1,3�Ԃ̋��� ||^{2}p_{1}||','1,4�Ԃ̋��� ||^{2}p_{1}||','1,5�Ԃ̋��� ||^{2}p_{1}||');
    legend('�ڕW�̋��� d_{12}','1,2�Ԃ̋��� ||^{2}p_{1}||');
    set(gca,'FontSize',15);
    xlabel('���� t [s]','Fontsize',20);
    ylabel('���΋��� [cm]','Fontsize',20);
    
    