%% ���Έʒu�����߂�֐�

function [rx,ry]=relative_position(x,y,theta,N,rx,ry)

 for i=1:N
     for j=1:N
       rx(i,j)=cos(theta(i))*(x(j)-x(i))+sin(theta(i))*(y(j)-y(i));  %i�̃��[�J�����W�n���j�̈ʒu
       ry(i,j)=-sin(theta(i))*(x(j)-x(i))+cos(theta(i))*(y(j)-y(i));
     end
 end
 end