%% ���[���h���W�̏����g��Ȃ��ő��Έʒu�����߂�֐��@
% �������������Ă鎞�ɂ����g���Ȃ������

function [rx,ry]=relative_position2(rx,ry,ux,uy,tt,N,phi)

 for i=1:N
     for j=1:N
       rx(i,j)=rx(i,j)-ux(i)*tt+ux(j)*tt; %���i
       ry(i,j)=ry(i,j)-uy(i)*tt+uy(j)*tt; 
     end
 end
 rrx=rx
 rry=ry
 for i=1:N
     for j=1:N
       rx(i,j)=cos(phi(i))*rrx(i,j)+sin(phi(i))*rry(i,j); %��]
       ry(i,j)=-sin(phi(i))*rrx(i,j)+cos(phi(i))*rry(i,j);
     end
 end
rx;
ry;
 end