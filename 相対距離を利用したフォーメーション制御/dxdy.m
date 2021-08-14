%% �ڕW�ʒu�����߂�֐�
%rx(i,j)��i���猩��j�̑��Έʒu�@dd���ڕW�̑��΋����@dx(i),dy(i)���G�[�W�F���gi�̃��[�J�����W�n�ɂ�����ڕW�ʒu

function [dx,dy]=dxdy(dx,dy,rx,ry,dd,sgn,N,phi)

dx(2)=rx(2,1)+cos(phi(2))*dd(2,1);
dy(2)=ry(2,1)-sin(phi(2))*dd(2,1);

 for i=2:N-1 %i+1���q�D���̗אڂ���_�͂P��i
        % �אړ_�������ʒu
        if sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2)==0    
            dx(i+1)=rx(i+1,1)+dd(i+1,1);
            dy(i+1)=ry(i+1,1);
        end
        
        % �אړ_���߂�����
        if sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2)<abs(dd(i+1,i)-dd(i+1,1))
            dx(i+1)=rx(i+1,1)-dd(i+1,1)*(rx(i+1,i)-rx(i+1,1))/sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2);
            dy(i+1)=ry(i+1,1)-dd(i+1,1)*(ry(i+1,i)-ry(i+1,1))/sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2);
        end
        
        % �D�ʒu
         if abs(dd(i+1,i)-dd(i+1,1))<=sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2)&&sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2)<=abs(dd(i+1,i)+dd(i+1,1))
             ctheta=(dd(i+1,1)^2+((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2)-dd(i+1,i)^2)/(2*dd(i+1,1)*sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2));
             stheta=sqrt(1-ctheta^2);
             R=[ctheta sgn*stheta; -sgn*stheta ctheta];
             dx(i+1)=rx(i+1,1)+(R(1,1)*(rx(i+1,i)-rx(i+1,1))+R(1,2)*(ry(i+1,i)-ry(i+1,1)))*dd(i+1,1)/(sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2));
             dy(i+1)=ry(i+1,1)+(R(2,1)*(rx(i+1,i)-rx(i+1,1))+R(2,2)*(ry(i+1,i)-ry(i+1,1)))*dd(i+1,1)/(sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2));
         end
         
        % �אړ_����������
         if abs(dd(i+1,i)+dd(i+1,1))<sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2)
             dx(i+1)=rx(i+1,1)+dd(i+1,1)*(rx(i+1,i)-rx(i+1,1))/(sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2));
             dy(i+1)=ry(i+1,1)+dd(i+1,1)*(ry(i+1,i)-ry(i+1,1))/(sqrt((rx(i+1,1)-rx(i+1,i))^2+(ry(i+1,1)-ry(i+1,i))^2));
         end
 end
end