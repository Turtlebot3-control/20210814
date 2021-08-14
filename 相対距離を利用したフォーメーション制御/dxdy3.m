%% �ڕW�ʒu�����߂�֐�
%rx(i,j)��i���猩��j�̑��Έʒu�@dd���ڕW�̑��΋����@dx(i),dy(i)���G�[�W�F���gi�̃��[�J�����W�n�ɂ�����ڕW�ʒu
%��������������

function [dx,dy]=dxdy3(dx,dy,rx,ry,dd,sgn,N)

 dx(2)=rx(2,1)+dd(2,1);
 dy(2)=ry(2,1);

 for i=2:N-1 %i+1���q�D���̗אڂ���_�͂P��i
        % �אړ_�������ʒu
        if sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)==0    
            dx(i+1)=rx(i+1,i)+dd(i+1,i);
            dy(i+1);
        end
        
        % �אړ_���߂�����
        if 0<sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)&&sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)<abs(dd(i+1,1)-dd(i+1,i))
            dx(i+1)=rx(i+1,i)-dd(i+1,i)*(rx(i+1,1)-rx(i+1,i))/sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2);
            dy(i+1)=ry(i+1,i)-dd(i+1,i)*(ry(i+1,1)-ry(i+1,i))/sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2);
        end
        
        % �D�ʒu
         if abs(dd(i+1,1)-dd(i+1,i))<=sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)&&sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)<=abs(dd(i+1,1)+dd(i+1,i))
             costheta=(dd(i+1,i)^2+((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)-dd(i+1,1)^2)/(2*dd(i+1,i)*sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2));
             sintheta=sqrt(1-costheta^2);
             R=[costheta sgn*sintheta; -sgn*sintheta costheta]; %��]�s��
             dx(i+1)=rx(i+1,i)+(R(1,1)*(rx(i+1,1)-rx(i+1,i))+R(1,2)*(ry(i+1,1)-ry(i+1,i)))*dd(i+1,i)/(sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2));
             dy(i+1)=ry(i+1,i)+(R(2,1)*(rx(i+1,1)-rx(i+1,i))+R(2,2)*(ry(i+1,1)-ry(i+1,i)))*dd(i+1,i)/(sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2));
         end
         
        % �אړ_����������
         if abs(dd(i+1,1)+dd(i+1,i))<sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2)
             dx(i+1)=rx(i+1,i)+dd(i+1,i)*(rx(i+1,1)-rx(i+1,i))/(sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2));
             dy(i+1)=ry(i+1,i)+dd(i+1,i)*(ry(i+1,1)-ry(i+1,i))/(sqrt((rx(i+1,i)-rx(i+1,1))^2+(ry(i+1,i)-ry(i+1,1))^2));
         end
 end
end