%final_angular.m
%�ڕW�̑��Ίp�x�Ɏ������邽�߂̊p���x
%�ڕW�̑��΋����Ɏ������Ă��炱�̊֐��������n�߂�
%�������Y��ȏ���������񂾂낤�Ȃ�

function omega3=final_angular(omega3,rx,ry,Kw,dp21,dp31,dp32,dp41,dp43,dp51,dp54,N)
    %theta�͌��݂̑��Ίp�x�ƖڕW�̑��Ίp�x�Ƃ̍�
    %Theta�͂����𑫂������
    theta=zeros(N,N);
    Theta=zeros(1,N);
    
    %�r�[�N��2
    R21 = (1/norm(dp21)*[dp21(1,1) dp21(1,2); -dp21(1,2) dp21(1,1)])*(1/norm([rx(2,1) ry(2,1)])*[rx(2,1) -ry(2,1); ry(2,1) rx(2,1)]); %2�ɑ΂���1��theta�̊p�x�s��
    
    if asin(R21(2,1))>0
        theta(2,1)=acos(R21(1,1));
    else
        theta(2,1)=-acos(R21(1,1));
    end
    Theta(2)=theta(2,1);
    %�r�[�N��3 3��1��2�̑��Ώ����擾�ł���
    R31 = (1/norm(dp31)*[dp31(1,1) dp31(1,2); -dp31(1,2) dp31(1,1)])*(1/norm([rx(3,1) ry(3,1)])*[rx(3,1) -ry(3,1); ry(3,1) rx(3,1)]);
    
    if asin(R31(2,1))>0
        theta(3,1)=acos(R31(1,1));
    else
        theta(3,1)=-acos(R31(1,1));
    end
    
    R32 = (1/norm(dp32)*[dp32(1,1) dp32(1,2); -dp32(1,2) dp32(1,1)])*(1/norm([rx(3,2) ry(3,2)])*[rx(3,2) -ry(3,2); ry(3,2) rx(3,2)]);
    
    if asin(R32(2,1))>0
        theta(3,2)=acos(R32(1,1));
    else
        theta(3,2)=-acos(R32(1,1));
    end
    Theta(3)=theta(3,1)+theta(3,2);
    
   %�r�[�N��4 4��1��3�̑��Ώ����擾�ł���
    R41 = (1/norm(dp41)*[dp41(1,1) dp41(1,2); -dp41(1,2) dp41(1,1)])*(1/norm([rx(4,1) ry(4,1)])*[rx(4,1) -ry(4,1); ry(4,1) rx(4,1)]);
    
    if asin(R41(2,1))>0
        theta(4,1)=acos(R41(1,1));
    else
        theta(4,1)=-acos(R41(1,1));
    end
    
    R43 = (1/norm(dp43)*[dp43(1,1) dp43(1,2); -dp43(1,2) dp43(1,1)])*(1/norm([rx(4,3) ry(4,3)])*[rx(4,3) -ry(4,3); ry(4,3) rx(4,3)]);
    
    if asin(R43(2,1))>0
        theta(4,3)=acos(R43(1,1));
    else
        theta(4,3)=-acos(R43(1,1));
    end
    Theta(4)=theta(4,1)+theta(4,3);
    
    %�r�[�N��5 5��1��4�̑��Ώ����擾�ł���
    R51 = (1/norm(dp51)*[dp51(1,1) dp51(1,2); -dp51(1,2) dp51(1,1)])*(1/norm([rx(5,1) ry(5,1)])*[rx(5,1) -ry(5,1); ry(5,1) rx(5,1)]);
    
    if asin(R51(2,1))>0
        theta(5,1)=acos(R51(1,1));
    else
        theta(5,1)=-acos(R51(1,1));
    end
    
    R54 = (1/norm(dp54)*[dp54(1,1) dp54(1,2); -dp54(1,2) dp54(1,1)])*(1/norm([rx(5,4) ry(5,4)])*[rx(5,4) -ry(5,4); ry(5,4) rx(5,4)]);
    
    if asin(R54(2,1))>0
        theta(5,4)=acos(R54(1,1));
    else
        theta(5,4)=-acos(R54(1,1));
    end
    Theta(5)=theta(5,1)+theta(5,4);
    
    %�p���x
    omega3=Kw*Theta;
end