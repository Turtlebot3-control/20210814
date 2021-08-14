%final_angular.m
%目標の相対角度に収束するための角速度
%目標の相対距離に収束してからこの関数が動き始める
%もっと綺麗な書き方あるんだろうなぁ

function omega3=final_angular(omega3,rx,ry,Kw,dp21,dp31,dp32,dp41,dp43,dp51,dp54,N)
    %thetaは現在の相対角度と目標の相対角度との差
    %Thetaはそれらを足したやつ
    theta=zeros(N,N);
    Theta=zeros(1,N);
    
    %ビークル2
    R21 = (1/norm(dp21)*[dp21(1,1) dp21(1,2); -dp21(1,2) dp21(1,1)])*(1/norm([rx(2,1) ry(2,1)])*[rx(2,1) -ry(2,1); ry(2,1) rx(2,1)]); %2に対する1のthetaの角度行列
    
    if asin(R21(2,1))>0
        theta(2,1)=acos(R21(1,1));
    else
        theta(2,1)=-acos(R21(1,1));
    end
    Theta(2)=theta(2,1);
    %ビークル3 3は1と2の相対情報を取得できる
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
    
   %ビークル4 4は1と3の相対情報を取得できる
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
    
    %ビークル5 5は1と4の相対情報を取得できる
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
    
    %角速度
    omega3=Kw*Theta;
end