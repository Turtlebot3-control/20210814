function dX = twdd_vehicle_r(ts,X,N,rx,ry,dx,dy,dd,sgn,K,ver)
    x = zeros(1,N);
    y = zeros(1,N);
    theta = zeros(N,1);
    omega2 = zeros(1,N); %非ホロノミックロボットの制約を考えた角速度
    v2 = zeros(1,N);
    phi=zeros(1,N); %変角の積分
    
    for ii = 1:N
        x(ii) = X((ii-1)*4+1);
        y(ii) = X((ii-1)*4+2);
        theta(ii) = X((ii-1)*4+3);
        phi(ii) = X((ii-1)*4+4);
    end
    [rx,ry]=relative_position(x,y,theta,N,rx,ry); %相対位置を求める関数
    
    if ver == 0
        [dx,dy]=dxdy3(dx,dy,rx,ry,dd,sgn,N); %ローカル座標の目標位置を求める関数
    elseif ver == 1
        [dx,dy]=dxdy2(dx,dy,rx,ry,dd,sgn,N,phi); 
        ux=dx*K; %速度入力
        uy=dy*K;
        [omega,v] = cart2pol(ux,uy); %速度を極座標変換して線速度と角速度を求める
        [omega2,v2] = nonholonomic(omega,v,N,omega2,v2); %非ホロノミックロボットの制約を回避するために
    end
    
    dX = zeros(3*N,1);
    if ver == 0
        for ii = 1:N
            dX((ii-1)*4+1) = dx(ii)*K;
            dX((ii-1)*4+2) = dy(ii)*K;
            dX((ii-1)*4+3) = theta(ii)*0;
            dX((ii-1)*4+4) = phi(ii);
        end
    elseif ver == 1
         for ii = 1:N
            dX((ii-1)*4+1) = cos(theta(ii))*v2(ii);
            dX((ii-1)*4+2) = sin(theta(ii))*v2(ii);
            dX((ii-1)*4+3) = omega2(ii);
            dX((ii-1)*4+4) = omega2(ii);
        end
    end
end