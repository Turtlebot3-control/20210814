function dX = twdd_vehicle_w(ts,X,N,rx,ry,dx,dy,dd,sgn,K,ver,aex,aey,gamma)
    x = zeros(1,N);
    y = zeros(1,N);
    theta = zeros(N,1);
    omega2 = zeros(1,N); %非�?�ロノミ�?クロボット�?�制�?を�??えた角�?�度
    v2 = zeros(1,N);
    phi=zeros(1,N); %変角�?�積�?
    vex=zeros(1,N);
    vey=zeros(1,N);
    ux=zeros(1,N);
    uy=zeros(1,N);
    if ver == 1
        for ii = 1:N
            x(ii) = X((ii-1)*6+1);
            y(ii) = X((ii-1)*6+2);
            theta(ii) = X((ii-1)*6+3);
            phi(ii) = X((ii-1)*6+4);
            vex(ii) = X((ii-1)*6+5);
            vey(ii) = X((ii-1)*6+6);
        end
    elseif ver == 0
        for ii = 1:N
            x(ii) = X((ii-1)*5+1);
            y(ii) = X((ii-1)*5+2);
            theta(ii) = X((ii-1)*5+3);
            vex(ii) = X((ii-1)*5+4);
            vey(ii) = X((ii-1)*5+5);
        end
    end
    [rx,ry]=relative_position(x,y,theta,N,rx,ry); %相対位置を求める関数
%     wrx=rx+randn(N)*5;
%     wry=ry+randn(N)*5;
      wrx=rx;
      wry=ry;

    if ver == 0
        [dx,dy]=dxdy3(dx,dy,wrx,wry,dd,sgn,N); %ローカル座標�?�目標位置を求める関数
        ux(1) = vex(1);
        uy(1) = vey(1);
        vex(1);
        for ii =2:N
%             ux(ii)=(1/(ts+1))*vex(ii)+K*(1/(ts+1))*dx(ii);
%             uy(ii)=(1/(ts+1))*vey(ii)+K*(1/(ts+1))*dy(ii);
            ux(ii)=vex(ii)+K*dx(ii)+randn(1)*8;
            uy(ii)=vey(ii)+K*dy(ii)+randn(1)*8;
        end
    elseif ver == 1
        [dx,dy]=dxdy2(dx,dy,rx,ry,dd,sgn,N,phi); 
        ux=dx*K; %速度入�?
        uy=dy*K;
        [omega,v] = cart2pol(ux,uy); %速度を極座標変換して線�?�度と角�?�度を求め�?
        [omega2,v2] = nonholonomic(omega,v,N,omega2,v2); %非�?�ロノミ�?クロボット�?�制�?を回避するために
    end
    
    if ver == 0
        dX = zeros(5*N,1);
        dX(1) = ux(1);
        dX(2) = uy(1);
        dX(3) = 0;
        dX(4) = aex;
        dX(5) = aey;
        for ii = 2:N
            dX((ii-1)*5+1) = ux(ii);
            dX((ii-1)*5+2) = uy(ii);
            dX((ii-1)*5+3) = theta(ii)*0;
%             dX((ii-1)*5+4) = aex+gamma*K*(1/(ts+1))*dx(ii);
%             dX((ii-1)*5+5) = aey+gamma*K*(1/(ts+1))*dy(ii);
            dX((ii-1)*5+4) = gamma*K*dx(ii);
            dX((ii-1)*5+5) = gamma*K*dy(ii);
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