function [T,X,Y,Theta] = vehicle_r(ts,N,x,y,theta,rx,ry,dx,dy,dd,sgn,K,ver,phi)
    XX0 = [x;y;theta;phi];
    initial = XX0(:);
    [T,XX] = ode45(@twdd_vehicle_r,ts,initial,[],N,rx,ry,dx,dy,dd,sgn,K,ver);
    X=XX(:,1:4:end);
    Y=XX(:,2:4:end);
    Theta=XX(:,3:4:end);
end