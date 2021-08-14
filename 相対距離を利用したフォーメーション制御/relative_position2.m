%% ワールド座標の情報を使わないで相対位置を求める関数　
% 同じ方向向いてる時にしか使えないじゃん

function [rx,ry]=relative_position2(rx,ry,ux,uy,tt,N,phi)

 for i=1:N
     for j=1:N
       rx(i,j)=rx(i,j)-ux(i)*tt+ux(j)*tt; %並進
       ry(i,j)=ry(i,j)-uy(i)*tt+uy(j)*tt; 
     end
 end
 rrx=rx
 rry=ry
 for i=1:N
     for j=1:N
       rx(i,j)=cos(phi(i))*rrx(i,j)+sin(phi(i))*rry(i,j); %回転
       ry(i,j)=-sin(phi(i))*rrx(i,j)+cos(phi(i))*rry(i,j);
     end
 end
rx;
ry;
 end