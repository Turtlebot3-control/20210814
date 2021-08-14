%collision_avoidance.m
function omega3=collision_avoidance2(omega,rx,ry,N)
%dp=0.2;
dp=20;
k=10;
%k=0.1;
K=zeros(1,N);
U=zeros(2,N);
omega3=zeros(1,N);
%potential
for i=1:N
    for j=1:N
       if (i==2 && j==1) || (i==3 && j==1) || (i==3 && j==2) || (i==4 && j==1) || (i==4 && j==3) || (i==5 && j==1) || (i==5 && j==4)
           U(1,i)=U(1,i)-2*dp*(rx(i,j))/((rx(i,j)^2+ry(i,j)^2)^2)+rx(i,j)/((rx(i,j))^2+(ry(i,j))^2);
           U(2,i)=U(2,i)-2*dp*(ry(i,j))/((rx(i,j)^2+ry(i,j)^2)^2)+ry(i,j)/((rx(i,j))^2+(ry(i,j))^2);
           K(1,i)=K(1,i)+k*(omega(i)-omega(j));
      end
    end
    omega3(1,i)=omega+(-sinU(1,i))*K(1,i);
end
end