function  [dfx,dfy]=tformation(A,a,N,rx,ry)
    mdx=zeros(N,N);
    mdy=zeros(N,N);
    dx=zeros(1,N);
    dy=zeros(1,N);
    dfx=zeros(1,N);
    dfy=zeros(1,N);
    ltheta=zeros(N,N);
%     for i=2:N
%         if (rx(1,i)>0&&ry(1,i)>0) || (rx(1,i)>0&&ry(1,i)<0)
%             ltheta(i)=atan(ry(1,i)/rx(1,i));
%         else
%             ltheta(i)=atan(ry(1,i)/rx(1,i))+pi/2;
%         end
%     end
%    
%     for i=2:N
%         dx(i)=cos(ltheta(i))*a;
%         dy(i)=sin(ltheta(i))*a;
%     end
%     for i=1:N
%         for j=1:N
%             mdx(i,j)=dx(i)-dx(j);
%             mdy(i,j)=dy(i)-dy(j);
%         end
%     end
%     for i=2:N
%         dfx(i)=(mdx(i,:)*(A(i,:))')/sum(A(i,:));
%         dfy(i)=(mdy(i,:)*(A(i,:))')/sum(A(i,:));
%     end
% end
for i=2:N
    for j=1:N
        if  (rx(i,j)>=0 && i~=j)
             ltheta(i,j)=atan(ry(i,j)/rx(i,j));
        elseif (rx(i,j)<0 && i~=j)
             ltheta(i,j)=atan(ry(i,j)/rx(i,j))+pi;
         end
    end
end
for i=2:N
    for j=1:N
         dx(i,j)=cos(ltheta(i,j))*a;
         dy(i,j)=sin(ltheta(i,j))*a;
    end
end
for i=2:N
    dfx(i)=(dx(i,:)*(A(i,:))')/sum(A(i,:));
    dfy(i)=(dy(i,:)*(A(i,:))')/sum(A(i,:));
end

