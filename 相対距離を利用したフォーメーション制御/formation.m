function  [dfx,dfy]=formation(A,a,N)
    mdx=zeros(N,N);
    mdy=zeros(N,N);
    dx=zeros(1,N);
    dy=zeros(1,N);
    dx(2)=a; dy(2)=0;
    dx(3)=a/2; dy(3)=(sqrt(3)*a)/2;
    dx(4)=-a/2; dy(4)=(sqrt(3)*a)/2;
    dx(5)=-a; dy(5)=0;
    dx(6)=-a/2; dy(6)=-(sqrt(3)*a)/2;
    dx(7)=a/2; dy(7)=-(sqrt(3)*a)/2;
    for i=1:N
        for j=1:N
            mdx(i,j)=dx(i)-dx(j);
            mdy(i,j)=dy(i)-dy(j);
        end
    end
    for i=2:N
        dfx(i)=(mdx(i,:)*(A(i,:))')/sum(A(i,:));
        dfx(i)=(mdy(i,:)*(A(i,:))')/sum(A(i,:));
    end
end