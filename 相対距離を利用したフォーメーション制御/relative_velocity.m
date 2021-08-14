function  [drx,dry]=relative_velocity(A,ux,uy,theta,N)
    drx=zeros(1,N);
    dry=zeros(1,N);
    rux=zeros(N,N);
    ruy=zeros(N,N);
    for i=1:N
        wux(i)=ux(i)*cos(theta(i))-uy(i)*sin(theta(i)); %ワールド座標系の速度
        wuy(i)=ux(i)*sin(theta(i))+uy(i)*cos(theta(i));
        for j=2:N
            R=[cos(theta(j)) -sin(theta(j));
                  sin(theta(j)) cos(theta(j))]; %回転行�??
            ru=R'*[wux(i);wuy(i)];   
            rux(j,i)=ru(1,1); %jから見たiの相対速度
            ruy(j,i)=ru(2,1);
        end
    end
%     wrux=rux+wgn(N,N,15);%ノイズを加える
%     wruy=ruy+wgn(N,N,15);
    wrux=rux;%ノイズを加える
    wruy=ruy;
    for i=2:N
%     drx(i)=(rux(i,:)*(A(i,:))')/sum(A(i,:));
%     dry(i)=(ruy(i,:)*(A(i,:))')/sum(A(i,:));
      drx(i)=(wrux(i,:)*(A(i,:))')/sum(A(i,:));
      dry(i)=(wruy(i,:)*(A(i,:))')/sum(A(i,:));
    end
end