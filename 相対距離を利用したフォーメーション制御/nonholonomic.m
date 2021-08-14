%非ホロノミックロボットの制約を回避するために
function [omega2,v2] = nonholonomic(omega,v,N,omega2,v2)

for i=1:N
    if (((-pi/2)<=omega(i) && omega(i)<=0) || (omega(i)>=0 && omega(i)<=(pi/2)))
        omega2(i)=omega(i);
    else 
        omega2(i)=omega(i)+pi;
    end 
end
for i=1:N
    if (((-pi/4)<=omega(i) && omega(i)<=0) || (omega(i)>=0 && omega(i)<=(pi/4)))
        v2(i)=v(i);
    elseif (pi/4)<=omega(i)&&omega(i)<=(3*pi/4)
        v2(i)=v(i)/tan(omega(i));
    elseif  (((-3*pi/4)<=omega(i)) && (omega(i)<=(-pi/4)))
        v2(i)=-v(i)/tan(omega(i));
    elseif (pi*3/4<=omega(i) && omega(i)<=pi) || (-pi<=omega(i) && omega(i)<=-pi*3/4)
        v2(i)=-v(i);
    end
end
end
