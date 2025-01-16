function Jac = jacobiano(L, Q)
    l2 = L(2); l3 = L(3); l4 = L(4);
    q1 = Q(1);q2 = Q(2); q3 = Q(3);q4 = Q(4);

    Jac = zeros(3,4);
    Jac(1,2) = -l2*sin(q2) -l3*sin(q2+q3) -l4*sin(q2+q3+q4); 
    Jac(1,3)= -l3*sin(q2+q3)-l4*sin(q2+q3+q4);
    Jac(1,4)= -l4*sin(q2+q3+q4);

    Jac(2,2)= l2*cos(q2)+l3*cos(q2+q3)+l4*cos(q2+q3+q4);
    Jac(2,3)= l3*cos(q2+q3)+l4*cos(q2+q3+q4);
    Jac(2,4)= l4*cos(q2+q3+q4);

    Jac(3,1)=1;
    Jac(3,2)=0;
    Jac(3,3)=0;
    Jac(3,4)=0;
end