function Jp=SCARAjacPdin(Q,Qp,L)
    l1=L(2); l2=L(3); l3=L(4);
    
    Jp = zeros(16, 4); 
    
    Jp(1, 2) = -l1*cos(Q(2))*Qp(2) - l2*cos(Q(2) + Q(3))*(Qp(2) + Qp(3)) - l3*cos(Q(2) + Q(3) + Q(4))*(Qp(2) + Qp(3) + Qp(4));
    Jp(1, 3) = -l2*cos(Q(2) + Q(3))*(Qp(2) + Qp(3)) - l3*cos(Q(2) + Q(3) + Q(4))*(Qp(2) + Qp(3) + Qp(4));
    Jp(1, 4) = -l3*cos(Q(2) + Q(3) + Q(4))*(Qp(2) + Qp(3) + Qp(4));
    
    Jp(2, 2) = -l1*sin(Q(2))*Qp(2) - l2*sin(Q(2) + Q(3))*(Qp(2) + Qp(3)) - l3*sin(Q(2) + Q(3) + Q(4))*(Qp(2) + Qp(3) + Qp(4));
    Jp(2, 3) = -l2*sin(Q(2) + Q(3))*(Qp(2) + Qp(3)) - l3*sin(Q(2) + Q(3) + Q(4))*(Qp(2) + Qp(3) + Qp(4));
    Jp(2, 4) = -l3*sin(Q(2) + Q(3) + Q(4))*(Qp(2) + Qp(3) + Qp(4));
    
    Jp(5, 2) = -l1*cos(Q(2))*Qp(2) - (l2/2)*cos(Q(2) + Q(3))*(Qp(2) + Qp(3));
    Jp(5, 3) = -(l2/2)*cos(Q(2) + Q(3))*(Qp(2) + Qp(3));
    
    Jp(6, 2) = -l1*sin(Q(2))*Qp(2) - (l2/2)*sin(Q(2) + Q(3))*(Qp(2) + Qp(3));
    Jp(6, 3) = (l2/2)*sin(Q(2) + Q(3))*(Qp(2) + Qp(3));
    
    Jp(9, 2) = -(l1/2)*cos(Q(2))*Qp(2);
    
    Jp(10, 2) = (l1/2)*sin(Q(2))*Qp(2);

end

