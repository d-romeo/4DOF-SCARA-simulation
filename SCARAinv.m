function Q=SCARAinv(S,L,sol)
    Q = zeros(4,1); 
    x = S(1); y = S(2); z = S(3); phi_target = S(4); 
    l2 = L(2); l3 = L(3); l4 = L(4); hmax = L(1); 
    Q(1) = z;
    
    if Q(1)>hmax
         error('Valore di Q(1) (%f) supera il limite massimo consentito (%f).', Q(1), hmax);
    end

    x_prime = x - l4 * cos(phi_target); y_prime = y - l4 * sin(phi_target);
       
    beta = acos((x_prime^2 + y_prime^2 - l2^2 - l3^2) / (2 * l2 * l3));
        
    if (sol > 0)
        Q(3) = beta;  
    else
        Q(3) = -beta;
    end
   
    Q(2) = atan2(y_prime, x_prime) - atan2(l3 * sin(Q(3)), l2 + l3 * cos(Q(3)));
    Q(4) = phi_target - (Q(2) + Q(3));  
    
    % [-pi, pi]
    Q(4) = mod(Q(4) + pi, 2*pi) - pi; 

end