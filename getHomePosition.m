% Funzione che definisce il punto homePosition in modo dinamico in funzione
% dei parametri scelti dall'utente. 
function res = getHomePosition(L)
    Sz = L(1)/3; 
    l2 = L(2);
    l3 = L(3);
    l4 = L(4);

    q2 = -0.0614114263671467; 
    q3 = 2.37058295534036; 
    q4 = -1.52377336557576; 
    res.phi = pi/4; 
    res.Sx = l2*cos(q2) + l3*cos(q2+q3)+ l4*cos(q2+q3+q4); 
    res.Sy = l2*sin(q2) + l3*sin(q2+q3)+ l4*sin(q2+q3+q4); 
    res.Sz = L(1)/3; 
end