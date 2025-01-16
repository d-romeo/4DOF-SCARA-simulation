% Funzione per ottenere i punti temporanei dinamicamente prima del 
% punto di pick e place.
function points = tempPoint(point,ripiano)
    x = point(1); y = point(2); z = point(3);
    phi = point(4);

    % offset z variabile, non supera hmax
    if ripiano == 3 
       z = z - 0.01; 
    else 
        z = z + 0.01; 
    end

    if phi == 0
        points = [x-x/4; y ; z; phi]; 
        return; 
    end
    if phi == pi/2 
        points = [x; y-y/4; z; phi];
        return;
    end
    if phi == pi
         points = [x+abs(x)/4; y; z; phi];
         return;
    else 
        points = [x; y+abs(y)/4; z; phi];
        return;
    end
end