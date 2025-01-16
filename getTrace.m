% Funzione che, date le posizioni in ingresso ne calcola la
% cinematica inversa e applica la trasmissione. 
function trace = getTrace(positions,tau,L)
    lengthPositions = size (positions,2); 
     Q = zeros(4,lengthPositions);  
    for i= 1:lengthPositions
        Q(:,i) = SCARAinv(positions(:,i),L,1); 
    end
    trace = zeros(4,lengthPositions); 
    for i = 1:lengthPositions
        trace(:, i) = Q(:, i) ./ diag(tau);
    end
   
