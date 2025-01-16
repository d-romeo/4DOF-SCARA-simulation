function res=linesAndParabola2(trace, aMotor, dMotor, vMotor)
    tbar = vMotor * (1/aMotor + 1/dMotor);
    deltaqBar = 0.5 * vMotor^2 * (1/aMotor + 1/dMotor);
    Qhome = trace(:,1);
    traceElement = size(trace, 2);
    dQmat = [];
    for i=1:(traceElement-1)
        colonna_i = trace(:, i);
        colonna_f = trace(:, i+1);
        dQ = colonna_f - colonna_i;
        dQmat = [dQmat, dQ];
    end
    for i = 1:(traceElement - 1)
        tmin(i) = tempoMinimo(dQmat(:, i), deltaqBar, vMotor, aMotor, dMotor,tbar);
    end
    
    dQmat1 = dQmat(:,1);
    dQmatend =dQmat(:,traceElement-1);
    for i = 1:4
        t1(i) = tmin(1) - sqrt((tmin(1)^2 - (2*abs(dQmat1(i))/aMotor)));
    end
    
    for i = 1:4
        tend(i) = tmin(traceElement-1) - sqrt((tmin(traceElement-1)^2 - (2*abs(dQmatend(i))/dMotor)));
    end
    for i = 1:4
        tmin1(i) =tmin(1) - 0.5*t1(i);
        tminEnd(i) = tmin(traceElement-1) - 0.5*tend(i);
    end
    tminj = zeros(4,traceElement-1);
    tminj = tmin1';
    for i = 2:(traceElement - 2)
        tmindopo = [tmin(i), tmin(i), tmin(i), tmin(i)]';
        tminj(:,i) = tmindopo;
    end
    tminj(:,traceElement - 1) = tminEnd';
    for i = 1:4
        for j = 2:(traceElement -1)
            tj(i,j-1) = abs(((dQmat(i,j)/tminj(i,j))-(dQmat(i,j-1)/tminj(i,j-1)))/aMotor);
        end
    end
    deltaTj = zeros(4,traceElement - 1);
    deltaTj(:,1) = tminj(:,1) - (tj(:,1)/2) - (t1'*0.5);
    deltaTj(:,traceElement-1) = tminj(:,traceElement-1) - (tj(:,traceElement-2)/2) - (tend'*0.5);
    for i = 2:(traceElement-2)
        deltaTj(:,i)  = tminj(:,i) - (tj(:,i)/2) - (tj(:,i-1)/2);
    end
    for i = 1:4
        for j = 1:(traceElement - 1)
            qdot(i,j) = dQmat(i,j)/tminj(i,j);
        end
    end
    tvec = zeros(4, 1);
    current_column = t1';
    numStepsLine = size(deltaTj,2);
    numStepsPar = size(tj,2);
    totSteps= numStepsLine + numStepsPar;
    actualNumStepLine = 1;
    actualNumStepPar = 1;
    tvec = [tvec,t1'];
    %tvec1 = [zeros(4,1), t1', t1'+deltaTj(:,1), t1'+deltaTj(:,1)+ tj(:,1), t1'+deltaTj(:,1)+ tj(:,1)+deltaTj(:,2), t1'+deltaTj(:,1)+ tj(:,1)+deltaTj(:,2)+tj(:,2),t1'+deltaTj(:,1)+ tj(:,1)+deltaTj(:,2)+tj(:,2)+deltaTj(:,3),t1'+deltaTj(:,1)+ tj(:,1)+deltaTj(:,2)+tj(:,2)+deltaTj(:,3)+tend']
    for j = 1:totSteps
        if mod(j, 2) == 1 %dispari
            current_column = current_column + deltaTj(:,actualNumStepLine);
            actualNumStepLine = actualNumStepLine + 1;
        end
        if mod(j, 2) == 0
            current_column = current_column + tj(:,actualNumStepPar);
            actualNumStepPar = actualNumStepPar + 1;
        end
        tvec = [tvec, current_column];
    end
    tvec = [tvec , current_column + tend'];
    res.tLine = zeros(4, 1);
    temp = 0.5 * t1';
    res.tLine = [res.tLine, temp];
    for j = 1:(traceElement - 1)
        res.tLine = [res.tLine, res.tLine(:, end) + tminj(:, j)];
    end
    res.tLine = [res.tLine, res.tLine(:, end) + 0.5 * tend'];
    q = [Qhome,Qhome];
    for j = 1:(traceElement - 1)
        q = [q, q(:, end) + dQmat(:, j)];
    end
    bl = zeros(4,traceElement-1);
    al = qdot;
    q = [q,q(:,end)];
    qElement = size(q,2);
    count = 1;
    for i = 1:4
        for j = 3: qElement-1
            bl(i,count) = q(i,j) - (al(i,count)*res.tLine(i,j));
            count = count + 1;
        end
        count = 1;
    end
    res.q = q;
    % matrice ap
    ap = zeros(4, traceElement);
    for i = 1:4
        ap(i,1) = al(i,1)/(2*tvec(i,2));
    end
    %ultimo elemento matrice ap
    for i = 1:4
        for j = 2 : (traceElement-1)
            index = 2*j;
            ap(i,j) = (al(i,j)-al(i,j-1))/(2*(tvec(i,index)-tvec(i,index-1)));
        end
    end
    tvecElement = size(tvec,2);
    for i = 1:4
        ap(i,traceElement) = (0-al(i,traceElement-1))/(2*(tvec(i,tvecElement)-tvec(i,tvecElement-1)));
    end
    % matrice bp
    bp =  zeros(4, traceElement);
    for i = 1:4
        for j = 1 : (traceElement-1)
            index=j*2;
            bp(i,j) = qdot(i,j)-(2*(ap(i,j)*tvec(i,index)));
        end
    end
    % ultima colonna matrice bp
    for i = 1:4
        bp(i,traceElement) = (0-(2*(ap(i,traceElement)*tvec(i,tvecElement))));
    end
    
    for i = 1:4
        for j = 1 : (traceElement-1)
            index = 2*j;
            x(i,j) = (al(i,j)*(tvec(i,index)))+ bl(i,j);
        end
    end
    % costruzione cp usando la matrice x calcolata precedentmente
    cp =  zeros(4, traceElement);
    for i = 1:4
        for j = 1 : (traceElement-1)
            index=j*2;
            cp(i,j) = x(i,j)-((ap(i,j)*(tvec(i,index))^2))- (bp(i,j)*tvec(i,index));
        end
    end
    % ultima colonna matrice cp
    for i = 1:4
        cp(i,traceElement) = trace(i,traceElement)-((ap(i,traceElement)*(tvec(i,tvecElement))^2))-(bp(i,traceElement)*tvec(i,tvecElement));
    end
    
    tLineElement = size(res.tLine,2);
    
    numGiunti = 4;
    numPuntiPerIntervallo = 100;
    numIntervalli = size(tvec, 2) - 1;
    res.vettore_posizioni = zeros(numGiunti, numPuntiPerIntervallo * numIntervalli);
    
    for i = 1:numGiunti
        countPar = 1;
        countLin = 1;
        colIndex = 1;
    
    
        for j = 1:numIntervalli
            t_start = tvec(i, j);
            t_end = tvec(i, j + 1);
    
            if j == numIntervalli
                t_range = linspace(t_start, t_end, numPuntiPerIntervallo);
            else
                t_range = linspace(t_start, t_end, numPuntiPerIntervallo + 1);
                t_range = t_range(1:end-1);
            end
    
            if mod(j, 2) == 1
                temp = ap(i, countPar) * t_range.^2 + ...
                    bp(i, countPar) * t_range + ...
                    cp(i, countPar);
                countPar = countPar + 1;
            else
                temp = al(i, countLin) * t_range + bl(i, countLin);
                countLin = countLin + 1;
            end
    
            % Salva i risultati nel matricione
            res.vettore_posizioni(i, colIndex:colIndex + length(t_range) - 1) = temp;
            res.tempo(i, colIndex:colIndex + length(t_range) - 1) = t_range;
            colIndex = colIndex + length(t_range);
        end
    end
    
    step=size(res.vettore_posizioni,2);
    for i=1:4
        for j=2:step
            vel(i,j-1)=(res.vettore_posizioni(i,j)-res.vettore_posizioni(i,j-1))/(res.tempo(i,j)-res.tempo(i,j-1));
        end
    end
    res.vel = [vel, zeros(4,1)];
    for i = 1:4
        for j = 2:step-1
            res.acc(i,j) = (res.vel(i,j+1) - res.vel(i,j-1)) / (res.tempo(i,j+1) - res.tempo(i,j-1));
        end
    end
end

             