function tmax = tempoMinimo(dqvec, qBar, v, a, d,tBar)
    tmax = 0;
    for i= 1:4
        if (abs(dqvec(i))>qBar) 
            t = abs(dqvec(i))/v + v*(a+d)/(2*a*d); 
            if t>tmax
                tmax=t;
            end
        else 
            t = (sqrt(a/d) + sqrt(d/a)) * sqrt(2*abs(dqvec(i))*(1/(a+d)));
            if t>tmax
                tmax=t;
            end 
        end
    end

    % PER IL PLOT GRAFICO DELLA PRESENTAZIONE
    % q = 0:0.01:2*pi;
    % 
    % t1 = zeros(size(q)); 
    % t2 = zeros(size(q)); 
    % 
    % for i = 1:length(q)
    %     if q(i) >= qBar
    %         t1(i) = abs(q(i))/v + v*(a+d)/(2*a*d);
    %     else
    %         t2(i) = (sqrt(a/d) + sqrt(d/a)) * sqrt(2*abs(q(i))*(1/(a+d)));
    %     end
    % end
    % 
    % q_t1 = q(t1 ~= 0);
    % t1 = t1(t1 ~= 0);
    % 
    % q_t2 = q(t2 ~= 0);
    % t2 = t2(t2 ~= 0);
    % dqVec = [pi/4, pi/2, 3*pi/4, pi]; % Esempio di valori
    % colors = lines(length(dqVec)); % Genera colori distinti per ogni joint
    % 
    % t_dqVec = zeros(size(dqVec));
    % for j = 1:length(dqVec)
    %     if dqVec(j) >= qBar
    %         t_dqVec(j) = abs(dqVec(j))/v + v*(a+d)/(2*a*d);
    %     else
    %         t_dqVec(j) = (sqrt(a/d) + sqrt(d/a)) * sqrt(2*abs(dqVec(j))*(1/(a+d)));
    %     end
    % end
    % 
    % % Per plottare, combini i valori
    % figure(20)
    % hold on
    % grid on
    % 
    % 
    % h1 = plot(q_t1, t1, 'LineWidth', 2, 'DisplayName', 'q \geq qBar'); 
    % h2 = plot(q_t2, t2, 'LineWidth', 2, 'DisplayName', 'q < qBar'); 
    % yline(tBar, '--k', 'LineWidth', 1.5, 'DisplayName', 'tBar'); 
    % xline(qBar, '--k', 'LineWidth', 1.5, 'DisplayName', 'qBar'); 
    % lineHandles = [];
    % for k = 1:length(dqVec)
    %     % Linea verticale fino al punto
    %     lineHandles(k) = plot([dqVec(k), dqVec(k)], [0, t_dqVec(k)], '-', 'Color', colors(k, :), 'LineWidth', 1.5); 
    %     % Punto
    %     plot(dqVec(k), t_dqVec(k), 'o', 'Color', colors(k, :), 'MarkerSize', 8, 'MarkerFaceColor', colors(k, :));
    % end
    % title('Minimum Time', 'FontSize', 14, 'FontWeight', 'bold');
    % legend([h1, h2, lineHandles], {'q \geq qBar', 'q < qBar', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4'}, 'Location', 'best');
    % 
    % hold off

end