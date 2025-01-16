% Funzione per il plot 3d del robot, ripiano e scatola. Il tutto è
% parametrizzato in modo che in funzione dei valori inseriti dall'utente (L, Pick,PLace)
% gli oggetti nel plot siano ben visualizzabili. 
function PlotScara3D(Q, L, fig,traccia,pickPosition,placePosition,homePosition,tempPick,tempPlace,ripiano)
    figure(fig)
    cla; 
    hold on;
 
    q1 = Q(2); q2 = Q(3); q3 = Q(4);  
    l1 = L(2);  l2 = L(3);  l3 = L(4);hmax = L(1); 
     
    forkRatio = 0.30;  
    lengthFork = forkRatio * l3;  
    lengthLink = (1 - forkRatio) * l3; 
    % converti q per plot 3d
    x1 = l1 * cos(q1);y1 = l1 * sin(q1);z1 = Q(1); 
    x2 = x1 + l2 * cos(q1 + q2);y2 = y1 + l2 * sin(q1 + q2); z2 = z1;  
    x3 = x2 + lengthLink * cos(q1 + q2 + q3); y3 = y2 + lengthLink * sin(q1 + q2 + q3);z3 = z2;

   % braccia
    plot3([0 0 x1 x2 x3], [0 0 y1 y2 y3], [0 Q(1) z1 z2 z3], 'LineWidth', 4, 'color', 'k');
    plot3([0 0], [0 0], [0 0.150], 'LineWidth', 4, 'color', 'k');

     
    plot3(0, 0, Q(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(x1, y1, Q(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); 
    plot3(x2, y2, Q(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'g');  
     
    % parametri forca
    widthFork = lengthFork; 
    fork_x1 = x3 + lengthFork * cos(q1 + q2 + q3); fork_y1 = y3 + lengthFork * sin(q1 + q2 + q3);fork_z1 = z3;  
    offset_x = widthFork * cos(q1 + q2 + q3 + pi/2);  
    offset_y = widthFork * sin(q1 + q2 + q3 + pi/2);
    fork1_x_start = x3 + offset_x; fork1_y_start = y3 + offset_y; fork1_z_start = z3;
    fork1_x_end = fork_x1 + offset_x; fork1_y_end = fork_y1 + offset_y; fork1_z_end = fork_z1;
    fork2_x_start = x3 - offset_x;fork2_y_start = y3 - offset_y;fork2_z_start = z3;
    fork2_x_end = fork_x1 - offset_x; fork2_y_end = fork_y1 - offset_y; fork2_z_end = fork_z1;
    
    plot3([fork1_x_start fork1_x_end], [fork1_y_start fork1_y_end], [fork1_z_start fork1_z_end], 'LineWidth', 3, 'color', 'k');
    plot3([fork2_x_start fork2_x_end], [fork2_y_start fork2_y_end], [fork2_z_start fork2_z_end], 'LineWidth', 3, 'color', 'k');
    plot3([fork1_x_start fork2_x_start], [fork1_y_start fork2_y_start], [fork1_z_start fork2_z_start], 'LineWidth', 3, 'color', 'k');
    
    %base
    baseSize = (l1 + l2 + l3) * 0.25;  
    square_x = [-1, 1, 1, -1, -1] * baseSize;
    square_y = [-1, -1, 1, 1, -1] * baseSize;
    square_z = zeros(size(square_x));
    fill3(square_x, square_y, square_z, [0.8, 0.8, 0.8]);

    shelfHeight = hmax;
    shelfWidth = baseSize * 0.8;
    
    % Calcolo degli offset in base a placePosition
    if placePosition(4) == 0
        xOffset = 0.05; 
        yOffset = 0;   
    elseif placePosition(4) == pi/2
        xOffset = 0;   
        yOffset = 0.05; 
    elseif placePosition(4) == pi
        xOffset = -0.05; 
        yOffset = 0;    
    elseif placePosition(4) == -pi/2
        xOffset = 0;    
        yOffset = -0.05; 
    else
        xOffset = 0;
        yOffset = 0;
    end
    
    % Calcolo della posizione centrata del ripiano
    centerX = placePosition(1) + xOffset; 
    centerY = placePosition(2) + yOffset; 
    
    % Disegno dei ripiani
    for i = 1:3
        zOffset = i * (shelfHeight / 3); 
        shelf_x = [-1, 1, 1, -1, -1] * shelfWidth / 2 + centerX; 
        shelf_y = [-1, -1, 1, 1, -1] * shelfWidth / 2 + centerY; 
        shelf_z = ones(size(shelf_x)) * zOffset; 

        fill3(shelf_x, shelf_y, shelf_z, [0.8, 0.8, 0.8]);
    end

    if pickPosition(4) == 0
        xOffsetground = pickPosition(1) + 0.05;  
        yOffsetground = pickPosition(2); 
    elseif pickPosition(4) == pi/2 
        xOffsetground = pickPosition(1); 
        yOffsetground = pickPosition(2) + 0.05;
    elseif pickPosition(4) == pi
        xOffsetground = pickPosition(1) - 0.05; 
        yOffsetground = pickPosition(2); 
    elseif pickPosition(4) == -pi/2
        xOffsetground = pickPosition(1); 
        yOffsetground = pickPosition(2) - 0.05;
    else
        xOffsetground = pickPosition(1); 
        yOffsetground = pickPosition(2); 
    end

    shelfDepth = shelfWidth; 
    groundShelf_x = [-1, 1, 1, -1, -1] * shelfWidth / 2 + xOffsetground;
    groundShelf_y = [-1, -1, 1, 1, -1] * shelfDepth / 2 + yOffsetground;
    groundShelf_z = ones(size(groundShelf_x)) * pickPosition(3); 
    
    % ripiano
    fill3(groundShelf_x, groundShelf_y, groundShelf_z, [0.8, 0.8, 0.8]);
    cubeSize = l3/2;
    
    % posizione del cubo
    if pickPosition(4) == 0
        xOffsetbox = pickPosition(1) + 0.05;
        yOffsetbox = pickPosition(2);
    elseif pickPosition(4) == pi/2
        xOffsetbox = pickPosition(1);
        yOffsetbox = pickPosition(2) + 0.05;
    elseif pickPosition(4) == pi
        xOffsetbox = pickPosition(1) - 0.05;
        yOffsetbox = pickPosition(2);
    elseif pickPosition(4) == -pi/2
        xOffsetbox = pickPosition(1);
        yOffsetbox = pickPosition(2) - 0.05;
    else
        xOffsetbox = pickPosition(1);
        yOffsetbox = pickPosition(2);
    end
   
    if traccia == 1
        vertices = cubeSize / 2 * [
            -1, -1, -1;
             1, -1, -1;
             1,  1, -1;
            -1,  1, -1;
            -1, -1,  1;
             1, -1,  1;
             1,  1,  1;
            -1,  1,  1
        ] + [xOffsetbox, yOffsetbox, pickPosition(3)+hmax/12];
    
    elseif traccia == 2
        zOffset = cubeSize/2;
      
        center_x = (fork1_x_start + fork2_x_start + fork1_x_end + fork2_x_end) / 4;
        center_y = (fork1_y_start + fork2_y_start + fork1_y_end + fork2_y_end) / 4;
        center_z = (fork1_z_start + zOffset); 
    
        cubeLength = l3/2; cubeWidth = l3/2; cubeHeight = l3/2;
    
        halfLength = cubeLength / 2; halfWidth = cubeWidth / 2;halfHeight = cubeHeight / 2;
    
        theta = q1 + q2 + q3; 
        Rz = [
            cos(theta), -sin(theta), 0;
            sin(theta),  cos(theta), 0;
            0,           0,          1
        ];
    
        baseVertices = [
            -halfLength, -halfWidth, -halfHeight;
             halfLength, -halfWidth, -halfHeight;
             halfLength,  halfWidth, -halfHeight;
            -halfLength,  halfWidth, -halfHeight;
            -halfLength, -halfWidth,  halfHeight;
             halfLength, -halfWidth,  halfHeight;
             halfLength,  halfWidth,  halfHeight;
            -halfLength,  halfWidth,  halfHeight
        ];

        vertices = (Rz * baseVertices')' + [center_x, center_y, center_z];
    
      else
        vertices = cubeSize / 2 * [
            -1, -1, -1;
             1, -1, -1;
             1,  1, -1;
            -1,  1, -1;
            -1, -1,  1;
             1, -1,  1;
             1,  1,  1;
            -1,  1,  1
        ] + [centerX, centerY, str2double(ripiano)*(shelfHeight / 3)+hmax/12];
    end
    faces = [
        1, 2, 3, 4;  
        5, 6, 7, 8;  
        1, 2, 6, 5;  
        2, 3, 7, 6;  
        3, 4, 8, 7;  
        4, 1, 5, 8;  
    ];

    % print legenda, punti
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', 'k');
    
    hold on;
    coordinates = [homePosition, pickPosition, placePosition, tempPick, tempPlace];
    colors = ['r', 'g', 'b','c','k']; 
    labels = {'home', 'pick', 'place','Generated temp. Pick', 'Generated temp. Place'};
    h = zeros(size(coordinates, 2), 1);

    for i = 1:size(coordinates, 2)
        h(i) = plot3(coordinates(1, i), coordinates(2, i), coordinates(3, i), ...
                     'o', 'MarkerSize', 5, 'MarkerFaceColor', colors(i), 'MarkerEdgeColor', 'k');
    end
    legend(h, labels, 'Location', 'best');  
    patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'b', 'EdgeColor', 'k');
    end
