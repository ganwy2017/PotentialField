function create_map()
    close all
    WIDTH = 2;
    HEIGHT = 3;
    ROB_RADIUS = 0.13;
    THRESHOLD = 0.7;
    K_REPULSIVE = -1;
    K_ATTRACTIVE = 1e5;
    myquiver = 0;
    mycontour = 0;
    attractiverectangle = 0;
    ui= zeros(1, 12);
    targetpoint = 0;
    ginx = 0;
    giny = 0;
    doanimate = 0;
    trace = zeros(1, 500);
    speed = 0.05;
    distances_to_go = [];
    relations = [];
    pathlines = zeros(1, 50);
     
    bigcells = [];
    bigcells = [bigcells Cell(0, -1.000, -0.500, 0.830, 1.500)];
    bigcells = [bigcells Cell(1, -0.500, 0.500, 0.830, 1.500)];
    bigcells = [bigcells Cell(2, 0.500, 1.000, 1.000, 1.500)];
    bigcells = [bigcells Cell(3, -1.000, -0.500, 0.100, 0.830)];
    bigcells = [bigcells Cell(4, -0.500, -0.200, 0.400, 0.830)];
    bigcells = [bigcells Cell(5, -0.200, 0.200, 0.400, 0.830)];
    bigcells = [bigcells Cell(6, 0.200, 0.500, 0.400, 0.830)];
    bigcells = [bigcells Cell(7, 0.500, 1.000, 0.000, 0.300)];
    bigcells = [bigcells Cell(8, -0.500, -0.200, 0.100, 0.400)];
    bigcells = [bigcells Cell(9, 0.200, 0.500, 0.300, 0.400)];
    bigcells = [bigcells Cell(10, -0.100, 0.200, -0.300, 0.300)];
    bigcells = [bigcells Cell(11, 0.200, 0.500, -0.300, 0.300)];
    bigcells = [bigcells Cell(12, -1.000, -0.500, -0.830, -0.100)];
    bigcells = [bigcells Cell(13, -0.500, -0.200, -0.400, -0.100)];
    bigcells = [bigcells Cell(14, 0.200, 0.500, -0.400, -0.300)];
    bigcells = [bigcells Cell(15, 0.500, 1.000, -0.300, 0.000)];
    bigcells = [bigcells Cell(16, -0.500, -0.200, -0.830, -0.400)];
    bigcells = [bigcells Cell(17, -0.200, 0.200, -0.830, -0.400)];
    bigcells = [bigcells Cell(18, 0.200, 0.500, -0.830, -0.400)];
    bigcells = [bigcells Cell(19, -1.000, -0.500, -1.500, -0.830)];
    bigcells = [bigcells Cell(20, -0.500, 0.500, -1.500, -0.830)];
    bigcells = [bigcells Cell(21, 0.500, 1.000, -1.500, -1.000)];
    bigcells = [bigcells Cell(22, 0.500, 1.000, 0.400, 0.830)];
    bigcells = [bigcells Cell(23, 0.500, 1.000, -0.830, -0.400)];
    bigcells = [bigcells Cell(24, 0.500, 1.000, 0.300, 0.400)];
    bigcells = [bigcells Cell(25, 0.500, 1.000,  -0.400, -0.300)];
    bigcells = [bigcells Cell(26, -1.000, -0.500,  -0.100, 0.100)];
    bigcells = [bigcells Cell(27, 0.500, 1.000,  -1.000, -0.830)];
    bigcells = [bigcells Cell(28, 0.500, 1.000, 0.830, 1.000)];

    
    relations = [relations Relation(0, 0, 1, Direction.WE)];
    relations = [relations Relation(1, 1, 5, Direction.NS)];
    relations = [relations Relation(2, 1, 4, Direction.NS)];
    relations = [relations Relation(3, 1, 6, Direction.NS)];
    relations = [relations Relation(4, 2, 28, Direction.NS)];
    relations = [relations Relation(5, 4, 5, Direction.WE)];
    relations = [relations Relation(6, 5, 6, Direction.WE)];
    relations = [relations Relation(7, 6, 22, Direction.WE)];
    relations = [relations Relation(8, 3, 4, Direction.WE)];
    relations = [relations Relation(9, 3, 8, Direction.WE)];
    relations = [relations Relation(10, 4, 8, Direction.NS)];
    relations = [relations Relation(11, 6, 9, Direction.NS)];
    relations = [relations Relation(12, 9, 7, Direction.WE)];
    relations = [relations Relation(13, 10, 11, Direction.WE)];
    relations = [relations Relation(14, 11, 7, Direction.WE)];
    relations = [relations Relation(15, 7, 15, Direction.NS)];
    relations = [relations Relation(16, 14, 15, Direction.WE)];
    relations = [relations Relation(17, 15, 25, Direction.NS)];
    relations = [relations Relation(18, 3, 12, Direction.NS)];
    relations = [relations Relation(19, 12, 13, Direction.WE)];
    relations = [relations Relation(20, 12, 16, Direction.WE)];
    relations = [relations Relation(21, 13, 16, Direction.NS)];
    relations = [relations Relation(22, 16, 17, Direction.WE)];
    relations = [relations Relation(23, 17, 18, Direction.WE)];
    relations = [relations Relation(24, 18, 23, Direction.WE)];
    relations = [relations Relation(25, 16, 20, Direction.NS)];
    relations = [relations Relation(26, 17, 20, Direction.NS)];
    relations = [relations Relation(27, 18, 20, Direction.NS)];
    relations = [relations Relation(28, 11, 15, Direction.WE)];
    relations = [relations Relation(29, 19, 20, Direction.WE)];
    relations = [relations Relation(30, 11, 14, Direction.NS)];
    relations = [relations Relation(31, 11, 9, Direction.SN)];
    relations = [relations Relation(32, 23, 27, Direction.NS)];
    relations = [relations Relation(33, 22, 24, Direction.NS)];
    relations = [relations Relation(34, 14, 18, Direction.NS)];
    relations = [relations Relation(35, 24, 7, Direction.NS)];
    relations = [relations Relation(36, 25, 23, Direction.NS)];
    relations = [relations Relation(37, 27, 21, Direction.NS)];
    relations = [relations Relation(38, 28, 22, Direction.NS)];


    sources = [];
    sources = [sources Source( (-0.150 ),  ( 0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.150 ),  ( 0.150 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.150 ),  ( 0.250 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.150 ),  ( 0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.050 ),  ( 0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.050 ),  ( 0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.150 ),  ( 0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.150 ),  ( -0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.150 ),  ( -0.150 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.150 ),  ( -0.250 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.150 ),  ( -0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.050 ),  ( -0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.050 ),  ( -0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.150 ),  ( -0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.250 ),  ( 0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.350 ),  ( 0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.450 ),  ( 0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.450 ),  ( -0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.350 ),  ( -0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.250 ),  ( -0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.950 ),  ( -0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.850 ),  ( -0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.750 ),  ( -0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.650 ),  ( -0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.550 ),  ( -0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.950 ),  ( 0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.850 ),  ( 0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.750 ),  ( 0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.650 ),  ( 0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source( (-0.550 ),  ( 0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.450 ),  ( -1.050 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.450 ),  ( -1.150 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.450 ),  ( -1.250 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.450 ),  ( -1.350 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.450 ),  ( -1.450 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.450 ),  ( 1.050 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.450 ),  ( 1.150 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.450 ),  ( 1.250 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.450 ),  ( 1.350 ), SourceType.REPULSIVE)];
    sources = [sources Source( (0.450 ),  ( 1.450 ), SourceType.REPULSIVE)];
    
    for si_=0:32
        sources = [sources Source( (-1.050),  ( -1.550+si_*0.100 ), SourceType.REPULSIVE)];
        sources = [sources Source( (1.050),  ( -1.550+si_*0.100 ), SourceType.REPULSIVE)];
    end
    
    for si_=0:22
        sources = [sources Source( (-1.050+si_*0.100),  ( -1.550 ), SourceType.REPULSIVE)];
        sources = [sources Source( (-1.050+si_*0.100),  ( 1.550 ), SourceType.REPULSIVE)];
    end
    
   %  for si_=0:7
   %     sources = [sources Source( (0.200),  ( -0.350 + si_*0.100), SourceType.REPULSIVE)];
   % end
    
    attractiveSource = Source( ( -1 ),  ( 0 ), SourceType.ATTRACTIVE);
    sources = [sources attractiveSource];
    
    path = [];
    
    function set_threshold(source, event)
        THRESHOLD = source.Value;
        
        redraw()
    end

    function set_repulsive_gain(source, event)
        K_REPULSIVE = -10 ^ source.Value;
        redraw()
    end

    function set_attractive_gain(source, event)
        K_ATTRACTIVE = 10 ^ source.Value;
        redraw()
    end

    function resize_cb(~, ~)
        figure(1)
        create_ui()
        figure(1)
        axis([-WIDTH/2 WIDTH/2 -HEIGHT/2 HEIGHT/2]) 
        axis equal
    end

    function animate(source, event)
        if doanimate
            doanimate = 0;
            for i=1:numel(trace)
                if trace(i) ~= 0
                   delete(trace(i)) 
                   trace(i) = 0;
                end
            end
        else
            doanimate = 1;
        end
        tracei = 1;
        F(1:numel(trace)) = struct('cdata',[],'colormap',[]);
        while doanimate
            [fx, fy] = inspectfield(ginx, giny);
            theta = atan2(fy, fx);
            
            ginx = ginx + cos(theta) * speed;
            giny = giny + sin(theta) * speed;
            
            drawnow
            if trace(tracei) ~= 0
               delete(trace(tracei))
            end
            figure(1)
            trace(tracei) = rectangle('Position',...
                [ginx - 0.001, giny - 0.001, 0.002,  0.002],...
                'FaceColor','m',...
                'EdgeColor', 'm',...
                'Curvature', [1 1],...
                'LineWidth', 0.001, 'Parent', gca, 'HitTest', 'off');
            
            
            drawnow
            frame1 = getframe(figure(1));
            frame2 = getframe(figure(2));
            F(tracei).cdata = [ frame1.cdata, frame2.cdata ];
            
            
            tracei = tracei + 1;
            if tracei > numel(trace)
                tracei = 1;
                doanimate = 0;
            end
            
            if norm([ginx giny] - [attractiveSource.x attractiveSource.y]) < speed * 1.5
                doanimate = 0;
            end
        end
        
        v = VideoWriter('newmovie.mp4');
        open(v);
        writeVideo(v, F(1:tracei-1))
        beep

    end
    
    function button_down_cb(source, event)
        if event.Button == 1
            attractiveSource.x = event.IntersectionPoint(1);
            attractiveSource.y = event.IntersectionPoint(2);
            sources = [ sources(1:end-1) attractiveSource];
            redraw();
        elseif event.Button == 3
            ginx = event.IntersectionPoint(1);
            giny = event.IntersectionPoint(2);
            inspectfield(event.IntersectionPoint(1), event.IntersectionPoint(2));
        end
        
    end

    function [fx, fy] = inspectfield(inx, iny)
       figure(1)
       
        if targetpoint ~= 0
            delete(targetpoint)
        end
        targetpoint = rectangle('Position',[inx-ROB_RADIUS/2, iny-ROB_RADIUS/2, ROB_RADIUS ROB_RADIUS],...
            'FaceColor','m',...
            'EdgeColor', 'm',...
            'Curvature', [1 1],...
            'LineWidth', 0.001, 'Parent', gca, 'HitTest', 'off');

        path = compute_path(inx, iny, attractiveSource.x, attractiveSource.y);
        
        figure(2)

        F = compute_all_forces(inx, iny);

        subplot(2,2,1)

        theta = atan2(F(2,:), F(1,:));
        mag = log(sqrt(F(2,:).^2 + F(1,:).^2));
        compass(cos(theta) .* mag, sin(theta) .* mag);
        title('Forces (log scale)')

        subplot(2,2,2)
        fx = sum(F(1,:));
        fy = sum(F(2,:));
        compass(fx, fy);
        title('Total force')

        ftheta = atan2(fy, fx);
        theta = 0:0.001:2*pi;
        l_speed = zeros(1, numel(theta));
        r_speed = zeros(1, numel(theta));
        for i=1:numel(theta)
            robot_relative_theta = (theta(i) - ftheta);
            robot_relative_theta = robot_relative_theta - 2*pi*floor( (robot_relative_theta+pi)/(2*pi) ); 
            if robot_relative_theta <= pi/4 && robot_relative_theta >= -pi/4
                l_speed(i) = 1-sin(robot_relative_theta);
                r_speed(i) = 1+sin(robot_relative_theta);
            elseif(robot_relative_theta > pi/4 && robot_relative_theta < 3*pi/4)
                l_speed(i) = 1;
                r_speed(i) = -1;
            elseif((robot_relative_theta >= 3*pi/4 && robot_relative_theta <= -pi) || (robot_relative_theta >= -pi && robot_relative_theta <= -3*pi/4))
                l_speed(i) = 1;
                r_speed(i) = -1;
            elseif(robot_relative_theta > -3*pi/4 && robot_relative_theta < -pi/4)
                l_speed(i) = -1;
                r_speed(i) = 1;
            end
        end

        subplot(2,1,2);
        polarplot(theta, l_speed, 'b:', theta, r_speed, 'r:');
        title('Wheel Speed wrt Absolute Robot Orientation')
        legend('Left wheel', 'Right wheel') 
        
%         distance_to_go = norm([inx iny] - [attractiveSource.x attractiveSource.y]);
%         angle_to_go = atan2(attractiveSource.y - iny, attractiveSource.x - inx);
        
%         distances_to_go = [distances_to_go distance_to_go];
%         subplot(2,2,4);
%         plot(distances_to_go);
%         if numel(distances_to_go) > 4 &&...
%             (max(distances_to_go(end-4:end)) - min(distances_to_go(end-4:end))) < speed * 0.99
%             title('Distance to objective (trapped)')
%             figure(1)
%             nsx = inx+cos(angle_to_go + rand/2)*speed*1.1;
%             nsy = iny+sin(angle_to_go + rand/2)*speed*1.1;
%             ns = Source(nsx, nsy, SourceType.UNIFORM_CIRCULAR_REPULSIVE);
%             ns.force = -K_ATTRACTIVE * distance_to_go * 1.1;
%             rectangle('Position', [ nsx-0.01 nsy-0.01 0.02 0.02],...
%                 'FaceColor','g',...
%                 'EdgeColor', 'g',...
%                 'Curvature', [1 1],...
%                 'LineWidth', 0.001, 'Parent', gca, 'HitTest', 'off');
%             %sources = [sources ns];
%             
%         else
%             title('Distance to objective')
%         end
    end

    function [path] = compute_path(robx, roby, targetx, targety)
        robotCell = -1;
        targetCell = -1;
        path = [];
        for i=1:numel(bigcells)
           bigcell = bigcells(i);
           if bigcell.inside(robx, roby)
              robotCell = bigcell; 
           end
           if bigcell.inside(targetx, targety)
              targetCell = bigcell; 
           end
        end
        if isa(robotCell, 'Cell') && isa(targetCell, 'Cell')
            path = big_path_finding(robotCell, targetCell, []);
            for pathi=1:numel(pathlines)
               if ~isa(pathlines(pathi), 'double')
                   delete(pathlines(pathi));
                   pathlines(pathi) = 0;
               else
                   break
               end
            end
            for pathi=1:numel(path)-1
                this = path(pathi);
                next = path(pathi+1);
                pathlines(pathi) = line([this.getCenterX() next.getCenterX()],...
                    [this.getCenterY() next.getCenterY()],...
                    'Color', 'c', 'Parent', gca, 'HitTest', 'off');
            end
            
        end
    end

    function [path] = big_path_finding(startCell, stopCell, history)
        path = startCell;
        shortest = -1;
        subpath = [];
        if startCell.id == stopCell.id
            return 
        end
        for i=1:numel(relations)
            rel = relations(i);
            next = -1;
            if rel.from == startCell.id
                next = rel.to;
                direction = rel.direction;
            elseif rel.to == startCell.id
                next = rel.from;
                direction = rel.direction.getopposite();
            end
            
         %   disp ([startCell.id rel.from rel.to next])
            
            if next < 0 || sum(next == history) > 0
                continue
            end
            nextCell = bigcells(next + 1);
            subpath = big_path_finding(nextCell, stopCell, [history startCell.id]);
            if isa(subpath, 'Cell') && (isa(shortest, 'double') || numel(subpath) < numel(shortest))
               shortest = subpath; 
            end
        end
        if isa(shortest, 'double')
            path = -1;
        else
            path = [path shortest];
        end
    end

    function F = compute_all_forces(px, py)
        F = zeros(2, length(sources));
        for si = 1:length(sources)
            s = sources(si);
            dist_x = (s.x - px);
            dist_y = (s.y - py);
            dist_min = max(0, sqrt(dist_x * dist_x + dist_y * dist_y)) - ROB_RADIUS;
            if s.type == SourceType.REPULSIVE ||...
                    s.type == SourceType.CUSTOM_REPULSIVE &&...
                    s.type == SourceType.HIGHLY_REPULSIVE &&...
                    dist_min < THRESHOLD && dist_min > 0
                k = K_REPULSIVE;
                if s.type == SourceType.HIGHLY_REPULSIVE
                    k = k * 100;
                elseif s.type == SourceType.CUSTOM_REPULSIVE
                    k = s.customk;
                end
                
                F(1, si) = k * (1 / dist_min - 1 / THRESHOLD) * (1/(dist_min*dist_min)) * (dist_x/dist_min);
                F(2, si) = k * (1 / dist_min - 1 / THRESHOLD) * (1/(dist_min*dist_min)) * (dist_y/dist_min);
            elseif s.type == SourceType.ATTRACTIVE
                F(1, si) = K_ATTRACTIVE * dist_x;
                F(2, si) = K_ATTRACTIVE * dist_y;
            elseif s.type == SourceType.UNIFORM_CIRCULAR_REPULSIVE
                F(1, si) = s.force * cos(atan2(dist_y, dist_x));
                F(2, si) = s.force * sin(atan2(dist_y, dist_x));
            else
                F(1, si) = 0;
                F(2, si)= 0;
            end
        end
        
        K_STREAM = K_ATTRACTIVE * 1e9;
        
        for pathi=1:numel(path)-1
            bigcell = path(pathi);
            next = path(pathi + 1);
            if bigcell.inside(px, py)
                for reli=1:numel(relations)
                    rel = relations(reli);
                    if rel.from == bigcell.id && rel.to == next.id
                        F = [F rel.direction.force*K_STREAM];
                    elseif rel.to == bigcell.id && rel.from == next.id
                        F = [F rel.direction.getopposite.force*K_STREAM];
                    end
                end
            end
        end
    end

    function [fx, fy] = compute_force(px, py)
        F = compute_all_forces(px, py);
        fx = sum(F(1,:));
        fy = sum(F(2,:));
    end

    function compute_field()
        [x,y] = meshgrid(-WIDTH/2:0.1:WIDTH/2,-HEIGHT/2:0.1:HEIGHT/2);
        u = zeros(size(x));
        v = zeros(size(y));
        [npx, npy] = size(x);
        for pi=1:npx
            if mod(pi, 10) == 0
                waitbar(pi/npx)
            end
            for pj=1:npy

                px = x(pi, pj);
                py = y(pi, pj);
                
                [fx, fy] = compute_force(px, py);
                
                u(pi, pj) = fx;
                v(pi, pj) = fy;
            end
        end
        
        if myquiver ~= 0
            delete(myquiver)
        end
        if mycontour ~= 0
            delete(mycontour)
        end
        
        figure(1)
     %   [~, mycontour] = contour(x, y, sqrt(u.^2 + v.^2), 3, 'Parent', gca, 'HitTest', 'off', 'Color', 'b');
        myquiver = quiver(x,y,u,v, 'Parent', gca, 'HitTest', 'off', 'Color', 'b');
    end

    function create_ui()
        figure(1)
        for i = 1:numel(ui)
           if ui(i) ~=0
               delete(ui(i))
           end
        end
        % Create slider
        ui(1) = uicontrol('Style', 'slider',...
            'Min',0.0001,'Max',3,'Value',(THRESHOLD),...
            'Position', [50 20 120 20],...
            'Callback', @set_threshold);
        ui(2) = uicontrol('Style', 'slider',...
            'Min',0,'Max',25,'Value', log10(K_ATTRACTIVE),...
            'Position', [200 20 120 20],...
            'Callback', @set_attractive_gain);
        
        ui(3) =uicontrol('Style', 'slider',...
            'Min',0,'Max',10,'Value', log10(-K_REPULSIVE),...
            'Position', [350 20 120 20],...
            'Callback', @set_repulsive_gain);
        
        ui(4) =uicontrol('Style','text',...
            'Position',[50 45 120 20],...
            'String',['THRE' num2str(THRESHOLD, '%10.3e')]);
        ui(5) =uicontrol('Style','text',...
            'Position',[200 45 120 20],...
            'String',['KATT' num2str(K_ATTRACTIVE, '%10.3e')]);
        ui(6) =uicontrol('Style','text',...
            'Position',[350 45 120 20],...
            'String',['KREP' num2str(K_REPULSIVE, '%10.3e')]);
        
        pos = get(gcf, 'Position');
        height = pos(4);
        ui(7) =uicontrol('Style','text',...
            'Position',[50 height - 60 200 40],...
            'String','Left click to set attractive source. Right click to inspect field.');
        
        ui(8) = uicontrol('Style','pushbutton',...
            'Position',[50 height - 80 200 20],...
            'String','Animate', 'Callback', @animate);
    end

    function redraw()
        hwait = waitbar(0, 'Computing sources');
        figure(1)
        hold on
        if myquiver ~= 0
            delete(myquiver)
        end
        if mycontour ~= 0
            delete(mycontour)
        end
                
        waitbar(0, hwait, 'Computing field')
        compute_field()
        create_ui()
        
        if attractiverectangle ~= 0
            delete(attractiverectangle)
        end
        attractiverectangle = rectangle('Position',[attractiveSource.x-0.005 attractiveSource.y-0.005 0.01 0.01],...
            'FaceColor','g',...
            'EdgeColor', 'g',...
            'LineWidth', 0.001, 'Parent', gca, 'HitTest', 'off');

        axis([-WIDTH/2 WIDTH/2 -HEIGHT/2 HEIGHT/2]) 
        axis equal
        close(hwait) 
    end

    function draw()
        hwait = waitbar(0, 'Computing sources');

        figure(1)
        hold on
                
        for i=1:length(sources)
            if mod(i, 10) == 0
                waitbar(i/length(sources))
            end
            source = sources(i);
            if source.type == SourceType.REPULSIVE
                rectangle('Position',[source.x-0.050 source.y-0.050 0.1 0.1],...
                    'FaceColor',[0.6275 0.0196 0.0196],...
                    'EdgeColor', [0.6275 0.0196 0.0196],...
                    'LineWidth', 0.001, 'Parent', gca, 'HitTest', 'off')
                rectangle('Position',[source.x source.y 0.01 0.01],...
                    'FaceColor','r',...
                    'EdgeColor', 'r',...
                    'Curvature', [1 1],...
                    'LineWidth', 0.001, 'Parent', gca, 'HitTest', 'off')
            end
%             rectangle('Position',[source.x-THRESHOLD/2 source.y-THRESHOLD/2 THRESHOLD THRESHOLD],...
%                 'EdgeColor', 'k',...
%                 'Curvature', [1 1],...
%                 'LineWidth', 0.001)
        end
        
        for i=1:numel(bigcells)
            bigcell = bigcells(i);
            rectangle('Position', bigcell.asRectangle(),...
                'EdgeColor', 'k',...
                'LineStyle', '--',...
                'Parent', gca,...
                'HitTest', 'off');
            text(bigcell.getCenterX(), bigcell.getCenterY(), num2str(bigcell.id),...
                'Parent', gca,...
                'HitTest', 'off');
        end
        
        for reli=1:numel(relations)
                this = bigcells(relations(reli).from + 1);
                next = bigcells(relations(reli).to + 1);
                line([this.getCenterX()+rand()*0.05 next.getCenterX()+rand()*0.05],...
                    [this.getCenterY()+rand()*0.05 next.getCenterY()+rand()*0.05],...
                    'Color', 'c', 'Parent', gca, 'HitTest', 'off');
        end


        redraw();
        
        axis([-WIDTH/2 WIDTH/2 -HEIGHT/2 HEIGHT/2]) 
        axis equal
        close(hwait)
        
        create_ui()
    end

    figure(1)
    set(gcf, 'ResizeFcn', @resize_cb);
    set(gca, 'ButtonDownFcn', @button_down_cb);
    hold all
    draw()
end