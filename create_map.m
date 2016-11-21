function create_map()
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

    sources = [];
    sources = [sources Source(WIDTH/2 + (-0.150 ), HEIGHT / 2 + ( 0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 ), HEIGHT / 2 + ( 0.150 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 ), HEIGHT / 2 + ( 0.250 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 ), HEIGHT / 2 + ( 0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.050 ), HEIGHT / 2 + ( 0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.050 ), HEIGHT / 2 + ( 0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.150 ), HEIGHT / 2 + ( 0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 ), HEIGHT / 2 + ( -0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 ), HEIGHT / 2 + ( -0.150 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 ), HEIGHT / 2 + ( -0.250 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 ), HEIGHT / 2 + ( -0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.050 ), HEIGHT / 2 + ( -0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.050 ), HEIGHT / 2 + ( -0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.150 ), HEIGHT / 2 + ( -0.350 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.250 ), HEIGHT / 2 + ( 0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.350 ), HEIGHT / 2 + ( 0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.450 ), HEIGHT / 2 + ( 0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.450 ), HEIGHT / 2 + ( -0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.350 ), HEIGHT / 2 + ( -0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.250 ), HEIGHT / 2 + ( -0.050 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.950 ), HEIGHT / 2 + ( -0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.850 ), HEIGHT / 2 + ( -0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.750 ), HEIGHT / 2 + ( -0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.650 ), HEIGHT / 2 + ( -0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.550 ), HEIGHT / 2 + ( -0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.950 ), HEIGHT / 2 + ( 0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.850 ), HEIGHT / 2 + ( 0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.750 ), HEIGHT / 2 + ( 0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.650 ), HEIGHT / 2 + ( 0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.550 ), HEIGHT / 2 + ( 0.850 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 ), HEIGHT / 2 + ( -1.050 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 ), HEIGHT / 2 + ( -1.150 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 ), HEIGHT / 2 + ( -1.250 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 ), HEIGHT / 2 + ( -1.350 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 ), HEIGHT / 2 + ( -1.450 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 ), HEIGHT / 2 + ( 1.050 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 ), HEIGHT / 2 + ( 1.150 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 ), HEIGHT / 2 + ( 1.250 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 ), HEIGHT / 2 + ( 1.350 ), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 ), HEIGHT / 2 + ( 1.450 ), SourceType.REPULSIVE)];
    
    for i=0:32
        sources = [sources Source(WIDTH/2 + (-1.050), HEIGHT / 2 + ( -1.550+i*0.100 ), SourceType.REPULSIVE)];
        sources = [sources Source(WIDTH/2 + (1.050), HEIGHT / 2 + ( -1.550+i*0.100 ), SourceType.REPULSIVE)];
    end
    
    for i=0:22
        sources = [sources Source(WIDTH/2 + (-1.050+i*0.100), HEIGHT / 2 + ( -1.550 ), SourceType.REPULSIVE)];
        sources = [sources Source(WIDTH/2 + (-1.050+i*0.100), HEIGHT / 2 + ( 1.550 ), SourceType.REPULSIVE)];
    end
    
    attractiveSource = Source(WIDTH/2 + ( -1 ), HEIGHT / 2 + ( 0 ), SourceType.ATTRACTIVE);
    sources = [sources attractiveSource];

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
        create_ui()
    end
    
    function button_down_cb(source, event)
        if event.Button == 1
            attractiveSource.x = event.IntersectionPoint(1);
            attractiveSource.y = event.IntersectionPoint(2);
            sources = [ sources(1:end-1) attractiveSource];
            redraw();
        elseif event.Button == 3
            figure(1)
            
            if targetpoint ~= 0
                delete(targetpoint)
            end
            targetpoint = rectangle('Position',[event.IntersectionPoint(1)-0.01, event.IntersectionPoint(2)-0.01 0.02 0.02],...
                'FaceColor','m',...
                'EdgeColor', 'm',...
                'LineWidth', 0.001, 'Parent', gca, 'HitTest', 'off');

            
            figure(2)
            
            F = compute_all_forces(event.IntersectionPoint(1), event.IntersectionPoint(2));
            
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
        end
        
    end

    function F = compute_all_forces(px, py)
        F = zeros(2, length(sources));
        for si = 1:length(sources)
            s = sources(si);
            dist_x = (s.x - px);
            dist_y = (s.y - py);
            dist_min = max(0, sqrt(dist_x * dist_x + dist_y * dist_y)) - ROB_RADIUS;
            if s.type == SourceType.REPULSIVE && dist_min < THRESHOLD && dist_min > 0
                F(1, si) = K_REPULSIVE * (1 / dist_min - 1 / THRESHOLD) * (1/(dist_min*dist_min)) * (dist_x/dist_min);
                F(2, si) = K_REPULSIVE * (1 / dist_min - 1 / THRESHOLD) * (1/(dist_min*dist_min)) * (dist_y/dist_min);
            elseif s.type == SourceType.ATTRACTIVE
                F(1, si) = K_ATTRACTIVE * dist_x;
                F(2, si) = K_ATTRACTIVE * dist_y;
            else
                F(1, si) = 0;
                F(2, si)= 0;
            end
        end 
    end

    function [fx, fy] = compute_force(px, py)
        F = compute_all_forces(px, py);
        fx = sum(F(1,:));
        fy = sum(F(2,:));
    end

    function compute_field()
        [x,y] = meshgrid(0:0.1:WIDTH,0:0.1:HEIGHT);
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
        
     %   [~, mycontour] = contour(x, y, sqrt(u.^2 + v.^2), 3, 'Parent', gca, 'HitTest', 'off', 'Color', 'b');
        myquiver = quiver(x,y,u,v, 'Parent', gca, 'HitTest', 'off', 'Color', 'b');
    end

    function create_ui()
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

        
        axis square
        axis([0 2 0 3]) 
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
        redraw()
        
        axis square
        axis([0 2 0 3]) 
        close(hwait) 
        
        
        create_ui()
    end

    figure(1)
    set(gcf, 'ResizeFcn', @resize_cb);
    set(gca, 'ButtonDownFcn', @button_down_cb);
    hold all
    draw()
    
    % create_cell(0, -0.150, 0.050, REPULSIVE_CELL);
    % create_cell(1, -0.150, 0.150, REPULSIVE_CELL);
    % create_cell(2, -0.150, 0.250, REPULSIVE_CELL);
    % create_cell(3, -0.150, 0.350, REPULSIVE_CELL);
    % create_cell(4, -0.050, 0.350, REPULSIVE_CELL);
    % create_cell(5, 0.050, 0.350, REPULSIVE_CELL);
    % create_cell(6, 0.150, 0.350, REPULSIVE_CELL);
    % create_cell(7, -0.150, -0.050, REPULSIVE_CELL);
    % create_cell(8, -0.150, -0.150, REPULSIVE_CELL);
    % create_cell(9, -0.150, -0.250, REPULSIVE_CELL);
    % 
    % create_cell(10, -0.150, -0.350, REPULSIVE_CELL);
    % create_cell(11, -0.050, -0.350, REPULSIVE_CELL);
    % create_cell(12, 0.050, -0.350, REPULSIVE_CELL);
    % create_cell(13, 0.150, -0.350, REPULSIVE_CELL);
    % create_cell(14, -0.250, 0.050, REPULSIVE_CELL);
    % create_cell(15, -0.350, 0.050, REPULSIVE_CELL);
    % create_cell(16, -0.450, 0.050, REPULSIVE_CELL);
    % create_cell(17, -0.450, -0.050, REPULSIVE_CELL);
    % create_cell(18, -0.350, -0.050, REPULSIVE_CELL);
    % create_cell(19, -0.250, -0.050, REPULSIVE_CELL);
    % 
    % create_cell(20, -0.950, -0.850, REPULSIVE_CELL);
    % create_cell(21, -0.850, -0.850, REPULSIVE_CELL);
    % create_cell(22, -0.750, -0.850, REPULSIVE_CELL);
    % create_cell(23, -0.650, -0.850, REPULSIVE_CELL);
    % create_cell(24, -0.550, -0.850, REPULSIVE_CELL);
    % create_cell(25, -0.950, 0.850, REPULSIVE_CELL);
    % create_cell(26, -0.850, 0.850, REPULSIVE_CELL);
    % create_cell(27, -0.750, 0.850, REPULSIVE_CELL);
    % create_cell(28, -0.650, 0.850, REPULSIVE_CELL);
    % create_cell(29, -0.550, 0.850, REPULSIVE_CELL);
    % 
    % create_cell(30, 0.450, -1.050, REPULSIVE_CELL);
    % create_cell(31, 0.450, -1.150, REPULSIVE_CELL);
    % create_cell(32, 0.450, -1.250, REPULSIVE_CELL);
    % create_cell(33, 0.450, -1.350, REPULSIVE_CELL);
    % create_cell(34, 0.450, -1.450, REPULSIVE_CELL);
    % create_cell(35, 0.450, 1.050, REPULSIVE_CELL);
    % create_cell(36, 0.450, 1.150, REPULSIVE_CELL);
    % create_cell(37, 0.450, 1.250, REPULSIVE_CELL);
    % create_cell(38, 0.450, 1.350, REPULSIVE_CELL);
    % create_cell(39, 0.450, 1.450, REPULSIVE_CELL);
    % 
    % create_cell(PATH_PLANNING_TARGET, 0, 0, VOID_CELL);
    % 
    % for(int i=0;i<32;i++){
    %     create_cell(i+41, -1.050, -1.550+i*0.100, REPULSIVE_CELL);
    %     create_cell(i+41+32, 1.050, -1.550+i*0.100, REPULSIVE_CELL);
    % }
    % 
    % for(int i=0;i<22;i++){
    %     create_cell(i+41+32+32, -1.050+i*0.100, -1.550, REPULSIVE_CELL);
    %     create_cell(i+41+32+32+22, -1.050+i*0.100, 1.550, REPULSIVE_CELL);
    % }
    % 
    % create_cell(PATH_PLANNING_OPP1, 0, 0, VOID_CELL);
    % create_cell(PATH_PLANNING_OPP2, 0, 0, VOID_CELL);
end