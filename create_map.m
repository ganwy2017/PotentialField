function create_map()
    WIDTH = 2000;
    HEIGHT = 3000;
    ROB_RADIUS = 130;
    THRESHOLD = 700;

    map = zeros(HEIGHT, WIDTH);

    map(HEIGHT/2+50, WIDTH/2-150) = 1; 

    [x,y] = meshgrid(0:50:WIDTH,0:50:HEIGHT);
    u = zeros(size(x));
    v = zeros(size(y));

    [npx, npy] = size(x);

    sources = [];
    sources = [sources Source(WIDTH/2 + (-0.150 * 1000), HEIGHT / 2 + ( 0.050 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 * 1000), HEIGHT / 2 + ( 0.150 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 * 1000), HEIGHT / 2 + ( 0.250 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 * 1000), HEIGHT / 2 + ( 0.350 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.050 * 1000), HEIGHT / 2 + ( 0.350 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.050 * 1000), HEIGHT / 2 + ( 0.350 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.150 * 1000), HEIGHT / 2 + ( 0.350 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 * 1000), HEIGHT / 2 + ( -0.050 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 * 1000), HEIGHT / 2 + ( -0.150 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 * 1000), HEIGHT / 2 + ( -0.250 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.150 * 1000), HEIGHT / 2 + ( -0.350 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.050 * 1000), HEIGHT / 2 + ( -0.350 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.050 * 1000), HEIGHT / 2 + ( -0.350 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.150 * 1000), HEIGHT / 2 + ( -0.350 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.250 * 1000), HEIGHT / 2 + ( 0.050 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.350 * 1000), HEIGHT / 2 + ( 0.050 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.450 * 1000), HEIGHT / 2 + ( 0.050 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.450 * 1000), HEIGHT / 2 + ( -0.050 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.350 * 1000), HEIGHT / 2 + ( -0.050 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.250 * 1000), HEIGHT / 2 + ( -0.050 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.950 * 1000), HEIGHT / 2 + ( -0.850 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.850 * 1000), HEIGHT / 2 + ( -0.850 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.750 * 1000), HEIGHT / 2 + ( -0.850 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.650 * 1000), HEIGHT / 2 + ( -0.850 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.550 * 1000), HEIGHT / 2 + ( -0.850 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.950 * 1000), HEIGHT / 2 + ( 0.850 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.850 * 1000), HEIGHT / 2 + ( 0.850 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.750 * 1000), HEIGHT / 2 + ( 0.850 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.650 * 1000), HEIGHT / 2 + ( 0.850 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (-0.550 * 1000), HEIGHT / 2 + ( 0.850 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 * 1000), HEIGHT / 2 + ( -1.050 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 * 1000), HEIGHT / 2 + ( -1.150 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 * 1000), HEIGHT / 2 + ( -1.250 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 * 1000), HEIGHT / 2 + ( -1.350 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 * 1000), HEIGHT / 2 + ( -1.450 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 * 1000), HEIGHT / 2 + ( 1.050 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 * 1000), HEIGHT / 2 + ( 1.150 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 * 1000), HEIGHT / 2 + ( 1.250 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 * 1000), HEIGHT / 2 + ( 1.350 * 1000), SourceType.REPULSIVE)];
    sources = [sources Source(WIDTH/2 + (0.450 * 1000), HEIGHT / 2 + ( 1.450 * 1000), SourceType.REPULSIVE)];

    function compute_field(source,event)
        for pi=1:npx
            for pj=1:npy
                fx = 0;
                fy = 0;

                px = x(pi, pj);
                py = y(pi, pj);

                for si = 1:length(sources)
                    s = sources(si);
                    dist_x = (s.x - px);
                    dist_y = (s.y - py);
                    dist_min = sqrt(dist_x * dist_x + dist_y * dist_y) - ROB_RADIUS;
                    if s.type == SourceType.REPULSIVE && dist_min < THRESHOLD
                        myfx = s.type.K * (1 / dist_min - 1 / THRESHOLD) * (1/(dist_min*dist_min)) * (dist_x/dist_min);
                        myfy = s.type.K * (1 / dist_min - 1 / THRESHOLD) * (1/(dist_min*dist_min)) * (dist_y/dist_min);
                    else
                        myfx = 0;
                        myfy = 0;
                    end

                    fx = fx + myfx;
                    fy = fy + myfy;
                end
                u(pi, pj) = fx;
                v(pi, pj) = fy;
            end
        end
        quiver(x,y,u,v)
    end

    figure
    hold on

    % Create slider
    sld = uicontrol('Style', 'slider',...
        'Min',1,'Max',3000,'Value',700,...
        'Position', [400 20 120 20],...
        'Callback', @compute_field); 

    for i=1:length(sources)
        source = sources(i);
        rectangle('Position',[source.x-50 source.y-50 100 100],...
            'FaceColor',[0.6275 0.0196 0.0196],...
            'EdgeColor', [0.6275 0.0196 0.0196],...
            'LineWidth', 0.001)
        rectangle('Position',[source.x source.y 10 10],...
            'FaceColor','r',...
            'EdgeColor', 'r',...
            'Curvature', [1 1],...
            'LineWidth', 0.001)
    end

    quiver(x,y,u,v)
    axis square
    axis([0 2000 0 3000])

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