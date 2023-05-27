function nodes = RRT_star_planner(ax, q_start, q_goal, numNodes, EPS, CollisionArray, env4)


    nodes(1) = q_start;
    reached = false;
    
    for i = 1:1:numNodes
        q_rand = [(randi([-600,500],1)) (randi([-200,300],1)) (randi([100,800],1))];
        plot3(ax,q_rand(1), q_rand(2), q_rand(3), '.', 'Color','r')
        title('Transmission')
        for j = 1:1:length(nodes)
            if euclidean_distance(nodes(j).coord, q_goal.coord)<100 
                disp("Goal!")
                reached = true;
                break
            end
        end
        if reached
            break
        end

        ndist = [];
        for j = 1:1:length(nodes)
            n = nodes(j);
            tmp = euclidean_distance(n.coord, q_rand);
            ndist = [ndist tmp];
        end
        [val, idx] = min(ndist);
        q_near = nodes(idx);

        q_new.coord = move(q_rand, q_near.coord, val, EPS);

        out =check_free_path(q_new.coord,CollisionArray,env4);
        colliding = out(1);
        ang = out(2);
        if colliding
            continue
        end

        plot3(ax,[q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'g', 'LineWidth', 2);
        drawnow

        q_new.cost = euclidean_distance(q_new.coord, q_near.coord) + q_near.cost;

        q_nearest = [];
        r = 50;
        neighbor_count = 1;

        for j = 1:1:length(nodes)
            if (euclidean_distance(nodes(j).coord, q_new.coord)) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end

        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        q_new.parent = idx;
        q_new.rotation = ang;

        % Iterate through all nearest neighbors to find alternate lower cost paths
        nodes = [nodes q_new];
    end

end