function animate_simple(t,z,p, speed)
    hold on
    axis([-0.5 0.8 -0.5 0.8])
    axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',5);
    h1_leg    = plot([0],[0],'-o',...
                'LineWidth',3,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',6); 
    h2_leg    = plot([0],[0],'-o',...
                'LineWidth',3,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',6); 

    tic                                             % start counter
    while toc < t(end)/speed                        % while there's real time left
        tsim = toc*speed;                           % determine the simulation time to draw
        zint = interp1(t',z',tsim', 'linear')';     % interpolate to get coordinates at that time
        draw_lines(zint,p,h1_leg,h2_leg);
    end
    draw_lines(z(:,end),p,h1_leg,h2_leg);
    %draw_lines(z(1,2),p,h_leg);

end

function draw_lines(z,p, h1_leg, h2_leg)
    keypoints = keypoints_jumping_leg(z,p);
    h1_leg.XData = keypoints(1,1:2);
    h1_leg.YData = keypoints(2,1:2);
    
    h2_leg.XData = keypoints(1,2:3);
    h2_leg.YData = keypoints(2,2:3);
    
    drawnow
end