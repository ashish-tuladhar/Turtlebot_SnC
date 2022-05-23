function moveBot(tbot, x, y)
    [ld, move_angle] = getTargetInfo(tbot, x, y);
    moved_angle = 0; angular_vel = 0.1;
    if move_angle > 0                       % for smooth movement, set angular velocity at 0.1
        while(moved_angle < move_angle)
                setVelocity(tbot, 0, angular_vel, 'Time', 1);
                moved_angle = moved_angle + 0.1;
        end
    else
        while(moved_angle > move_angle)
                setVelocity(tbot, 0, -angular_vel, 'Time', 1);
                moved_angle = moved_angle - 0.1;
        end
    end

    [ld, move_angle] = getTargetInfo(tbot, x,y);

    if move_angle > 0.001 || move_angle < -0.001            
        setVelocity(tbot, 0, move_angle);
    end
    
    % consider minor changes during turning to target point
    while ld > 0.1
        [ld, move_angle] = getTargetInfo(tbot, x,y);
        setVelocity(tbot, 0.1 ,move_angle,'Time', 1);
    end
end