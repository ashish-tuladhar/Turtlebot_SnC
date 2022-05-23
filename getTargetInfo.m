function [ld turnAngle] = getTargetInfo(tbot, x, y)
    origin = getOdometry(tbot);

    dx = x - origin.Position(1);
    dy = y - origin.Position(2);
    
    ld = sqrt((dx.^2 + dy.^2));
    target_angle = 0;

    if dx < 0.001 && dx > -0.001
        target_angle = asin(dy/ld);
    else
        target_angle = atan2(dy, dx);
    end
    
    turnAngle = target_angle - origin.Orientation(1);

    if turnAngle > 2*pi
        turnAngle = turnAngle - 2*pi;
    elseif turnAngle < 0
        turnAngle = turnAngle + 2*pi;
    end

    if turnAngle > pi
        turnAngle = -((2 * pi) - turnAngle);
    end
end