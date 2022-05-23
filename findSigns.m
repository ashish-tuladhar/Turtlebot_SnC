function findSigns(tbot,points)
    
%     ipaddress = '192.168.17.149'; % IP address of your robot
%     tbot = turtlebot(ipaddress,11311);
%     tbot.Velocity.TopicName = '/cmd_vel';
        if points < 5
            angularVel = 0.5;
        end
        if points >= 5 %&& points < 10
            angularVel = 0.3;
        end
        
        setVelocity(tbot,0,angularVel, 'Time', 2);
end