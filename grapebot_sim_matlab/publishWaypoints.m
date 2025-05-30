function publishWaypoints(pts)

% Create ROS publisher
persistent pub lastStamp
if isempty(pub) || ~isvalid(pub)
    pub = rospublisher('/rviz/matlab/waypoints','visualization_msgs/Marker');
    lastStamp = rostime('now', 'DataFormat', 'struct');
end

% Create Marker message
persistent msg
if isempty(msg) || ~isvalid(msg)
    msg = rosmessage('visualization_msgs/Marker');
    msg.Header.FrameId = 'odom';
    msg.Header.Stamp   = rostime('now');
    
    msg.Type   = msg.POINTS;
    msg.Action = msg.ADD;
    
    msg.Ns = 'matlab';
    msg.Lifetime = rosduration(1);
    
    msg.Scale.X = 0.2;
    msg.Scale.Y = 0.2;
    msg.Scale.Z = 0.2;
    
    msg.Color.R = 0.0;
    msg.Color.G = 1.0;
    msg.Color.B = 0.0;
    msg.Color.A = 1.0;
    
    N = size(pts,2);
    
    for i=1:N
        p = rosmessage('geometry_msgs/Point');
        p.X = pts(1,i);
        p.Y = pts(2,i);
        p.Z = 0;
        msg.Points(i) = p;
    end
end

nowStamp = rostime('now', 'DataFormat', 'struct');
elapsed = double(nowStamp.Sec) + double(nowStamp.Nsec)*1e-9 - ...
          (double(lastStamp.Sec) + double(lastStamp.Nsec)*1e-9);

if elapsed >= 0.3
    msg.Header.Stamp = rostime('now');
    send(pub, msg);
    lastStamp = nowStamp;
end

end