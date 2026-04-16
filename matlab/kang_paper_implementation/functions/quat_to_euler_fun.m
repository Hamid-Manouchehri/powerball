function euler = quat_to_euler_fun(q)

    % q = [w x y z]
    
    q = q / norm(q);  % Normalization
    
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    
    roll  = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));
    pitch = asin(2*(w*y - z*x));
    yaw   = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2));
    
    euler = [roll pitch yaw];

end