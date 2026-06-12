function f_vec = rubine_fun(x, y, t)
delX = diff(x);
delY = diff(y);
delT = diff(t);
maxX = max(x); minX = min(x);
maxY = max(y); minY = min(y);

dx_init = x(3) - x(1);
dy_init = y(3) - y(1);
denom_init = sqrt(dx_init^2 + dy_init^2);
if denom_init == 0
    f1 = 0; f2 = 0;
else
    f1 = dx_init / denom_init;
    f2 = dy_init / denom_init;
end

f3 = sqrt((maxX - minX)^2 + (maxY - minY)^2);
f4 = atan2(maxY - minY, maxX - minX);

f5 = sqrt((x(end) - x(1))^2 + (y(end) - y(1))^2);
if f5 == 0
    f6 = 0; f7 = 0;
else
    f6 = (x(end) - x(1)) / f5;
    f7 = (y(end) - y(1)) / f5;
end

f8 = sum(sqrt(delX.^2 + delY.^2));

% Turning angles: loop from index 2 to end of delX
theta_p = zeros(1, length(delX)-1);
for i = 2:length(delX)
    num = delX(i)*delY(i-1) - delX(i-1)*delY(i);
    den = delX(i)*delX(i-1) + delY(i)*delY(i-1);
    theta_p(i-1) = atan2(num, den);  % atan2 avoids NaN at den=0
end

f9  = sum(theta_p);
f10 = sum(abs(theta_p));
f11 = sum(theta_p.^2);

valid = delT > 0;
if any(valid)
    f12 = max((delX(valid).^2 + delY(valid).^2) ./ delT(valid).^2);
else
    f12 = 0;
end

f13 = t(end) - t(1);

f_vec = [f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 f11 f12 f13];
end