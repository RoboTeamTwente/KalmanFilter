function G = Gjacobian(u)
c = cos(u(3)); s = sin(u(3));
G = [0,0,0;0,0,0;c, -s, -u(1)*s-u(2)*c; s, c, u(1)*c-u(2)*s];
