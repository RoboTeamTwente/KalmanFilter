function g = gfunc(u)
c = cos(u(3)); s = sin(u(3));
g = [0; 0; c*u(1)-s*u(2); s*u(1)+c*u(2)];
