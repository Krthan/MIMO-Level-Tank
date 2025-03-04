function dxdt = fourtank_non_lin(t,x, params)
    
    A1 = params(1); A2 = params(2); A3 = params(3); A4 = params(4);

    a1 = params(5); a2 = params(6); a3 = params(7); a4 = params(8);

    kc = params(9); g = params(10);

    gamma1 = params(11); gamma2 = params(12);

    k1 = params(13); k2 = params(14); v1 = params(15); v2 = params(16);

    h1 = x(1); h2 = x(2); h3 = x(3); h4 = x(4);
    
    dh1 = (-a1/A1) * sqrt(2*g*h1) + (a3/A1) * sqrt(2*g*h3) + (gamma1 * k1 * v1)/A1;

    dh2 = (-a2/A2) * sqrt(2*g*h2) + (a4/A2) * sqrt(2*g*h4) + (gamma2 * k2 * v2)/A2;

    dh3 = (-a3/A3) * sqrt(2*g*h3) + ((1 - gamma2) * k2 * v2)/A3;

    dh4 = (-a4/A4) * sqrt(2*g*h4) + ((1 - gamma1) * k1 * v1)/A4;

    dxdt = [dh1; dh2; dh3; dh4];

end