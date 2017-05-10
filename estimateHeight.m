function [ h ] = estimateHeight( VP, estHeight, p1, p2, p3, p4 )
%     p3 = [538, 107];
%     p4 = [538, 300];
   
    vp1 = [VP(1,1), VP(1,2)]; 
    vp2 = [VP(2,1), VP(2,2)];
    
    a = (vp1(2) - vp2(2)) / (vp1(1) - vp2(1));
    b = vp1(2) - (a * vp1(1));
    
    a0 = (p4(2) - p2(2))/(p4(1) - p2(1));
    b0 = p4(2) - (a0 * p2(1));
    eX = (vp1(2) - b0) / a0;
    
    eV = [eX, eX * a + b];
    
    a = (p1(2) - eV(2)) / (p1(1) - eV(1));
    b = (p1(2) - a * p1(1));
    t = [p3(1), p3(1) * a + b];
    eH = abs(estHeight / (p3(2) - p4(2)));
    icr = abs(t(2) - p2(2)) / abs(t(2) - p4(2));
    h = eH * abs(p2(2) - p1(2));
end

