function [ h ] = estimateHeight( VP, estHeight, p1, p2, p3, p4 ) 
    vp1 = VP(:,1)'; 
    vp2 = VP(:,2)'; 
    
    a = (vp2(2) - vp1(2)) / (vp2(1) - vp1(1));
    b = vp2(2) - (a * vp2(1));
    
    a0 = (p4(2) - p2(2))/(p4(1) - p2(1));
    b0 = p4(2) - (a0 * p2(1));
   
    sy = (b - b0)/(a0 - a); 
    eV = [sy, sy * a + b];
    
    a = (p1(2) - eV(2)) / (p1(1) - eV(1));
    b = (p1(2) - a * p1(1));
    t = [p3(1), p3(1) * a + b];
    eH = abs(estHeight / (p3(2) - p4(2)));
    icr = abs(t(2) - p4(2))  / abs(p3(2) - p4(2)) * abs(p1(2) / t(2));
    h = icr * estHeight;
end

