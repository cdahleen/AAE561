function nc = noCollision3D(A, o, buffer)
nc = 1; 
minx = o(1,1)-buffer;
maxx = o(1,2)+buffer;
miny = o(2,1)-buffer;
maxy = o(2,2)+buffer;
minz = o(3,1)-buffer;
maxz = o(3,2)+buffer;
if A(1)>minx && A(1)<maxx
    if A(2)>miny && A(2)<maxy
        if A(3)>minz && A(3)<maxz
            nc = 0;
        end
    end
end
end