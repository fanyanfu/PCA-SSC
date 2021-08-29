function [sc]=sc_dealwith(sc)
for i=1:size(sc,1)
    for j=1:size(sc,2)
        if((sc(i,j)<=1|sc(i,j)>=99))
            sc(i,j)=0;
        end
    end
end
end