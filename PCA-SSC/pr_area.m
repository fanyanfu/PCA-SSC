ans=[];
for i=1:size(Precisions,1)
    if(isnan(Precisions(i)))
        continue;
    end
    ans=[ans;Recalls(i),Precisions(i)];
end
ans=[0,0;ans;1,0];
area=polyarea(ans(:,1),ans(:,2));