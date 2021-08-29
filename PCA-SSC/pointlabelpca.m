function [data_new,label]=pointlabelpca(data_old,label)
%%%输入的是点云数据 data 和标签 label 返回值为变换后的data和label
%%步骤为先平移后旋转
%-----------------------
%改 输入和输出对象均为pointcloud对象
data_old=data_old.Location;
meanValue=mean(data_old);
normData=data_old-meanValue;
covMat=cov(normData);
[eigVec,eigVal]=eig(covMat);

%按对角元素重新抽取排序变换矩阵
[eigVal_sort,index]=sort(diag(eigVal),1,'descend');
eigVec=eigVec(:,index');

data_new=normData*eigVec;
data_new=pointCloud(data_new);

end