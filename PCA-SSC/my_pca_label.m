function [newData,label] = my_pca_label(data,label)
meanValue=mean(data);
normData=data-meanValue;
covMat=cov(normData);
[eigVec,eigVal]=eig(covMat);
[eigVal_sort,index]=sort(diag(eigVal),1,'descend');
eigVec=eigVec(:,index');
newData=normData*eigVec;