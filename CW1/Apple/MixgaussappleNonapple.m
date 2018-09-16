function MixgaussappleNonapple

%==========================================================================
%get pixels data
Iapples = cell(3,1);
Iapples{1} = 'apples/Apples_by_kightp_Pat_Knight_flickr.jpg';
Iapples{2} = 'apples/ApplesAndPears_by_srqpix_ClydeRobinson.jpg';
Iapples{3} = 'apples/bobbing-for-apples.jpg';

IapplesMasks = cell(4,1);
IapplesMasks{1} = 'apples/Apples_by_kightp_Pat_Knight_flickr.jpf';
IapplesMasks{2} = 'apples/ApplesAndPears_by_srqpix_ClydeRobinson2.jpg';
IapplesMasks{3} = 'apples/bobbing-for-apples2.jpg';
IapplesMasks{4} = 'testApples/Bbr98ad4z0A-ctgXo3gdwu8-original.png';

ItestApples = cell(3,1);
ItestApples{1} = 'testApples/Apples_by_MSR_MikeRyan_flickr.jpg';
ItestApples{2} = 'testApples/audioworm-QKUJj2wmxuI-original.jpg';
ItestApples{3} = 'testApples/Bbr98ad4z0A-ctgXo3gdwu8-original.jpg';
%--------------------------------------------------------------------------
%testapple2
ItestApples2 = cell(3,1);
ItestApples2{1} = 'testApples2/test2.jpg';
ItestApples2{2} = 'testApples2/test4.jpg';
ItestApples2{3} = 'testApples2/test5.jpeg';

IapplesMasks2 = cell(3,1);
IapplesMasks2{1} = 'testApples2/test2-1.jpg';
IapplesMasks2{2} = 'testApples2/test4-1.jpg';
IapplesMasks2{3} = 'testApples2/test5-1.jpeg';

%==========================================================================
appleData=zeros(0);
nonappleData=zeros(0);
maskData=zeros(0);
curIData=zeros(0);
for iImage=1:3
    curI = double(imread(  Iapples{iImage}   )) / 255;
    curImask = imread(  IapplesMasks{iImage}   );
    curImask = curImask(:,:,2) > 128;  % Picked green-channel arbitrarily.    
    [curIY curIX curIZ] = size(curI);
    curIdata=(reshape(curI,curIY*curIX,curIZ))';%get 3*ndata
    [curImaskY curImaskX curImaskZ] = size(curImask);
    curImaskdata=reshape(curImask,curImaskX*curImaskY,curImaskZ);
    AppleIndex=find(curImaskdata);%get apple and nonapple index(1D)
    nonAppleIndex=find(curImaskdata==0);
    Appledata=curIdata(:,AppleIndex);%get apple and nonapple pixels:3*ndata
    nonAppledata=curIdata(:,nonAppleIndex);
    appleData=[appleData Appledata];
    nonappleData=[nonappleData nonAppledata];
    %maskData=[maskData; curImaskdata];
    %curIData=[curIData curIdata];
end;

priorApple=size(appleData,2)/(size(appleData,2)+size(nonappleData,2));
priorNonApple=1-priorApple;
%==========================================================================
%test which K value is best
%curTest3 = double(imread(  ItestApples{3}   )) / 255;
%[curTestY curTestX curTestZ] = size(curTest3);
%testapple=(reshape(curTest3,curTestY*curTestX,curTestZ))';%get the 3*ndata test data

%iImage4 = 4;
%curImask4 = imread(  IapplesMasks{iImage4}   );
%curImask4 = curImask4(:,:,2) > 128;  % Picked green-channel arbitrarily.
%[curImaskY4 curImaskX4 curImaskZ4] = size(curImask4);
%curImaskdata4=reshape(curImask4,curImaskX4*curImaskY4,curImaskZ4);%ndata*1
%auc=zeros(3,3);
%for n=3:5
 %   for non=3:5
  %      mixGaussApple = fitMixGauss(appleData,n);
   %     mixGaussnonApple = fitMixGauss(nonappleData,non);
    %    likeApple = getMixGaussLike(testapple,mixGaussApple);
     %   likeNonApple = getMixGaussLike(testapple,mixGaussnonApple);
      %  posteriorApple = likeApple*priorApple./(likeApple*priorApple+likeNonApple*priorNonApple);
       % auc(n,non)=calcROC(curImaskdata4,posteriorApple);
    %end;
%end;

%[inn,innon]=find(auc==max(max(auc)))
%maxauc=max(max(auc));
%nGauss=inn
%nonnGauss=innon
%fprintf('Maximum auc  %d %d: %4.3f\n',nGauss,nonnGauss,maxauc);

%==========================================================================
%==========================================================================
nGauss = 3;
nonnGauss = 3;
mixGaussApple = fitMixGauss(appleData,nGauss);
mixGaussnonApple = fitMixGauss(nonappleData,nonnGauss);

for im=1:3
    curTest = double(imread(  ItestApples{im}   )) / 255;
    [curTestY curTestX curTestZ] = size(curTest);
    testapple=(reshape(curTest,curTestY*curTestX,curTestZ))';%get the 3*ndata test data
    likeApple = getMixGaussLike(testapple,mixGaussApple);
    likeNonApple = getMixGaussLike(testapple,mixGaussnonApple);
    posteriorApple = likeApple*priorApple./(likeApple*priorApple+likeNonApple*priorNonApple);
    plotapple(curTest, posteriorApple)
end;
%==========================================================================
%==========================================================================
%mask image for test image 3
iImage4 = 4;
curImask4 = imread(  IapplesMasks{iImage4}   );
curImask4 = curImask4(:,:,2) > 128;  % Picked green-channel arbitrarily.
[curImaskY4 curImaskX4 curImaskZ4] = size(curImask4);
curImaskdata4=reshape(curImask4,curImaskX4*curImaskY4,curImaskZ4);%ndata*1
%--------------------------------------------------------------------------
%get roc curve
%test3 roc curve
auc=calcROC(curImaskdata4,posteriorApple)
%==========================================================================
%==========================================================================
%testapple2
for i=1:3
    curItests = double(imread(  ItestApples2{i}   )) / 255;
    curImasks = imread(  IapplesMasks2{i}   );
    curImasks = curImasks(:,:,2) > 128; 
    [curItestsY curItestsX curItestsZ] = size(curItests);
    curItestsdata=(reshape(curItests,curItestsY*curItestsX,curItestsZ))';%get 3*ndata
    [curImasksY curImasksX curImasksZ] = size(curImasks);
    curImasksdata=reshape(curImasks,curImasksX*curImasksY,curImasksZ);%ndata*1
    liketestApple = getMixGaussLike(curItestsdata,mixGaussApple);
    liketestNonApple = getMixGaussLike(curItestsdata,mixGaussnonApple);
    posteriortestApple = liketestApple*priorApple./(liketestApple*priorApple+liketestNonApple*priorNonApple);
    auc=calcROC(curImasksdata,posteriortestApple);
    fprintf('AUC value of image %d is %4.4f\n ',i,auc);
    plotapple(curItests, posteriortestApple)

end;
%==========================================================================
%==========================================================================

function mixGaussEst = fitMixGauss(data,k);
        
[nDim nData] = size(data);

postHidden = zeros(k, nData);

%in the E-M algorithm, we calculate a complete posterior distribution over
%the (nData) hidden variables in the E-Step.  In the M-Step, we
%update the parameters of the Gaussians (mean, cov, w).  

%we will initialize the values to random values
mixGaussEst.d = nDim;
mixGaussEst.k = k;
mixGaussEst.weight = (1/k)*ones(1,k);
%mixGaussEst.mean = 2*randn(nDim,k);
mixGaussEst.mean = randn(nDim,k);
%mixGaussEst.mean = rand(nDim,k);

for (cGauss =1:k)
    mixGaussEst.cov(:,:,cGauss) = (1.5+1*rand(1))*eye(nDim,nDim);
end;

%calculate current likelihood
logLike = getMixGaussLogLike(data,mixGaussEst);
fprintf('Log Likelihood Iter 0 : %4.3f\n',logLike);

cIter=1;
continuenext=1;
while continuenext==1
    
    %======================================================================
    prx=zeros(nData,1);
    for (i=1:mixGaussEst.k)
        prx=prx+mixGaussEst.weight(i)*calcGaussianProb(data,mixGaussEst.mean(:,i),mixGaussEst.cov(:,:,i));
    end;
    for (i=1:mixGaussEst.k)
        postHidden(i,:)= mixGaussEst.weight(i)*calcGaussianProb(data,mixGaussEst.mean(:,i),mixGaussEst.cov(:,:,i))./prx;
    end;
    
    %======================================================================
   
   %Maximization Step
   
   %for each constituent Gaussian
   for (cGauss = 1:k) 
        mixGaussEst.weight(cGauss) = sum(postHidden(cGauss,:))/sum(postHidden(:)); 
        mixGaussEst.mean(:,cGauss)=sum(postHidden(cGauss,:).*data(:,:),2)/sum(postHidden(cGauss,:));
        mixGaussEst.cov(:,:,cGauss)=postHidden(cGauss,:).*(data-mixGaussEst.mean(:,cGauss))*(data-mixGaussEst.mean(:,cGauss))'/sum(postHidden(cGauss,:));
        % mixGaussEst.weight(cGauss)
     %mixGaussEst.mean(:,cGauss)
     %mixGaussEst.cov(:,:,cGauss)
   end;
   
   l=logLike;
   logLike = getMixGaussLogLike(data,mixGaussEst);
   if logLike-l<10
   %if cIter==10
       continuenext=0;
   end;
   fprintf('Log Likelihood Iter %d : %4.3f\n',cIter,logLike);
   cIter=cIter+1;

end;


%==========================================================================
%==========================================================================

%the goal of this routine is to calculate the log likelihood for the whole
%data set under a mixture of Gaussians model. We calculate the log as the
%likelihood will probably be a very small number that Matlab may not be
%able to represent.

%==========================================================================
function logLike = getMixGaussLogLike(data,mixGaussEst);

%find total number of data items
nData = size(data,2);

%initialize log likelihoods
logLike = 0;
like = zeros(nData,1);
%run through each data item
for (i=1:mixGaussEst.k)
    like=like+mixGaussEst.weight(i)*calcGaussianProb(data,mixGaussEst.mean(:,i),mixGaussEst.cov(:,:,i));
end;
logLike = sum(log(like));        

%==========================================================================
function like = calcGaussianProb(data,gaussMean,gaussCov) %data:3*n

[nDim nData] = size(data);
like = 1/((2*pi)^(nDim/2)*sqrt(det(gaussCov)))*exp(sum(-0.5*(data-gaussMean).'*inv(gaussCov).*(data-gaussMean).',2));


function like = getMixGaussLike(data,mixGaussEst);

%find total number of data items
nData = size(data,2);

like = zeros(nData,1);
%run through each data item
for (i=1:mixGaussEst.k)
    like=like+mixGaussEst.weight(i)*calcGaussianProb(data,mixGaussEst.mean(:,i),mixGaussEst.cov(:,:,i));
end;
%--------------------------------------------------------------------------

function auc=calcROC(y,posterior) %y:n*1, posterior:ndata*1

%threshold=posterior'; %threshold: 1*ndata
threshold=linspace(0,1,100);
ys=posterior>=threshold;

tp=sum(ys(find(y),:)); %tp: 1*n
fn=sum(ys(find(y),:)==0); 
fp=sum(ys(find(y==0),:));
tn=sum(ys(find(y==0),:)==0);

tpr=tp./(tp+fn);%roc curve y
fpr=fp./(fp+tn);%roc curve x
%find the best threshold
acc=(tp+tn)./(tp+tn+fp+fn);
in=find(acc==max(acc));
maxth=threshold(in)

f1=fpr(1,1:99);
f2=fpr(1,2:100);
deltaf=f2-f1;
auc=abs(tpr(1,1:99)*deltaf');
%plot roc curve
figure;
title('ROC')
plot(fpr,tpr,threshold,threshold,'--')


%plot result function
function plotapple(image, posterior)

[Y X Z] = size(image);
posterior=reshape(posterior,Y,X);
clims = [0, 1];
figure;
%posterior=posterior>0.5;
posterior=posterior>0.7;
subplot(1,2,1);
title('Original ')
imshow(image)
subplot(1,2,2);
title('Apple pixels ')
imagesc(posterior, clims); colormap(gray); axis off; axis image;












