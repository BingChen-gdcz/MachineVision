% LoadApplesScript.m
% This optional script may help you get started with loading of photos and masks.
%
% Note that there are more elegant ways to organize your (photo, mask)
% pairs, and for sorting through many files in a directory. We don't use
% them here because we only have a small number of files, but consider
% useful functions like fileparts(). For simplicity, this example code just
% makes a few cell-arrays to hold the hard-coded filenames.

if( ~exist( 'apples', 'dir') || ~exist('testApples', 'dir') )
    display('Please change current directory to the parent folder of both apples/ and testApples/');
end

% Note that cells are accessed using curly-brackets {} instead of parentheses ().
Iapples = cell(3,1);
Iapples{1} = 'apples/Apples_by_kightp_Pat_Knight_flickr.jpg';
Iapples{2} = 'apples/ApplesAndPears_by_srqpix_ClydeRobinson.jpg';
Iapples{3} = 'apples/bobbing-for-apples.jpg';

IapplesMasks = cell(3,1);
IapplesMasks{1} = 'apples/Apples_by_kightp_Pat_Knight_flickr.png';
IapplesMasks{2} = 'apples/ApplesAndPears_by_srqpix_ClydeRobinson.png';
IapplesMasks{3} = 'apples/bobbing-for-apples.png';


iImage = 1; % Could use this index to loop.
curI = double(imread(  Iapples{iImage}   )) / 255;
% curI is now a double-precision 3D matrix of size (width x height x 3). 
% Each of the 3 color channels is now in the range [0.0, 1.0].
figure;
imagesc(curI)


curImask = imread(  IapplesMasks{iImage}   );
% These mask-images are often 3-channel, and contain grayscale values. We
% would prefer 1-channel and just binary:
curImask = curImask(:,:,2) > 128;  % Picked green-channel arbitrarily.
figure;
imshow(curImask)

nGaussEst = 2;

%fit mixture of Gaussians
%TO DO fill in this routine (below)
data=reshape(curI,397*500,3);
data=data'
mixGaussEst = fitMixGauss(data,nGaussEst);

%==========================================================================
%==========================================================================

function mixGaussEst = fitMixGauss(data,k);
        
[nDim nData] = size(data);

%MAIN E-M ROUTINE 
%there are nData data points, and there is a hidden variable associated
%with each.  If the hidden variable is 0 this indicates that the data was
%generated by the first Gaussian.  If the hidden variable is 1 then this
%indicates that the hidden variable was generated by the second Gaussian
%etc.

postHidden = zeros(k, nData);

%in the E-M algorithm, we calculate a complete posterior distribution over
%the (nData) hidden variables in the E-Step.  In the M-Step, we
%update the parameters of the Gaussians (mean, cov, w).  

%we will initialize the values to random values
mixGaussEst.d = nDim;
mixGaussEst.k = k;
mixGaussEst.weight = (1/k)*ones(1,k);
%mixGaussEst.mean = 2*randn(nDim,k);
mixGaussEst.mean = 15*rand(nDim,k);

for (cGauss =1:k)
    %mixGaussEst.cov(:,:,cGauss) = (0.5+1.5*rand(1))*eye(nDim,nDim);
    mixGaussEst.cov(:,:,cGauss) = (1.5+1*rand(1))*eye(nDim,nDim);
end;

%calculate current likelihood
%TO DO - fill in this routine
logLike = getMixGaussLogLike(data,mixGaussEst);
fprintf('Log Likelihood Iter 0 : %4.3f\n',logLike);

nIter = 30;
for (cIter = 1:nIter)
   %Expectation step
   
   for (cData = 1:nData)
        %TO DO (g): fill in column of 'hidden' - calculate posterior probability that
        %this data point came from each of the Gaussians
        %replace this:
        prx=0;
        for (i=1:mixGaussEst.k)
            prx=prx+mixGaussEst.weight(i)*calcGaussianProb(data(:,cData),mixGaussEst.mean(:,i),mixGaussEst.cov(:,:,i));
        end;
        
        for (i=1:mixGaussEst.k)            
            postHidden(i,cData) = mixGaussEst.weight(i)*calcGaussianProb(data(:,cData),mixGaussEst.mean(:,i),mixGaussEst.cov(:,:,i))/prx;
        end;
        
        %postHidden(:,cData) = 1/k;
   end;
   
   %Maximization Step
   
   %for each constituent Gaussian
   for (cGauss = 1:k) 
        %TO DO (h):  Update weighting parameters mixGauss.weight based on the total
        %posterior probability associated with each Gaussian. Replace this:
        mixGaussEst.weight(cGauss) = sum(postHidden(cGauss,:))/sum(postHidden(:)); 
        %mixGaussEst.weight(cGauss) = mixGaussEst.weight(cGauss); 
        
        %TO DO (i):  Update mean parameters mixGauss.mean by weighted average
        %where weights are given by posterior probability associated with
        %Gaussian.  Replace this:
        %mixGaussEst.mean(:,cGauss)=(postHidden(cGauss,:)*data')/sum(postHidden(cGauss,:));
        %de=sum(postHidden(cGauss,:));
        %mu=zeros(2,1);
        %for (cData = 1:nData)
         %   mu=mu+postHidden(cGauss,cData)*data(:,cData);
        %end;
        mixGaussEst.mean(:,cGauss)=sum(postHidden(cGauss,:).*data(:,:),2)/sum(postHidden(cGauss,:));
        %mixGaussEst.mean(:,cGauss)=mu/de;
       
        %mixGaussEst.mean(cGauss) = mixGaussEst.mean(cGauss);
        
        %TO DO (j):  Update covarance parameter based on weighted average of
        %square distance from update mean, where weights are given by
        %posterior probability associated with Gaussian
       % mixGaussEst.cov(:,:,cGauss) = mixGaussEst.cov(:,:,cGauss);
        %mixGaussEst.cov(:,:,cGauss) = sum(postHidden(cGauss,:).*...
         %   (data-mixGaussEst.mean(:,cGauss))*(data-mixGaussEst.mean(:,cGauss))',2)...
          %  /sum(postHidden(cGauss,:));
        mixGaussEst.cov(:,:,cGauss)=postHidden(cGauss,:).*(data-mixGaussEst.mean(:,cGauss))*(data-mixGaussEst.mean(:,cGauss))'/sum(postHidden(cGauss,:));
        %de=sum(postHidden(cGauss,:));
        %cov=zeros(2,2,3);
        %for (i=1:nData)
         %   cov(:,:,cGauss)=cov(:,:,cGauss)+postHidden(cGauss,i).*...
          %      (data(:,i)-mixGaussEst.mean(:,cGauss))*(data(:,i)-mixGaussEst.mean(:,cGauss))';
            %mixGaussEst.cov(:,:,cGauss)=mixGaussEst.cov(:,:,cGauss)+postHidden(cGauss,i).*...
            %(data(:,i)-mixGaussEst.mean(:,cGauss))*(data(:,i)-mixGaussEst.mean(:,cGauss))';    
       % end;
        %mixGaussEst.cov(:,:,cGauss)=cov(:,:,cGauss)/de;
         mixGaussEst.weight(cGauss)
     mixGaussEst.mean(:,cGauss)
     mixGaussEst.cov(:,:,cGauss)
   end;
   
   %draw the new solution
   drawEMData2d(data,mixGaussEst);drawnow;

   %calculate the log likelihood
   logLike = getMixGaussLogLike(data,mixGaussEst);
   fprintf('Log Likelihood Iter %d : %4.3f\n',cIter,logLike);

end;


%==========================================================================
%==========================================================================

%the goal of this routine is to calculate the log likelihood for the whole
%data set under a mixture of Gaussians model. We calculate the log as the
%likelihood will probably be a very small number that Matlab may not be
%able to represent.
function logLike = getMixGaussLogLike(data,mixGaussEst);

%find total number of data items
nData = size(data,2);

%initialize log likelihoods
logLike = 0;

%run through each data item
for(cData = 1:nData)
    thisData = data(:,cData);    
    %TO DO - calculate likelihood of this data point under mixture of
    %Gaussians model. Replace this
    like = 0;
    for (i=1:mixGaussEst.k)
        like=like+mixGaussEst.weight(i)*calcGaussianProb(thisData,mixGaussEst.mean(:,i),mixGaussEst.cov(:,:,i));
    
    end;
    %add to total log like
    logLike = logLike+log(like);        
end;

%%self-define
function pr = calcGaussianProb(data,gaussMean,gaussCov)

[nDim nData] = size(gaussMean);
pr = 1/((2*pi)^(nDim/2)*sqrt(det(gaussCov)))*exp(-0.5*(data-gaussMean).'*inv(gaussCov)*(data-gaussMean));

%==========================================================================
end;