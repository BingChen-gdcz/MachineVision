function K = learningK()


%close all open figures
close all;

%load in the required data
load('PracticalDataSm','im1','im2','im3','pts1','pts2','pts3','pts1b');
%im1 is center image with grey background
%im2 is left image 
%pts1 and pts2 are matching points between image1 and image2
%im3 is right image
%pts1b and pts3 are matching points between image 1 and image 3

%show images and points
figure; set(gcf,'Color',[1 1 1]);image(uint8(im1));axis off;hold on;axis image;
plot(pts1(1,:),pts1(2,:),'r.'); 
plot(pts1b(1,:),pts1b(2,:),'m.');
figure; set(gcf,'Color',[1 1 1]);image(uint8(im2));axis off;hold on;axis image;
plot(pts2(1,:),pts2(2,:),'r.'); 
figure; set(gcf,'Color',[1 1 1]);image(uint8(im3));axis off;hold on;axis image;
plot(pts3(1,:),pts3(2,:),'m.'); 

im1b=im1;
%****TO DO**** 
%calculate homography from pts1 to pts2
H = calcBestHomography(pts1, pts2);

%****TO DO**** 
%for every pixel in image 1
    %transform this pixel position with your homography to find where it 
    %is in the coordinates of image 2
    %if it the transformed position is within the boundary of image 2 then 
        %copy pixel colour from image 2 pixel to current position in image 1 
        %draw new image1 (use drawnow to force it to draw)
    %end
%end;



[dim,ndata,c]=size(im1);
[dim2,ndata2,c]=size(im2);
%x1=reshape((1:dim).*ones(ndata,dim),[1,dim*ndata]);
%x2=repmat((1:ndata),1,dim);
%x3=ones(1,dim*ndata);
%x=[x1;x2;x3];

%ptx2=H*x;
%ptx2=ptx2(1:2,:)./repmat(ptx2(3,:),2,1);
%pt12=(ptx2>=0)+[(ptx2(1,:)<=dim2);(ptx2(2,:)<=ndata2)];
%indice=find(pt12(1,:)+pt12(2,:)==4);
%i=ceil(indice/ndata);
%j=indice-(i-1)*ndata;
%in2=ceil(ptx2(:,indice));
%for l=size(i,2)
 %   im1(i(l),j(l),:)=im2(in2(1,l),in2(2,l),:);
  %  set(gcf,'Color',[1 1 1]);image(uint8(im1));axis off;hold on;axis image;
   % drawnow
%end;

for p = 1:dim*ndata
    j = ceil(p/ndata);
    i = p-(j-1)*ndata;
    w=[i,j,1]';
    x=H*w;
    x=x(1:2,:)./x(3,:);
    if x(2)>0 & x(2)<=dim2 & x(1)>0 & x(1)<=ndata2
        im1(j,i,:)=im2(ceil(x(2)),ceil(x(1)),:);
    end;
end;
    
%for i=1:dim
 %   for j=1:ndata
  %      hh=(i-1)*ndata+j;
   %     xy=ptx2(:,hh);
    %    if xy(1)>0 & xy(1)<dim2 & xy(2)>0 & xy(2)<ndata2

     %       im1(i,j,:)=im2(ceil(xy(1)),ceil(xy(2)),:);
            %figure; set(gcf,'Color',[1 1 1]);image(uint8(im1));
       %     drawnow
        %end;
            
    %end;
%end;
figure; set(gcf,'Color',[1 1 1]);image(uint8(im1));axis off;hold on;axis image;


%****TO DO****
%repeat the above process mapping image 3 to image 1.

H2 = calcBestHomography(pts1b, pts3);

[dim3,ndata3,c]=size(im3);

for p2 = 1:dim*ndata
    j2 = ceil(p2/ndata);
    i2 = p2-(j2-1)*ndata;
    w2=[i2,j2,1]';
    x2=H2*w2;
    x2=x2(1:2,:)./x2(3,:);
    if x2(2)>0 & x2(2)<=dim3 & x2(1)>0 & x2(1)<=ndata3
        im1(j2,i2,:)=im3(ceil(x2(2)),ceil(x2(1)),:);
    end;
end;

figure; set(gcf,'Color',[1 1 1]);image(uint8(im1));axis off;hold on;axis image;

%==========================================================================
%==========================================================================
function H = calcBestHomography(pts1, pts2)

%should apply direct linear transform (DLT) algorithm to calculate best
%homography that maps the points in pts1Cart to their corresonding matchin in 
%pts2Cart


%****TO DO ****: replace this
H = zeros(3);

%**** TO DO ****;
%first turn points to homogeneous
%then construct A matrix which should be (10 x 9) in size
%solve Ah = 0 by calling
%h = solveAXEqualsZero(A); (you have to write this routine too - see below)
ndata=size(pts1,2);
pts1Hom = [pts1; ones(1,ndata)]';

y=[zeros(ndata,3),-pts1Hom,[pts2(2,:)]'.*[pts1Hom]];
x=[pts1Hom,zeros(ndata,3),[pts2(1,:)]'.*[-pts1Hom]];
%A=[y;x];
A=zeros(2*ndata,9);
for i=1:ndata
    n=2*i-1;
    n2=2*i;
    A(n,:)=y(i,:);
    A(n2,:)=x(i,:);
end;
    
h = solveAXEqualsZero(A)

%reshape h into the matrix H
H=reshape(h,[3,3])';
%Beware - when you reshape the (9x1) vector x to the (3x3) shape of a homography, you must make
%sure that it is reshaped with the values going first into the rows.  This
%is not the way that the matlab command reshape works - it goes columns
%first.  In order to resolve this, you can reshape and then take the
%transpose


%==========================================================================
function x = solveAXEqualsZero(A);
[U L V]=svd(A);
x=V(:,size(V,2));
%****TO DO **** Write this routine 




