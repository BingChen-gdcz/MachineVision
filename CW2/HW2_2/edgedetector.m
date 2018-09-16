function edgePixels = edgedetector(img)
%Using prewitt filter 

%LoadVideoFrames
%iTime=7;
%img = double( Imgs{iTime} );
img2=mat2gray(img);
s=size(img2);
rows=s(1);col=s(2);
prewitt=img2;
x=img2(:,3:col)-img2(:,1:col-2);
y=img2(1:rows-2,:)-img2(3:rows,:);
gx=x(1:rows-2,:)+x(2:rows-1,:)+x(3:rows,:);
gy=y(:,1:col-2)+y(:,2:col-1)+y(:,3:col);
prewitt(2:rows-1,2:col-1)=abs(gx+gy);
threshold=0.15;
edgePixels=(prewitt>threshold);%*255;
%edgeindex=find(prewittfilter>0);
%figure
%imshow(img2)


