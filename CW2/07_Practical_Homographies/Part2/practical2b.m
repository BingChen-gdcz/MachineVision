function practical2b

%The goal of this part of the practical is to take a real image containing
%a planar black square and figure out the transformation between the square
%and the camera.  We will then draw a wire-frame cube with it's base
%corners at the corner of the square.  You should use this
%template for your code and fill in the missing sections marked "TO DO"

%load in image 
im = imread('test104.jpg');

%define points on image
xImCart = [  140.3464  212.1129  346.3065  298.1344   247.9962;...
             308.9825  236.7646  255.4416  340.7335   281.5895];
         
%define 3D points of plane
XCart = [-50 -50  50  50 0 ;...
          50 -50 -50  50 0;...
           0   0   0   0 0];

%We assume that the intrinsic camera matrix K is known and has values
K = [640  0    320;...
     0    640  240;
     0    0    1];

%draw image and 2d points
figure; set(gcf,'Color',[1 1 1]);
imshow(im); axis off; axis image; hold on;
plot(xImCart(1,:),xImCart(2,:),'r.','MarkerSize',10);
       
%TO DO Use your routine to calculate TEst, the extrinsic matrix relating the
%plane position to the camera position.

TEst = estimatePlanePose(xImCart,XCart,K)



%define 3D points of plane
XWireFrameCart = [-50 -50  50  50 -50 -50  50  50;...
                   50 -50 -50  50  50 -50 -50  50;...
                    0   0   0   0 -100 -100 -100 -100];

%TO DO Draw a wire frame cube, by projecting the vertices of a 3D cube
%through the projective camera and drawing lines betweeen the resulting 2d image
%points

XWireFrameHom=[XWireFrameCart;ones(1,size(XWireFrameCart,2))];
XImWFHom=K*TEst(1:3,:)*XWireFrameHom;
       
xImWFCart=XImWFHom(1:2,:)./repmat(XImWFHom(3,:),2,1);
nPoint = size(xImWFCart,2)
for (cPoint = 1:nPoint)
    for (dPoint = 1:nPoint)
    %plot a green line between each pair of points
        if sum(XWireFrameCart(:,cPoint)==XWireFrameCart(:,dPoint))==2
            
            plot([xImWFCart(1,cPoint) xImWFCart(1,dPoint)],[xImWFCart(2,cPoint) xImWFCart(2,dPoint)],'g-');
    %make sure we don't replace with next point
            hold on;
        end;
    end;
end;

plot(xImWFCart(1,:),xImWFCart(2,:),'g.','MarkerSize',20);


%QUESTIONS TO THINK ABOUT...

%Do the results look realistic?
%If not, then what factors do you think might be causing this?

function T = estimatePlanePose(xImCart,XCart,K)

%replace this
%T = [];

%TO DO Convert Cartesian image points xImCart to homogeneous representation
%xImHom
xImHom=[xImCart;ones(1,size(xImCart,2))];
xHom=[XCart(1:2,:);ones(1,size(XCart,2))];
%TO DO Convert image co-ordinates xImHom to normalized camera coordinates
%xCamHom:x'
xCamHom=K^-1*xImHom;
%TO DO Estimate homography H mapping homogeneous (x,y)
%coordinates of positions in real world to xCamHom.  Use the routine you wrote for
%Practical 1B.
ndata=size(xHom,2);
y=[zeros(ndata,3),-xHom',[xImHom(2,:).*xHom]'];
x=[xHom',zeros(ndata,3),[xImHom(1,:).*-xHom]'];
A=zeros(2*ndata,9);
for i=1:ndata
    n=2*i-1;
    n2=2*i;
    A(n,:)=x(i,:);
    A(n2,:)=y(i,:);
end;
%TO DO Estimate first two columns of rotation matrix R from the first two
%columns of H using the SVD
[U L V]=svd(A);
H=V(:,size(V,2));
H=reshape(H,[3,3])';%phi'
H=K^(-1)*H;
R=zeros(3,3);
[U2 L2 V2]=svd(H(:,1:2));
temp=[1 0; 0 1; 0 0];
R(:,1:2)=U2*temp*V2';

%TO DO Estimate the third column of the rotation matrix by taking the cross
%product of the first two columns
R(:,3)=cross(R(:,1),R(:,2));
%TO DO Check that the determinant of the rotation matrix is positive - if
%not then multiply last column by -1.
if det(R)<0
    R(:,3)=R(:,3)*(-1);
end;
%TO DO Estimate the translation t by finding the appropriate scaling factor k
%and applying it to the third colulmn of H
k=sum(sum(H(:,1:2)./R(:,1:2)))/6;
t=H(:,3)/k;
%TO DO Check whether t_z is negative - if it is then multiply t by -1 and
%the first two columns of R by -1.
if t(3)<0
    t=t*(-1);
    R(:,1:2)=R(:,1:2)*(-1);
end;
%assemble transformation into matrix form
T  = [R t;0 0 0 1];