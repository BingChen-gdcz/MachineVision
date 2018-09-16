function r=practical2

%This project explores the geometry of a single camera. The aim is to take several points on
%a plane, and predict where they will appear in the camera image. Based on these observed
%points, we will then try to re-estimate the Euclidean transformation relating the plane and
%the camera. In practical 2b we will use this code to draw a wireframe cube
%on an augmented reality marker.   You should use this
%template for your code and fill in the missing sections marked "TO DO"


%We assume that the intrinsic camera matrix K is known and has values
K = [640  0    320;...
     0    640  240;
     0    0    1];
 
%We will assume an object co-ordinate system with the Z-axis pointing upwards and the
%origin in the centre of the plane. There are four known points on the plane, with coordinates
%(in mm):
%?four points in a plane?
XCart = [-100 -100  100  100 0 ;...
         -100  100  100 -100 0;...
          0    0    0    0   0 ];

%We will assume that the correct transformation from the plane co-ordinate system to the
%camera co-ordinate system (extrinsic matrix) is:

T = [ 0.9851  -0.0492  0.1619  46.00;...
     -0.1623  -0.5520  0.8181  70.00;...
      0.0490  -0.8324 -0.5518  500.89;...
      0        0       0       1]
  
% TO DO  Use the general pin-hole projective camera model discussed in the lectures to estimate 
%where the four points on the plane will appear in the image.  Fill in the
%details of the function "projectiveCamera" - body of function appears below

xImCart = projectiveCamera(K,T,XCart);

% TO DO Add noise to the pixel positions to simulate having to find these points in a noisy
%image. Store the results back in xImCart.  
%The noise should have standard deviation of one pixel in each direction.

xImCart = xImCart+randn(size(xImCart)); 

%Now we will take the image points and the known positions on the card and try to
%estimate the extrinsic matrix using the algorithm discussed in the lecture. 
%Fill in the details of the function "estimate plane pose" - body of function appears
%below

TEst = estimatePlanePose(xImCart,XCart,K)

%if you have got this correct, it should resemble T above.

%==========================================================================
%==========================================================================

%goal of function is to project points in XCart through projective camera
%defined by intrinsic matrix K and extrinsic matrix T.
function xImCart = projectiveCamera(K,T,XCart);

%replace this
xImCart = [];

%TO DO convert Cartesian 3d points XCart to homogeneous coordinates XHom
XHom=[XCart;ones(1,size(XCart,2))];
%TO DO apply extrinsic matrix to XHom to move to frame of reference of
%camera
XHom=T*XHom;
%TO DO project points into normalized camera coordinates xCamHom by (achieved by
%removing fourth row)
XHom=XHom(1:3,:);
%TO DO move points to image coordinates xImHom by applying intrinsic matrix
XHom=K*XHom;
%TO DO convert points back to Cartesian coordinates xImCart
xImCart=XHom(1:2,:)./repmat(XHom(3,:),2,1);


%==========================================================================
%==========================================================================

%goal of function is to estimate pose of plane relative to camera
%(extrinsic matrix) given points in image xImCart, points in world XCart
%and intrinsic matrix K.--estimate extrinsic matrix
%?xImCart: points in image; XCart: points in plane,w=0; K:intrinsic para? 
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

%==========================================================================
%==========================================================================


