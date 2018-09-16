function HW2_TrackingAndHomographies


LLs = HW2_Practical9c( 'll' );
LRs = HW2_Practical9c( 'lr' );
ULs = HW2_Practical9c( 'ul' );
URs = HW2_Practical9c( 'ur' );

%practical9candedgedetector.m is the function adding the edge detector
%LLs = practical9candedgedetector( 'll' );
%LRs = practical9candedgedetector( 'lr' );
%ULs = practical9candedgedetector( 'ul' );
%URs = practical9candedgedetector( 'ur' );

close all;

% Load frames from the whole video into Imgs{}.
% This is really wasteful of memory, but makes subsequent rendering faster.
LoadVideoFrames

% Coordinates of the known target object (a dark square on a plane) in 3D:
XCart = [-50 -50  50  50;...
          50 -50 -50  50;...
           0   0   0   0];

% These are some approximate intrinsics for this footage.
K = [640  0    320;...
     0    512  256;
     0    0    1];

% Define 3D points of wireframe object.
XWireFrameCart = [-50 -50  50  50 -50 -50  50  50;...
                   50 -50 -50  50  50 -50 -50  50;...
                    0   0   0   0 -100 -100 -100 -100];
 
hImg = figure;
       
% ================================================
for iFrame = 1:numFrames
    xImCart = [LLs(iFrame,:)' ULs(iFrame,:)' URs(iFrame,:)' LRs(iFrame,:)'];
    xImCart = circshift( xImCart, 1);

    % To get a frame from footage 
    im = Imgs{iFrame};

    % Draw image and 2d points
    set(0,'CurrentFigure',hImg);
    set(gcf,'Color',[1 1 1]);
    imshow(im); axis off; axis image; hold on;
    plot(xImCart(1,:),xImCart(2,:),'r.','MarkerSize',15);
    title(sprintf( 'Homography\n(Frame %d)', iFrame));


    %TO DO Use your routine to calculate TEst the extrinsic matrix relating the
    %plane position to the camera position.
    T = estimatePlanePose(xImCart, XCart, K);



    %TO DO Draw a wire frame cube, by projecting the vertices of a 3D cube
    %through the projective camera, and drawing lines betweeen the 
    %resulting 2d image points

    XWireFrameHom=[XWireFrameCart;ones(1,size(XWireFrameCart,2))];
    XImWireFrameHom=K*T(1:3,:)*XWireFrameHom;
    XImWireFrameCart=XImWireFrameHom(1:2,:)./repmat(XImWireFrameHom(3,:),2,1);
    nPoint = size(XImWireFrameCart,2);

    hold on;
    
    % TO DO: Draw a wire frame cube using data XWireFrameCart. You need to
    % 1) project the vertices of a 3D cube through the projective camera;
    % 2) draw lines betweeen the resulting 2d image points.
    % Note: CONDUCT YOUR CODE FOR DRAWING XWireFrameCart HERE
    for (cPoint = 1:nPoint)
        for (dPoint = 1:nPoint)
        %plot a green line between each pair of points
            if sum(XWireFrameCart(:,cPoint)==XWireFrameCart(:,dPoint))==2
                plot([XImWireFrameCart(1,cPoint) XImWireFrameCart(1,dPoint)],[XImWireFrameCart(2,cPoint) XImWireFrameCart(2,dPoint)],'g-');
    %make sure we don't replace with next point
            end;
            hold on;
        end;
    end;
    plot(XImWireFrameCart(1,:),XImWireFrameCart(2,:),'g.','MarkerSize',20);


    hold off;
    drawnow;
    
%     Optional code to save out figure
%     pngFileName = sprintf( '%s_%.5d.png', 'myOutput', iFrame );
%     print( gcf, '-dpng', '-r80', pngFileName ); % Gives 640x480 (small) figure

    
end % End of loop over all frames.
% ================================================
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

% ================================================

% TO DO: QUESTIONS TO THINK ABOUT...

% Q: Do the results look realistic?
% If not then what factors do you think might be causing this


% TO DO: your routines for computing a homography and extracting a 
% valid rotation and translation GO HERE. Tips:
%
% - you may define functions for T and H matrices respectively.
% - you may need to turn the points into homogeneous form before any other
% computation. 
% - you may need to solve a linear system in Ah = 0 form. Write your own
% routines or using the MATLAB builtin function 'svd'. 
% - you may apply the direct linear transform (DLT) algorithm to recover the
% best homography H.
% - you may explain what & why you did in the report.

