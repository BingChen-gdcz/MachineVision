function trackingandhomographytriangle

%LLs = HW2_Practical9c( 'll' );
%LRs = HW2_Practical9c( 'lr' );
%ULs = HW2_Practical9c( 'ul' );
%URs = HW2_Practical9c( 'ur' );
%angle1 = HW2_Practical9c( 'angle1' );
%angle2 = HW2_Practical9c( 'angle2' );
%angle3 = HW2_Practical9c( 'angle3' );
%angle4 = HW2_Practical9c( 'angle4' );

%Adding the edge detector
angle1 = candedgedetector2( 'angle1' );
angle2 = candedgedetector2( 'angle2' );
angle3 = candedgedetector2( 'angle3' );
angle4 = candedgedetector2( 'angle4' );
close all;

% Load frames from the whole video into Imgs{}.
% This is really wasteful of memory, but makes subsequent rendering faster.
LoadVideoFrames2

% Coordinates of the known target object (a dark square on a plane) in 3D:
XCart = [0     -40  -20   40;...
         -47   0     60     0;...
         0     0      0     0];

% Define 3D points of wireframe object.
XWireFrameCart = [0     -40  -20   40 0;...
                  -47    0     60   0 0;...
                   0     0      0   0 40];
hImg = figure;
       
% ================================================
for iFrame = 1:numFrames
    xImCart = [ angle1(iFrame,:)' angle2(iFrame,:)' angle3(iFrame,:)' angle4(iFrame,:)'];
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
    %T = estimatePlanePose(xImCart, XCart, K);
    H=calcHomography(xImCart, XCart(1:2,:));



    %TO DO Draw a wire frame cube, by projecting the vertices of a 3D cube
    %through the projective camera, and drawing lines betweeen the 
    %resulting 2d image points

    %XWireFrameHom=[XWireFrameCart;ones(1,size(XWireFrameCart,2))];
    %XImWireFrameHom=K*T(1:3,:)*XWireFrameHom;
    %XImWireFrameCart=XImWireFrameHom(1:2,:)./repmat(XImWireFrameHom(3,:),2,1);
    %nPoint = size(XImWireFrameCart,2);
    XImWireFrameHom=H*XWireFrameHom;
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
                plot([XImWireFrameCart(1,cPoint) XImWireFrameCart(1,dPoint)],[XImWireFrameCart(2,cPoint) XImWireFrameCart(2,dPoint)],'g-');
    %make sure we don't replace with next point
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

function H = calcHomography(pts1Cart, pts2Cart)

%H = zeros(3);
H = eye(3);
%**** TO DO ****;
%first turn points to homogeneous
%then construct A matrix which should be (10 x 9) in size
%solve Ah = 0 by calling
%h = solveAXEqualsZero(A); (you have to write this routine too - see below)
pts1Hom = [pts1Cart; ones(1,size(pts1Cart,2))]';
ndata=size(pts1Cart,2);
y=[zeros(ndata,3),-pts1Hom,[pts2Cart(2,:)]'.*[pts1Hom]];
x=[pts1Hom,zeros(ndata,3),[pts2Cart(1,:)]'.*[-pts1Hom]];

A=zeros(2*ndata,9);
for i=1:ndata
    n=2*i-1;
    n2=2*i;
    A(n,:)=y(i,:);
    A(n2,:)=x(i,:);
end;
    
h = solveAXEqualsZero(A);

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




