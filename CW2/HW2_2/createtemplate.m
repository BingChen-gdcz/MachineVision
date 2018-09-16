function [ output_args ] = createtemplate( input_args )
Im1 = double( imread( 'WechatIMG2021515954795_.pic.jpg' ));
imagesc(Im1/255);
Im1=Im1(:,:,1);

H = imrect;
pos = wait(H);
pos = round(pos);

maskSearchWhat = uint8(zeros(size(Im1, 1), size(Im1, 2), 1));
maskSearchWhat( pos(2):pos(2)+pos(4), pos(1):pos(1)+pos(3) ) = 1;
[Ys Xs] = find(maskSearchWhat == 1);
minXs = min(Xs); 
maxXs = max(Xs);    
minYs = min(Ys); 
maxYs = max(Ys);
pixelsTemplate = Im1(minYs:maxYs,   minXs:maxXs, :);  % Need colRange, rowRange

% Saving these off for the Lab practical
 minY = minYs; minX = minXs;
 patchOffset=round(size(pixelsTemplate)/2);
 save('angle4', 'pixelsTemplate', 'minY', 'minX','pos','patchOffset') %angle1,2,3,4


