% This script takes in a PNG file. 
% Equalises all channels to their average (makes things greyscale)
% Sets extreame values to 0 or 255. Scales values in between to a specific
% range

[file,path] = uigetfile('*.png')

% values outside bw_range are ceiled or floored,
bw_range=[5, 254]
% values inside bw_range are rescaled to float_range
float_range=[100, 254]% I think upper val must b e 254 otherwise freespace- terrain boundary will have a jump
baseMap = imread([path file] );
% alloc for result
resultMap=baseMap;
resultMap(:)=0;


avg=double(mean(baseMap,3));
avg(avg<bw_range(1))=0;
avg(avg>bw_range(2))=255;

%Scaling for float vals
avg(avg>=bw_range(1) & avg<=bw_range(2))=...
    round(float_range(1)+...
    (float_range(2)-float_range(1))*...
    (avg(avg>=bw_range(1) & avg<=bw_range(2))-bw_range(1))...
    /(bw_range(2)-bw_range(1)))

resultMap(:,:,1)=uint8(avg);
resultMap(:,:,2)=uint8(avg);
resultMap(:,:,3)=uint8(avg);

imshow(resultMap)
imwrite(resultMap, [path file])


