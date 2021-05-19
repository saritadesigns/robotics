clc;
clear all;
close all;

I = imread('sobel_image.jpg');
I = sobel_operator(I,70);
imsshow(I);

% [R, G, B] = imsplit(I);
% threshold = 70; %50, 70, 125, 255
% 
% R_output = sobel_operator(R,threshold);
% G_output = sobel_operator(G,threshold);
% B_output = sobel_operator(B,threshold);
% 
% % final_output = cat(3, R_output, G_output, B_output);
% 
% % imshow(I);
% imshow(R_output);
% % imshow(final_output);