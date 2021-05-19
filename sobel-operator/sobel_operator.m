function [output_image] = sobel_operator(A,threshold)
% A is a 2D array
% threshold: varies for application [0 255]
    Gx = [-1 0 1; -2 0 2; -1 0 1];
    Gy = [-1 -2 -1; 0 0 0; 1 2 1];
    rows = size(A, 1);
    columns = size(A, 2);
    mag = zeros(rows,columns);
    A = double(A);
    for i=1:rows-2
        for j=1:columns-2
            S1 = sum(sum(Gx.*A(i:i+2,j:j+2)));
            S2 = sum(sum(Gy.*A(i:i+2,j:j+2)));
            mag(i+1, j+1) = sqrt(S1.^2+S2.^2);
        end  
    end  
    output_image = max(mag, threshold);
    output_image(output_image == round(threshold)) = 0;
end 
