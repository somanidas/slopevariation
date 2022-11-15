clc
close alL

%%READING THE IMAGE 
%to read the image

I = imread('C:\MASTERS\MINOR PROJECT\MINOR PROJECT-IMAGES\SPINE IMAGES\types-of-scoliosis\dataset\OLD IMAGES\ex\c.5.png');
figure
imshow(I);%%display the image
hold on;
%To get the number of segments of the image
[first,second,third]=size(VertebraeROI)

%%Calculating the slope

for i = (1:third)
    
  orderedPoly = orderVertices((VertebraeROI(:,:,i)));  
  drawpolygon('Position', orderedPoly, 'Deletable', false, 'FaceSelectable', false, 'InteractionsAllowed', 'none');
  
  p1=orderedPoly(1,:)%taking the top left point
  p2=orderedPoly(4,:)%getting the bottom left point
  c1 = drawpoint('Position',p1,'Color','red');
  c2 = drawpoint('Position',p2,'Color','red');
  xcordinates=[p1(1) p2(1)]%storing te x coordinates of the points
  ycordinates=[p1(2) p2(2)]%storing y coordinates
  xdiff= xcordinates(2)-xcordinates(1);%diff in x coordinates
  ydiff= ycordinates(2)-ycordinates(1);%diff in y coordinates

  Slopeofeachroi(i)=ydiff/xdiff;%slope=ydiff/xdiff
 
end
 display( Slopeofeachroi)
%%END OF SLOPE CALCULATION
%% DETECTING THE SIGN CHANGE


index=zeros(1,third)%created a matrix index to store the index positions of sign change
for i=1:third-1
         if sign(  Slopeofeachroi(i))~=sign(  Slopeofeachroi(i+1))%%Checking the sign change by taking the elements at  index positions i and i+1
            %% STORING THE INDEX POSITIONS IN INDEX MATRIX
            index(i)=i+1;        
         end
end
       
   indexofvertebraechange=find(index)
   changedvertebraeno=index(indexofvertebraechange)
 changedvertebraeno(2)=indexofvertebraechange(:,2)%To get the tilted  vertebrae in the bottom 
   display(changedvertebraeno)

   %%TO DRAW THE LINE 
   POINTS=orderVertices(VertebraeROI(:,:,changedvertebraeno(:,1)));
   POINTS2=orderVertices(VertebraeROI(:,:,changedvertebraeno(:,2)));
   vertebrae1topleftpoint = POINTS(1, :);
   vertebrae1toprightpoint = POINTS(2, :);
   vertebrae2bottomleftpoint = POINTS2(4, :);
   vertebrae2bottomrightpoint = POINTS2(3, :);
   
   drawpolygon('Position', POINTS, 'Deletable', false, 'FaceSelectable', false, 'InteractionsAllowed', 'none', 'Color', 'red');
   drawpolygon('Position', POINTS2, 'Deletable', false, 'FaceSelectable', false, 'InteractionsAllowed', 'none', 'Color', 'red'); 
   drawpoint('Position',vertebrae1topleftpoint,'Color','green');
   drawpoint('Position',vertebrae1toprightpoint,'Color','green');
   drawpoint('Position',vertebrae2bottomleftpoint,'Color','green');
   drawpoint('Position',vertebrae2bottomrightpoint,'Color','green');
  
   % DRAWING LINE
   L1XCoords=[vertebrae1topleftpoint(1),vertebrae1toprightpoint(1)];
   L1YCoords =[vertebrae1topleftpoint(2),vertebrae1toprightpoint(2)]; %% coordinates to plot the line 1 using the top left and top right points
   L2XCoords=[vertebrae2bottomleftpoint(1), vertebrae2bottomrightpoint(1)];
   L2YCoords=[vertebrae2bottomleftpoint(2), vertebrae2bottomrightpoint(2)];%%coordinates to plot line2 bottom left and right points
   
   xlim = get(gca,'XLim');
   m = (L1YCoords(2)-L1YCoords(1))/(L1XCoords(2)-L1XCoords(1));
   L1SLOPE=m
   n = L1YCoords(2) - L1XCoords(2)*m;
   y1 = m*xlim(1) + n;
   y2 = m*xlim(2) + n;
   line([xlim(1) xlim(2)],[y1 y2], 'Color', 'yellow')
 
  
   m = (L2YCoords(2)-L2YCoords(1))/(L2XCoords(2)-L2XCoords(1));
    L2SLOPE=m;
   n = L2YCoords(2) - L2XCoords(2)*m;
   y1 = m*xlim(1) + n;
   y2 = m*xlim(2) + n;
   line([xlim(1) xlim(2)],[y1 y2], 'Color', 'yellow');
   
   
   %%CALCULATION OF COBBS ANGLE
   
    angle1 = atan2(L1YCoords(2)-L1YCoords(1), L1XCoords(2)-L1XCoords(1));
        angle2 = atan2(L2YCoords(2)-L2YCoords(1), L2XCoords(2)-L2XCoords(1));
        angle = (angle2 - angle1) * 180/pi;
        if(angle > 90)
          angle = 180 - angle; 
        end


tantheta=abs((L2SLOPE-L1SLOPE)/(1+(L1SLOPE*L2SLOPE)))
theta=atand(tantheta)
display(theta)
%%END OF COBBS ANGLE CALCULATION
%%
%%DETECTING THE REGION OF BEND
if changedvertebraeno(1)>=1 && changedvertebraeno(2)<8
        string="bend is in thoracic region"
       
    end


  if changedvertebraeno(1)<8&& changedvertebraeno(2)<=third
          string="bend is in thoracic lumbar region"
           
      end
  
  if changedvertebraeno(1)>=10&&changedvertebraeno(2)==third
          
          
          string="bend is in lumbar region"
       
  end
 
  
  %%END OF DETECTION OF REGION OF BEND
  
  %%TO DISPLAY THE RESULT 
 
 str_1 = ['Cobb Angle  =' num2str(angle) string];
        msgbox(str_1);
%%END OF THE DISPLAY OF THE FINAL RESULT
   
   %% GIVEN poly points, order them in clockwise direction
   function result = orderVertices(poly)
   %poly is a 4x2 matrix
     p1 = poly(1, :);
     p2 = poly(2, :);
     p3 = poly(3, :);
     p4 = poly(4, :);
     
     X = [p1(1), p2(1), p3(1), p4(1)]; % All X coordinates of the 4 points of the poly
     Y = [p1(2), p2(2), p3(2), p4(2)]; % All Y coordinates of the 4 points of the poly
     
     % Random initial values
     first_smallest_x_index = 1;
     first_smallest_x_value = 99999999;
     second_smallest_x_index = 1;
     second_smallest_x_value = 999999999;
     
     first_largest_x_index = 1;
     first_largest_x_value = 1;
     second_largest_x_index = 1;
     second_largest_x_value = 1;
     
     for j = (1:4)
         if (X(j) <= first_smallest_x_value)    % If new number smallest than 1st smallest, move 1st smallest to 2nd smallest, and assign new number to 1st smallest
             second_smallest_x_index = first_smallest_x_index;
             second_smallest_x_value = first_smallest_x_value;
             first_smallest_x_index = j;
             first_smallest_x_value = X(j);
         elseif (X(j) < second_smallest_x_value) % If new number only smaller than 2nd smallest, assign new number to 2nd smallest
             second_smallest_x_index = j;
             second_smallest_x_value = X(j);
         end     
         
         if (X(j) >= first_largest_x_value)    % If new number larger than 1st largest, move 1st largest to 2nd largest, and assign new number to 1st largest
             second_largest_x_index = first_largest_x_index;
             second_largest_x_value = first_largest_x_value;
             first_largest_x_index = j;
             first_largest_x_value = X(j);
         elseif (X(j) > second_largest_x_value) % If new number only larger than 2nd largest, assign new number to 2nd largest
             second_largest_x_index = j;
             second_largest_x_value = X(j);
         end     
     end
     
     left_top_point = [0,0];
     left_bottom_point = [0,0];
     right_top_point = [0,0];
     right_bottom_point = [0,0];
     % Left index computation
     if (Y(first_smallest_x_index) < Y(second_smallest_x_index))
         left_top_point = [X(first_smallest_x_index), Y(first_smallest_x_index)];
         left_bottom_point = [X(second_smallest_x_index), Y(second_smallest_x_index)];
     else
         left_bottom_point = [X(first_smallest_x_index), Y(first_smallest_x_index)];
         left_top_point = [X(second_smallest_x_index), Y(second_smallest_x_index)];
     end    
     
     % Right index computation
     if (Y(first_largest_x_index) < Y(second_largest_x_index))
         right_top_point = [X(first_largest_x_index), Y(first_largest_x_index)];
         right_bottom_point = [X(second_largest_x_index), Y(second_largest_x_index)];
     else
         right_bottom_point = [X(first_largest_x_index), Y(first_largest_x_index)];
         right_top_point = [X(second_largest_x_index), Y(second_largest_x_index)];
     end    
     
 result = [left_top_point; right_top_point; right_bottom_point; left_bottom_point];
   end
   
 