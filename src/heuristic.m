% � Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 
% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using A* algorithm, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function h=heuristic(X,goal)
h = sqrt(sum((X-goal).^2));