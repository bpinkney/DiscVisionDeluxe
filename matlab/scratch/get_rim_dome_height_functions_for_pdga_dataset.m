
% try to find some crap mapping from the PDGA 
% 'Rim Configuration' (blunt edge area measureed by a contour gauge)
% to the actual dome height and lowerf rim camber heights
% ideally this would loosely correlate to other supplied aspects of the
% disc; use our existing (manually measured/validated dataset) to formulate
% this relationship. Neural nets would be funny here....
% We need to make sure discs with generated dome/rim heights are flagged as
% 'unverified', so they can be updated later

clc;
clear all;

thickness = [ ...
20
15
15.5
16.5
15.5
19.25
17
16.75
19
17
18.5
15
18.5
15.1
17
14.5
17.5
19.5
19
15.5
16
22
16
16
15
16.25
17.5
15
18
15
17
20
15.9
18
20
14.5
19
18
19.5
22
8.5
19
12.25
20.75
16.2
16.5
15
15.25
15
15
];

rim_width = [...
10.25
24.5
21
24.5
24.5
12.25
24.5
17.5
9.8
19.5
13
24
22
16.8
12.5
21
20.5
12
12
23.5
19.25
13.5
21
17.8
19
19.25
10.5
19.25
22
19
19
10
12.3
13
10
23.5
10
19
10.8
9
8.5
13.25
9
9
15
11.5
19.75
19.5
21.5
22.75
];

rim_height = [...
8
4.75
6.25
6
5
7.5
3.75
6
-1
7
8.25
4
5.5
7
8
5.75
5.5
7
6.75
4.25
5
5.75
6
-1
6
5.5
6
5.5
5
7
4.75
5.5
-1
7
6.5
5.5
6
4
-1
7.5
-1
6
5
-1
-1
8
8
8.5
7
5.75
];

dome_height = [...
2.75
4
4
2
3.25
2.5
5
6
-1
3
4.5
3.5
4.5
2.5
0.75
2.75
7
3
3.25
5.25
4
8
2.75
-1
4
5.25
1
2.75
6
4
5
1.5
-1
2
2.25
3
2.75
5.5
-1
4
-1
3.5
0.5
-1
-1
0
2.5
3
3.5
4.5
];

rim_depth = [...
13
11.5
11.75
11.5
11.5
13
12
11.5
15.6
11.75
15
11.5
12.5
11
13
11.5
11.75
12.75
12.75
11
11
13.5
11.5
11.3
12
11
14
11.5
12
11.75
11.5
15.25
12.7
13
15
11.25
15
13
14.3
14
6.25
12.25
9
15
12.3
14
12
11.5
12
11.25
];

diameter = [...
219.5
210.25
210
210.5
210.5
217
209
209
212.2
212
217
209
210
211.2
217
210
208.5
213
214.25
211.5
207.25
208.25
209
211.4
212
208
212
209
210
213
210.5
216
214.4
220
210
210
211
210
210.2
211
70
214
150
210
215.4
214
212
214
210.5
210.5
];

blunt_edge = thickness - dome_height - rim_height;



%% assume we only have access to:
% - rim depth
% - rim width
% - blunt edge height
% - thickness
% - diameter

% and we need to derive the rim_height and dome_height
% or rather, the dome/rim height factor (determines what the split is for
% anything left over from 'thickness - blunt edge height'

dome_o_rim = dome_height ./ (dome_height + rim_height);

idx = rim_height ~= -1;
% get rid of buzzz mini and marker
[~, buzzzmini] = min(abs(diameter - 70));
idx(buzzzmini) = 1<0;
[~, buzzzmini] = min(abs(diameter - 150));
idx(buzzzmini) = 1<0;

figure(1); hold on; grid on;
% plot3(rim_width(idx), diameter(idx), dome_o_rim(idx), 'o')


X = rim_depth(idx);
Y = diameter(idx);
Z = dome_o_rim(idx);   

% X_nodes = linspace(min(X), max(X), 50);
% Y_nodes = linspace(min(Y), max(Y), 50);
% 
% disp('Get Grid Points');
% g = gridfit(X,Y,Z,X_nodes,Y_nodes, 'smoothness', [0.05, 0.05]);
% 
% disp('Plot Surface');
xlabel('rim_width'); 
ylabel('diameter'); 
zlabel('dome_height / rim_height factor')
%         
% surf(X_nodes,Y_nodes,g,...
%     'AlphaData',ones(size(g))*0.7, 'alphadatamapping', 'none',...
%     'FaceAlpha','flat');

plot3(X, Y, Z, 'o');

n = 1;
m = 1;
% p = polyFit2D(Z,X,Y,n,m)

% try with exact inputs
% Z_fit = polyVal2D(p,X,Y,n,m);

p = polyfit(Y, Z, 2)
Z_fit = polyval(p,Y);

plot3(X, Y, Z_fit, '.')

figure(2); hold on; grid on;
plot(Z - Z_fit, 'LineWidth', 2);
plot(Z, 'o');
plot(Z_fit, '.');
title('dome_height / rim_height factor error to fit')
























